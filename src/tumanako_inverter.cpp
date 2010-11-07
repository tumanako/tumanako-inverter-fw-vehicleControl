//------------------------------------------------------------------------------
//   TumanakoVC - Electric Vehicle and Motor control software
//   Copyright (C) 2009 Philip Court <philip@greenstage.co.nz>
//
//   This file is part of TumanakoVC.
//
//   TumanakoVC is free software: you can redistribute it and/or modify
//   it under the terms of the GNU Lesser General Public License as published
//   by the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
//   TumanakoVC is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU Lesser General Public License for more details.
//
//   You should have received a copy of the GNU Lesser General Public License
//   along with TumanakoVC.  If not, see <http://www.gnu.org/licenses/>.
//
// DESCRIPTION:
//   This class implements the high level inverter and vehicle control logic
//   and maps the software control to the hardware.
//
// HISTORY:
//   Philip Court 5/Aug/2009 - First Cut
//------------------------------------------------------------------------------


//--- Includes -----------------------------------------------------------------
#include <cmath>
#include <time.h>

#include "tumanako_inverter.hpp"
#include "tumanako_serial.hpp"

/*----------------------------- Private typedef ------------------------------*/
/*----------------------------- Private define -------------------------------*/
#define TK_ABS(x) ( x < 0 ? - x :  x)

#define TUMANAKO_MAX_REGEN_POWER 40000000

#define EVAL_BOARD_POT 14 //PC4 (see page 30 UM0426)
#define EVAL_BOARD_MIC 15 //PC5

#ifdef TUMANAKO_KIWIAC
#define ACCEL_AI EVAL_BOARD_POT
#else  //i.e. STM Eval kit and vero board
#define ACCEL_AI EVAL_BOARD_MIC
#endif



#ifdef TUMANAKO_STANDARD

#define THROTTLE_POT_MAX 63000
#define THROTTLE_POT_MIN 0
#define TORQUE_MAX 19000
#define TORQUE_MIN 0
#define ZERO_THROTTLE_TORQUE 0 //want to run at ~800 RPM (to be tuned)
#define START_REGEN_RAMP_RPM 750
#define END_REGEN_RAMP_RPM 1950

#elif defined(TUMANAKO_SHIRE)

#define THROTTLE_POT_MAX 22100 //measured 2371 (P1 180 Ohm)
#define THROTTLE_POT_MIN 3610 //measured 398  (P1 180 Ohm)
//#define THROTTLE_POT_MAX 63000
//#define THROTTLE_POT_MIN 200
#define TORQUE_MAX 25000  //was 19000
#define TORQUE_MIN -250   //was -50
#define ZERO_THROTTLE_TORQUE 1200 //want to run at ~800 RPM (to be tuned)
#define START_REGEN_RAMP_RPM 750
#define END_REGEN_RAMP_RPM 950

#else //i.e. Test bench by default

#define THROTTLE_POT_MAX 63000
#define THROTTLE_POT_MIN 0
#define TORQUE_MAX 1200
#define TORQUE_MIN 0
#define ZERO_THROTTLE_TORQUE 0
#define START_REGEN_RAMP_RPM 750
#define END_REGEN_RAMP_RPM 950

#endif

#ifdef TUMANAKO_WS28
#define DEFAULT_FLUX 14000  //keep stationary flux low (reduce power consumption and cool down!)
    //use flux set 14000 with WS28 at Athol's - It ran real well!
#elif defined(TUMANAKO_WS20)
#define DEFAULT_FLUX 8000  //was 2400, was 9000
#else // Eds Test motor
#define DEFAULT_FLUX 30 //was 12
#endif

//Constants for linear equation to map throttle input to torque request
#define TUMANAKO_A (((float)TORQUE_MAX - TORQUE_MIN) / (THROTTLE_POT_MAX - THROTTLE_POT_MIN))
#define TUMANAKO_B (TORQUE_MIN - TUMANAKO_A * THROTTLE_POT_MIN)

#define FLUX_MIN 2500
#define FLUX_MAX DEFAULT_FLUX
#define FIELD_WEAKENING_RPM_START 3000  
#define FIELD_WEAKENING_RPM_END 6000  //range between START and END must be less than range between FLUX_MAX and FLUX_MIN

//Constants for linear equation to map RPM to flux request (stregthen from low point durin idle)
#define FIELD_STREGTHEN_RPM_START 1000 //Must be greater than END_REGEN_RAMP_RPM 
#define FIELD_STREGTHEN_RPM_END 1500  //Must be less than FIELD_WEAKENING_RPM_END

//Constants for linear equation to map RPM to flux request
#define TUMANAKO_FLUX_A (((float)FLUX_MAX - FLUX_MIN) / (FIELD_STREGTHEN_RPM_END - FIELD_STREGTHEN_RPM_START))
#define TUMANAKO_FLUX_B (FLUX_MIN - TUMANAKO_FLUX_A * FIELD_STREGTHEN_RPM_START)

#define THROTTLE_ERROR_COUNT 6

//------------------------------------------------------------------------------
//Constructor
TumanakoInverter::TumanakoInverter():
    mLoopTime(0),
    mMotorRPM(0),
    mState(RunState_IDLE),
    mOldIGN(true),
    mOldStart(false),
    mFlux(DEFAULT_FLUX),
    mAcceleratorRef(0),
    mPrevAccRef(0),
    mFlashRunLED(false),
    mAcceleratorMIN(10),  //TODO - To be configured via setup software
    mAcceleratorMAX(5000),  //TODO - To be configured via setup software
    mCountMinThrottleError(0),
    mCountMaxThrottleError(0),
    mMotorStallError(0),
    mMotorDirectionError(0){}

//------------------------------------------------------------------------------
Direction_T TumanakoInverter::getDirection(void) {
  if (mSTM32.getFWD() && !mSTM32.getREV()) return(FWD);
  else if (mSTM32.getREV() && !mSTM32.getFWD()) return(REV);
  else if (mSTM32.getNET()) return(NET);
  else {
    //TODO: Flault001 - FWD and REV inputs inconsistant (e.g. both FWD and REV selected)
    //mState=RunState_ERROR;
    mSTM32.setErrorLED(true);
    usartWriteChars("\nWARNING - 'FWD and REV inputs inconsistant'.");
    return(NET);  //Netural
  }
}

//------------------------------------------------------------------------------
//Controls the contactors
PreCharge_T TumanakoInverter::doPrecharge() {
  
  usartWriteChars("\nPrecharge START\n");
  unsigned short previousBusVoltage = 0;
  unsigned short busVoltage;

  //Check contactors are off to start with (if not indicates error elsewhere in system)
  if (!((mSTM32.getK1() == false) && (mSTM32.getK2() == false) && (mSTM32.getK3() == false))) {
    mSTM32.shutdownPower(); //contactors off, timers off, everything off!
    usartWriteChars("\nCONTACTOR_FEEDBACK_ERROR - ', At precharge start, not all contactors are off! (not really a precharge error though)'");
    return PreCharge_CONTACTOR_FEEDBACK_ERROR;
  }
  
  //start precharge
  mSTM32.setK2(false);  //to be sure K2 is off
  mSTM32.setK1(true);
  mSTM32.setK3(true);
  mSTM32.wait(TUMANAKO_PRECHARGE_FEEDBACK_TIME); //allow contactor change to settle

  printFormat("sbsbsbs","\n\rK1=",mSTM32.getK1(),"\n\rK2=",mSTM32.getK2(),"\n\rK3=",mSTM32.getK3(),"\n\r");

  //Test contactor feedback
  if (!((mSTM32.getK1() == true) && (mSTM32.getK2() == false) && (mSTM32.getK3() == true))) {
    mSTM32.shutdownPower(); //contactors off, timers off, everything off!
    usartWriteChars("\nCONTACTOR_FEEDBACK_ERROR - ', Initial contactor setup failed.'");
    return PreCharge_CONTACTOR_FEEDBACK_ERROR;
  }
  
  unsigned long startTime = mSTM32.getMillisecTimer();
  
  //TODO these two while loops could be combined (must retain 1 sec min precharge time though as a design safety feature - ie code design guarantiees 1 sec precharge even if bugs or feedback errors exist)
    
  //wait "at least" 1 sec and check that voltage is actually rising (or target Voltage reached)
  while ((mSTM32.getMillisecTimer() - startTime) < TUMANAKO_MIN_PRECHARGE_TIME) {//MIN_PRECHARGE_TIME
    busVoltage = mSTM32.getRawScaledBusVolt();
    printFormat("sIsI","\r\n PreCharge Phase 1   - BusVolts (V): ",busVoltage,", (digital): ",mSTM32.getRawBusVolt());
    if ((busVoltage > previousBusVoltage) || (busVoltage> TUMANAKO_PRECHARGE_V)) {
      //Voltage is still raising or we have reached target voltage
      previousBusVoltage = busVoltage;
      mSTM32.wait(100);
    } else { //Error: Voltage is not changing and we have not reached target voltage!
      mSTM32.shutdownPower(); //contactors off, timers off, everything off!
      usartWriteChars("\n\n\r  ERROR - 'Voltage is not changing and we have not reached target voltage!'");
      return PreCharge_PHASE1_ERROR;  //Precharge failed
    }
  }

  //Then check we have reached expected voltage (i.e. current has dropped and voltage has stabalised)
  while (mSTM32.getRawScaledBusVolt()< TUMANAKO_PRECHARGE_V) {
    busVoltage = mSTM32.getRawScaledBusVolt();
    printFormat("sIsI","\r\n   PreCharge Phase 2 - BusVolts (V): ",busVoltage,", (digital): ",mSTM32.getRawBusVolt());
    
    if ((mSTM32.getMillisecTimer() - startTime) < TUMANAKO_MAX_PRECHARGE_TIME) {  //MAX_PRECHARGE_TIME
      mSTM32.wait(100); //wait and then try again
    } else { //Error: MAX_PRECHARGE_TIME has elapsed and precharge not finished!
      mSTM32.shutdownPower(); //contactors off, timers off, everything off!
      usartWriteChars("\n\n\r  ERROR - 'MAX_PRECHARGE_TIME has elapsed and precharge not finished!'");
      return PreCharge_PHASE2_ERROR;  //Precharge failed
    }
  };
  
  //precharge finished, full opperating current begins
  mSTM32.setK2(true);   //main circuit on
  mSTM32.setK1(false);  //precharge circuit off
  mSTM32.wait(TUMANAKO_PRECHARGE_FEEDBACK_TIME); //allow contactor change to settle

  printFormat("sbsbsbs","\n\n\rK1=",mSTM32.getK1(),"\n\rK2=",mSTM32.getK2(),"\n\rK3=",mSTM32.getK3(),"\n\r");

  //Test contactor feedback
  //if (!((mSTM32.getK1() == false) && (mSTM32.getK2() == true) && (mSTM32.getK3() == true))) {
  if ( ! mSTM32.getContactorsInRunStateConfiguration() ) {
    mSTM32.shutdownPower(); //contactors off, timers off, everything off!
    usartWriteChars("\nCONTACTOR_FEEDBACK_ERROR - ',Precharge complete, but final contactor change failed.'");
    return PreCharge_CONTACTOR_FEEDBACK_ERROR;
  } 
  
  printFormat("sI","\r\nPrecharge COMPLETE, elapsed time: ",mSTM32.getMillisecTimer() - startTime);

  return(PreCharge_OK); //return precharge OK.
}

//------------------------------------------------------------------------------
//flash the LEDs
void TumanakoInverter::flash() {
  static bool led_on = false;
  static unsigned long last_flash_ms=0;

  //flash every 250ms
  if (mSTM32.getMillisecTimer() > (last_flash_ms+125)) {
    if (led_on) {
      if (mFlashRunLED) mSTM32.setRunLED(true);
      led_on=false;
    } else {
      if (mFlashRunLED) mSTM32.setRunLED(false);
      led_on=true;
    }
    last_flash_ms=mSTM32.getMillisecTimer();
  }
}

//very crude non timer based delay, used during power up (roughly provides 1 ms delay for each multple)
void TumanakoInverter::delay (int multiple) {
  while (multiple !=0) {
    volatile long i=1000;
    while (i!=0) {
      i--;
    }
    multiple--;
  }
}

//------------------------------------------------------------------------------
//SKiiP is reading current A and C (we need A and B), so this method gets us B!
long getPhaseC(long a, long b) {
  long temp;
  long c1 = a;
  long c2 = b;
  temp = -(c1 + c2);

  return temp;
}

//------------------------------------------------------------------------------
short convertToDegrees(short digitalAngle) {
  return (short)(((long)digitalAngle * 180)/32768);  //degrees
}

//------------------------------------------------------------------------------
//Main application loop
void TumanakoInverter::doIt(void) {
  //signed long power = 0;
  unsigned long end_time_ms, start_time_ms;
  //bool oldCrawl = false;
    
#ifndef TUMANAKO_SHIRE
  Direction_T direction = NET;
#endif

#ifdef TUMANAKO_TEST
  unsigned long first_time_ms = 0;
#endif
  
  //clear lights
  mSTM32.setRunLED(false);
  mSTM32.setErrorLED(false);
  
  //crude delay to allow powerstage logic to initalise.
  //{otherwise KiwiAC interupt driven sanity checks kick in and 
  //complain that powerstage is sick)
  delay(1000); //roughly 1 sec delay 
    
  mSTM32.init();
  usartInit();
  mSTM32.resetMillisecTimer();

#ifdef TUMANAKO_TEST
  first_time_ms = mSTM32.getMillisecTimer();
#endif

    
  while (1) { //Main loop
    end_time_ms = mSTM32.getMillisecTimer();
    mLoopTime = end_time_ms-start_time_ms;
    start_time_ms = mSTM32.getMillisecTimer();

#ifdef TUMANAKO_TEST
    if (end_time_ms - first_time_ms > 30000) {
      mSTM32.shutdownPower();
      usartWriteChars("\r\nHALT TEST");
      while (1) {}; //halt
    }
#endif

    flash();  //code to manage flashing LEDS on dash when needed.

    //Read throttle POT position and calc torque ref
    mRawAcceleratorRef = mSTM32.readADC(ACCEL_AI);
    mAcceleratorRef = (short)((TUMANAKO_A * mRawAcceleratorRef) + TUMANAKO_B);

#ifndef TUMANAKO_TEST
    //Check throttle limits (indicates wiring fault)
    if (mRawAcceleratorRef > THROTTLE_POT_MAX) {
      mCountMaxThrottleError++;
      if (mCountMaxThrottleError > THROTTLE_ERROR_COUNT) {
        if (mState != RunState_ERROR) usartWriteChars("\n\n\r  ERROR - 'POT_MAX exceeded! Check Throttle wires.'");
        mState = RunState_ERROR;
        mAcceleratorRef = 0;
      }
    } else mCountMaxThrottleError = 0; //reset error count
#pragma diag_suppress=Pe186 //PCC - Suppress Warning[Pe186]: pointless comparison of unsigned integer with zero
    if (mRawAcceleratorRef < THROTTLE_POT_MIN) {
#pragma diag_default=Pe186 //PCC
      mCountMinThrottleError++;
      if (mCountMinThrottleError > THROTTLE_ERROR_COUNT) {
        if (mState != RunState_ERROR) usartWriteChars("\n\n\r  ERROR - 'POT_MIN exceeded! Check Throttle wires.'");
        mState = RunState_ERROR;
        mAcceleratorRef = 0;
      }
    } else mCountMinThrottleError=0; //reset error count
#endif //NOT TUMANAKO_TEST
    
    //Constant speed and Regenerative braking logic
    mMotorRPM = mSTM32.getRPM(); 
    if ((mAcceleratorRef < 0) && (mMotorRPM < 800)) {
      //Prevent regen if RPM < START_REGEN_RAMP_RPM
      if (mMotorRPM <= START_REGEN_RAMP_RPM)
        mAcceleratorRef = ZERO_THROTTLE_TORQUE;  //could do with a ramp here as well (this is what makes ShireVan motor surge when idleing occasionally)
      //Regen Zero band, 750->770 RPM is zero band for torque
      else if (mMotorRPM <= (START_REGEN_RAMP_RPM+20))
        mAcceleratorRef = 0;       
      //Regen ramp, 770->950 RPM ramps regen from 0 to mAcceleratorRef (which is -ve here)
      else if (mMotorRPM < END_REGEN_RAMP_RPM) 
        mAcceleratorRef = (mMotorRPM - (START_REGEN_RAMP_RPM+20)) * mAcceleratorRef/(END_REGEN_RAMP_RPM - (START_REGEN_RAMP_RPM+20));
    }
    
#ifndef TUMANAKO_SHIRE //(Shire Van is always in FWD, so dont do this)
    //provide dead spot in pot curve
    if ((mAcceleratorRef > -20) && (mAcceleratorRef < 20)) {
      mAcceleratorRef = 0;
    }
#endif
    
    //temp hack for experimental Rotor Time Constant tuning
    //if (mSTM32.getCrawl() == true && oldCrawl == false) {
      //mSTM32.setRotorTimeConstant(mSTM32.getRotorTimeConstant()-1);
    //}

    //Detect crawl mode and reduce torque appropriately (temp disabled)
    if (false) { //mSTM32.getCrawl() == true)
      mAcceleratorRef/=2;
      if ((mMotorRPM > 300)) mAcceleratorRef=0;  //no torque if RPM above limit
    }
    //oldCrawl = mSTM32.getCrawl();

#ifndef TUMANAKO_SHIRE //(Shire Van is always in FWD, so dont do this)
    //TODO add speed limit to this logic
    direction = getDirection();
    if (direction == REV)
      mAcceleratorRef = -mAcceleratorRef;
    else if (direction == NET) {
      mAcceleratorRef = 0;
    }
#endif

    //TODO simple traction control logic


    
    //Field Weakening (Flux control)
    if (mMotorRPM < END_REGEN_RAMP_RPM) {
      mFlux = FLUX_MIN; 
    } else if (mMotorRPM < FIELD_STREGTHEN_RPM_END) {
      //New code to smooth flux bump
      mFlux = (unsigned short)((TUMANAKO_FLUX_A * mMotorRPM) + TUMANAKO_FLUX_B);
      if ((mFlux < FLUX_MIN) || (mFlux > FLUX_MAX)) mFlux = FLUX_MIN + 73;  //This indicates an error in the calc above
    } else if (mMotorRPM > FIELD_WEAKENING_RPM_START) {
      mFlux = FLUX_MAX - TK_ABS(mMotorRPM);
    } else if (mMotorRPM > FIELD_WEAKENING_RPM_END) {
      mFlux = FLUX_MIN;
    } else {
      mFlux = FLUX_MAX;  //default flux
    }
    mSTM32.setFlux(mFlux);
       
    #ifdef TUMANAKO_TEST
    mAcceleratorRef = TUMANAKO_TEST_TORQUE_REF;
    #endif
    
    //Wrong direction detect (current motor control can flip into reverse direction after a stall, hence the need for this code)
    if (((mSTM32.getTorque() >= 0) && (mMotorRPM < 0)) ||
        ((mSTM32.getTorque() < 0) && (mMotorRPM > 0)) ) {
      mMotorDirectionError++;
      if (mMotorDirectionError >= 10) {
        //if (mState != RunState_ERROR) usartWriteChars("\n\n\r  ERROR - 'MOTOR_DIRECTION_REVERSED! Check vehicle.'");
        //mState = RunState_ERROR;
        
        //Zero accelerator and flux to regain sanity at motor control level (TODO understand why it behaves this way)
        mAcceleratorRef = 0;
        mFlux = 0;
      }
    } else {
      if (mMotorDirectionError > 0) {
        mMotorDirectionError--;
      }
      else {
        mFlux = DEFAULT_FLUX;
      }
    }  
    
    mSTM32.setTorque(mAcceleratorRef);
    
    mPrevAccRef = mAcceleratorRef;
    stateMachineDo();   //main vehicle control state machine
    dashboard();   //write dashboard type data to USART1

    if (mState == RunState_RUN) {
      //Only run these tests if motor on!
      
      //power = mSTM32.getTorque() * mSTM32.getRPM();
      if (!mSTM32.busVoltageOK()) { // || (power > TUMANAKO_MAX_REGEN_POWER))
        //Go to error state and Log error to serial
        mState = RunState_ERROR;
        usartWriteChars("\n\n\r  ERROR - 'TUMANAKO_MAX_REGEN_POWER exceeded!'");  //print stuff to RS232
        //break;
      }

#ifndef TUMANAKO_TEST  //can't test this easily on the test rig currently
      //Check contactor feedback
      //if (!((mSTM32.getK1() == false) && (mSTM32.getK2() == true) && (mSTM32.getK3() == true))) {
      if ( ! mSTM32.getContactorsInRunStateConfiguration() ) {
        //Go to error state and Log error to serial
        printFormat("sbsbsbs","\n\rK1=",mSTM32.getK1(),"\n\rK2=",mSTM32.getK2(),"\n\rK3=",mSTM32.getK3(),"\n\r");

//        mState = RunState_ERROR;
        usartWriteChars("\nCONTACTOR_FEEDBACK_ERROR - ',Unexpected Contactor change reported during RunState_RUN.'");
      }
#endif
    }
  }
}


//------------------------------------------------------------------------------
void TumanakoInverter::stateMachineDo(void) {
  //Check we are not exceeding any limits
  int motorParamError = mSTM32.testVariousMotorParam();
  if ((mState != RunState_ERROR) && (motorParamError != 0)) {
    switch (motorParamError) {
    case 1:
      mState = RunState_ERROR;
      usartWriteChars("\n\n\r  ERROR - 'testVariousMotorParam OVERHEAT'");
      break;

    case 2:
      mState = RunState_ERROR;
      usartWriteChars("\n\n\r  ERROR - 'testVariousMotorParam OVERVOLTAGE'");
      break;

    case 3:
      //This case is checked after the contactors are engaged
      //usartWriteChars("ERROR - 'testVariousMotorParam UNDERVOLTAGE'");
      break;

    default:
      mState = RunState_ERROR;
      usartWriteChars("\n\n\r  ERROR - 'testVariousMotorParam UNKNOWN!!?'");
      break;
    }
  }

  switch (mState) {
  case RunState_IDLE:

#ifdef TUMANAKO_TEST
    
#ifdef TUMANAKO_PRECHARGE_TEST
    if (doPrecharge() != PreCharge_OK) {
#else
    if (false) {  //dont test precharge
#endif
      mState = RunState_ERROR;
      usartWriteChars("\n\n\r  ERROR - 'Precharge failed.'");
    } else {
      usartWriteChars("\r\nTEST - IDLE State");
      mSTM32.motorInit(); //Do the init state
      mState=RunState_READY;
      mSTM32.wait(500);
    }
    return;
#else

    //power on, but nothing happening (waiting for ignition to be pushed)
    if (mSTM32.getIGN() == true) {
      //inidicate Contactors being engaged (all dash lights on)
      mSTM32.setRunLED(true);
      mSTM32.setErrorLED(true);

      //do precharge (only do it once though!  State machine needed here)
      if (doPrecharge() != PreCharge_OK) {
        //Precharge failed
        mState = RunState_ERROR;
        usartWriteChars("\n\n\r  ERROR - 'Precharge failed.'");
      } else {
        //Precharge Successful, continue with motor startup
        mSTM32.motorInit(); //Do the init state

        //inidicate Contactors engaged (just Run LED flashing)
        mFlashRunLED = true ;
        mSTM32.setErrorLED(false);

        mState=RunState_READY;
        mOldStart = false;  //prepare debounce logic (need to encapsulate this)
      }
    }
    break;
#endif


  case RunState_READY:
#ifdef TUMANAKO_TEST
    usartWriteChars("\r\nTEST - READY State");
    mSTM32.motorStart();
    mState=RunState_RUN;
    mSTM32.wait(500);
    return;
#else

    //Precharge begins (and contactors engaged! Undervoltage check confirms this)

    //If IGN off, return to IDLE
    if ((mSTM32.getIGN() == false) && (mOldIGN == false)) {
      mSTM32.shutdownPower();
      mState=RunState_IDLE;
      mFlashRunLED = false;
      mSTM32.setRunLED(false);
    }
    mOldIGN = mSTM32.getIGN();

    if ((mSTM32.getStart()==true) && (mOldStart == true)) {
      //Check for start fault conditions

#ifndef TUMANAKO_SHIRE  //Shire Van is always in FWD
      if (!mSTM32.getNET()) {
        mState = RunState_ERROR;
        usartWriteChars("\n\n\r  ERROR - 'Must be in netural to start'");
        break;
      }
#endif
      
      if (motorParamError == 3) { //UNDERVOLTAGE - TODO enum!!!
        mState = RunState_ERROR;
        usartWriteChars("\n\n\r  ERROR - 'testVariousMotorParam UNDERVOLTAGE'");
        break;
      }

      if (mAcceleratorRef > ZERO_THROTTLE_TORQUE) { //throttle must not be engaged when start button pushed
        mState=RunState_ERROR;
        usartWriteChars("\n\n\r  ERROR - 'throttle must be zero when start button pushed'.");
        break;
      }

      //All OK, so start motor
      mSTM32.motorStart();
      mState=RunState_RUN;

      //Indicate Motor ON! (flashing off, solid green on).
      mFlashRunLED = false ;
      mSTM32.setRunLED(true);
      mOldIGN = true;  //prepare debounce logic (need to encapsulate this)
    }

    mOldStart = mSTM32.getStart();
    break;
#endif


  case RunState_RUN:
#ifdef TUMANAKO_TEST
    return;
#else
    mSTM32.motorTestForSpeedError();

    //If IGN off, stop the motor
    if ((mSTM32.getIGN() == false) && (mOldIGN == false)) {
      mSTM32.shutdownPower();
      mState=RunState_IDLE;
      mSTM32.setRunLED(false);
    }
    mOldIGN = mSTM32.getIGN();
    
    //Stall detect logic
    if ((mSTM32.getTorque() > 400) && (mMotorRPM == 0)) {
      mMotorStallError++;
      if (mMotorStallError >= 6000) {
        if (mState != RunState_ERROR) usartWriteChars("\n\n\r  ERROR - 'MOTOR_STALL! Check vehicle.'");
        mState = RunState_ERROR;
        mAcceleratorRef = 0;
      }
    } else {
      mMotorStallError=0;
    }   
    
    break;
#endif

  case RunState_ERROR:
    mSTM32.shutdownPower();  //TODO - need to explicitly disengaged contactors
    mSTM32.setErrorLED(true);
    //TODO move all usartWriteChars calls to here to reduce time to shutdown.
    //Use variable to pass appropriate Error message.
    break;

  default:
    //Error - should not get here!
    usartWriteChars("\n\n\r  ERROR - 'Unknown State!!! Going to Error State now.'");
    mState=RunState_ERROR;
    break;
  }
}

//------------------------------------------------------------------------------
// Write dashboard type data to RS232 port
void TumanakoInverter::dashboard()
{
  static unsigned long count=0;

  //Only output serial port data once every 5000 times
  if (count == 5000) {
    //signed short phaseC = getPhaseC(mSTM32.getPhase1(), mSTM32.getPhase2());
    signed short phaseC = mSTM32.getPhase3();
    float current = TK_ABS(mSTM32.getPhase1()) + TK_ABS(mSTM32.getPhase2()) + TK_ABS(phaseC);
    if (mState != RunState_ERROR) {
      usartWriteDisclaimer();
      printFormat("sIs","       Bus (V): ",mSTM32.busVoltage(), " ");
      printFormat("sIs","pwrStgTemp (C): ",mSTM32.powerStageTemperature(), "\r\n");
      printFormat("sIs","           RPM: ",mMotorRPM, " ");
      //printFormat("sIs","           IGN: ",(int)mSTM32.getIGN(), "\r\n");
      printFormat("sIs","          Flux: ",mSTM32.getFlux(), "\r\n");
      printFormat("sIs","     Torque In: ",mAcceleratorRef, " ");
      printFormat("sIs","   RawAccelPOT: ",mRawAcceleratorRef, "\r\n");
      printFormat("sIs","    Torque Out: ",mSTM32.getTorque(), " ");
      printFormat("sIs","Loop time (ms): ",mLoopTime, "\r\n");
      printFormat("sIs","   Phase 1 (A): ",mSTM32.getPhase1(), " ");
      printFormat("sIs","   Phase 2 (A): ",mSTM32.getPhase2(), "\r\n");
      printFormat("sIs"," PhAOffset (A): ",mSTM32.getPhaseAOffset(), " ");
      printFormat("sIs"," PhBOffset (A): ",mSTM32.getPhaseBOffset(), "\r\n");
      printFormat("sIs","    phase3 (A): ",phaseC, " ");
      printFormat("sfs","     Total (A): ",current, "\r\n");
      printFormat("sIs","RotorTimeConst: ",mSTM32.getRotorTimeConstant(), " ");
      printFormat("sIs","      SlipFreq: ",mSTM32.getSlipFreq(), "\r\n");
      printFormat("sIs","     FluxAngle: ",convertToDegrees(mSTM32.getFluxAngle()), " ");
      printFormat("sI" ," ElectricAngle: ",convertToDegrees(mSTM32.getElectricalAngle()));
  
      count=0;
    }
  }
  count++;
}