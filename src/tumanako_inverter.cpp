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
//   Philip Court 15/Mar/2011 - Updated during implementation of STM32_sine_interface.cpp
//   Philip Court 5/Feb/2012 - Added support for 2 contactor 1 IGBT precharge circuit and added init method to tidy up
//   Philip Court 29/Jan/2012 - Added watchdog timer and 3 sec start button check/delay
//   Philip Court 28/Aug/2012 - Added torque limits based on speed and DC Bus amps, plus watchdog refinements
//------------------------------------------------------------------------------


//--- Includes -----------------------------------------------------------------
#include <cmath>
#include <time.h>

#define STM32F1  //applicable to the STM32F1 series of devices
#include <libopencm3/cm3/common.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/stm32/f1/memorymap.h>
#include <libopencm3/stm32/wwdg.h>

#include "tumanako_inverter.hpp"
#include "tumanako_serial.hpp"
#include "digital/digital.hpp" //used to scale digital data
#include "tumanako_type.h"
#include "tumanako_vehicle_configuration.h"

/*----------------------------- Private typedef ------------------------------*/
/*----------------------------- Private define -------------------------------*/
#define TK_ABS(x) ( (x) < 0 ? -(x) : (x))
#define TK_MAX(a,b) ( (a) > (b) ) ? (a) : (b)
#define RESET_WATCHDOG WWDG_CR = 0x7f

#define TUMANAKO_MAX_REGEN_POWER 40000000

#define EVAL_BOARD_POT 14 //PC4 (see page 30 UM0426)
#define EVAL_BOARD_MIC 15 //PC5

#ifdef TUMANAKO_KIWIAC
#define ACCEL_AI EVAL_BOARD_POT
#else  //i.e. STM Eval kit and vero board
#define ACCEL_AI EVAL_BOARD_MIC
#endif

//Common to all vehicles
#define TUMANAKO_BUS_CURRENT_LIMIT_RANGE (TUMANAKO_BUS_CURRENT_LIMIT_END - TUMANAKO_BUS_CURRENT_LIMIT_START)
#define RETARD_COUNT_LIMIT (((long)TUMANAKO_BUS_CURRENT_LIMIT_EFFECT_MAX * TORQUE_MAX) / 100)  //MAX torque reduction when over temperature or similar event occurs (e.g. 30% of TORQUE_MAX)


#ifdef TUMANAKO_WS28
#define FLUX_MIN 8000  //New feature since last Saker run!
#define DEFAULT_FLUX 14000  //keep stationary flux low (reduce power consumption and cool down!)
    //use flux set 14000 with WS28 at Athol's - It ran real well!

#elif defined(TUMANAKO_WS20)
//WARNING: range between FIELD_WEAKENING_RPM_START and END must be less than range between FLUX_MAX and FLUX_MIN
#define FLUX_MIN 1000 //4900  //TODO: Increase this (was 2500 pre 8 Mar 2012) was 4900 at 20120515
#define DEFAULT_FLUX 9000  //was 2400, was 9000, was 8000 (pre 8 Mar 2012)

#elif defined(TUMANAKO_SWITCH_EV)
//WARNING: range between FIELD_WEAKENING_RPM_START and END must be less than range between FLUX_MAX and FLUX_MIN
#define FLUX_MIN 300  //Keep low to minimise current and losses at idle
#define DEFAULT_FLUX 24000  //Got to 22000 before dyno broke (estimate this will give > 600 Nm)

#else // Eds Test motor
#define FLUX_MIN 120 //was 500!!?
#define DEFAULT_FLUX 120 //was 12
#endif

//Constants for linear equation to map throttle input to torque request (y = Ax + B)
#define TUMANAKO_A (((float)TORQUE_MAX - TORQUE_MIN) / (THROTTLE_POT_MAX - THROTTLE_POT_MIN))
#define TUMANAKO_B (TORQUE_MIN - TUMANAKO_A * THROTTLE_POT_MIN)

#define FLUX_MAX DEFAULT_FLUX

//TODO these need to be made specific for each system (current target is Shire Van)
#define FIELD_WEAKENING_RPM_START 3000  
#define FIELD_WEAKENING_RPM_END 6000  //range between START and END must be less than range between FLUX_MAX and FLUX_MIN
//Constants for linear equation to map RPM to flux request (stregthen from low point during idle)
#define FIELD_STREGTHEN_RPM_START 1000 //Must be greater than END_REGEN_RAMP_RPM 
#define FIELD_STREGTHEN_RPM_END 1500  //Must be less than FIELD_WEAKENING_RPM_END

//Constants for linear equation to map RPM to flux request (y = Ax + B)
#define TUMANAKO_FLUX_A (((float)FLUX_MAX - FLUX_MIN) / (FIELD_STREGTHEN_RPM_END - FIELD_STREGTHEN_RPM_START))
#define TUMANAKO_FLUX_B (FLUX_MIN - TUMANAKO_FLUX_A * FIELD_STREGTHEN_RPM_START)

#define THROTTLE_ERROR_COUNT 6

//------------------------------------------------------------------------------
//Constructor
TumanakoInverter::TumanakoInverter():
    mLoopTime(0),
    mMotorRPM(0),
    mAbsoluteMotorRPM(0),
    mFlux(DEFAULT_FLUX),
    mAcceleratorRef(0),
    mAcceleratorRefSmooth(0),
    mAccelRetardCount(0),
    mSpeedBasedTorqueLimit(0),
    mBusCurrent(0),
    mBusCurrentAvg(0),
    mState(RunState_IDLE),
    mOldIGN(true),
    mOldStart(false),
    mPrevAccRef(0),
    mFlashRunLED(false),
    mAcceleratorMIN(10),  //TODO - To be configured via setup software (not used currently)
    mAcceleratorMAX(5000),  //TODO - To be configured via setup software (not used currently)
    mCountMinThrottleError(0),
    mCountMaxThrottleError(0),
    mMotorStallError(0),
    mDirectionError(0) //digital Input logic
    {}

//------------------------------------------------------------------------------
Direction_T TumanakoInverter::getDirection(void) {
  if (mSTM32.getFWD() && !mSTM32.getREV()) {mDirectionError = 0; return(FWD);}
  else if (mSTM32.getREV() && !mSTM32.getFWD()) {mDirectionError=0; return(REV);}
  else if (mSTM32.getNET()) {mDirectionError=0; return(NET);}
  else {
    //Flault001 - FWD and REV inputs inconsistant (e.g. both FWD and REV selected)
    mDirectionError++;
    if (mDirectionError > 3)  //allow some digital noise during switch over (digital filter should handle the rest) 
    {
      mSTM32.setErrorLED(true);  //activate warning light
      mState=RunState_ERROR;  //shut the system down
    }
    usartWriteChars("\nWARNING - 'FWD and REV inputs inconsistant'.\r\n");
    return(NET);  //Netural
  }
}

//------------------------------------------------------------------------------
//Controls the contactors
PreCharge_T TumanakoInverter::doPrecharge() {

  mSTM32.setKiwiACRedLED(true);  //turn KiwiAC red light on
  usartWriteChars("\nPrecharge START\n");
  unsigned short previousBusVoltage = 0;
  unsigned short busVoltage;

  //Check contactors are off to start with (if not indicates error elsewhere in system)
#ifdef TUMANAKO_PETER_UTE  //K3 controlled externally and will be on before precharge starts (within 5 sec)
  MyTimer k3Timer;
  while ((mSTM32.getK3() == false)&&(k3Timer.getElapsed() < 5000)) {mSTM32.wait(200);}  //wait 5 sec for K3 to come on (externally controlled)
  if (!((mSTM32.getK1() == false) && (mSTM32.getK2() == false) && (mSTM32.getK3() == true))) {
#else
  if (!((mSTM32.getK1() == false) && (mSTM32.getK2() == false) && (mSTM32.getK3() == false))) {
#endif
    
    mSTM32.shutdownPower(); //contactors off, timers off, everything off!
    usartWriteChars("\nCONTACTOR_FEEDBACK_ERROR - ', At precharge start, not all contactors are off! (not really a precharge error though)'");
    return PreCharge_CONTACTOR_FEEDBACK_ERROR;
  }
#ifndef TUMANAKO_THREE_RELAY  //i.e. GSCB
  //TODO turn K3 on first and check we don't see voltage increase (i.e. precharge not on)
  //Also need to cater for case where caps are still fully charged!
#endif
  //start precharge
  mSTM32.setK2(false);  //to be sure K2 is off
  mSTM32.setK1(true);
  mSTM32.setK3(true);
  mSTM32.wait(TUMANAKO_PRECHARGE_FEEDBACK_TIME); //allow contactor change to settle

  printFormat("sbsbsbs","\n\rK1=",mSTM32.getK1(),"\n\rK2=",mSTM32.getK2(),"\n\rK3=",mSTM32.getK3(),"\n\r");

  //Test contactor feedback (K1 is precharge contactor)
#ifdef TUMANAKO_THREE_RELAY //old style contactor box with 3 gx12 relays
  if (!((mSTM32.getK1() == true) && (mSTM32.getK2() == false) && (mSTM32.getK3() == true))) {
#else    // GSCB (greenstage contactor box) - ignore K1 because K1 only shows on if drawing current
  if (!((mSTM32.getK2() == false) && (mSTM32.getK3() == true))) { //temp for low current test
#endif
    mSTM32.shutdownPower(); //contactors off, timers off, everything off!
    usartWriteChars("\nCONTACTOR_FEEDBACK_ERROR - ', Initial contactor setup failed.'");
    return PreCharge_CONTACTOR_FEEDBACK_ERROR;
  }

  MyTimer prechargeTimer;

  //TODO these two while loops could be combined (must retain 1 sec min precharge time though as a design safety feature - ie code design guarantiees 1 sec precharge even if bugs or feedback errors exist)
    
  //wait "at least" 1 sec and check that voltage is actually rising (or target Voltage reached)
  while (prechargeTimer.getElapsed() < TUMANAKO_MIN_PRECHARGE_TIME) {//MIN_PRECHARGE_TIME
    busVoltage = mSTM32.getRawScaledBusVolt();
    printFormat("sIsI","\r\n PreCharge Phase 1   - BusVolts (V): ",busVoltage,", (digital): ",mSTM32.getRawBusVolt());
    if ((busVoltage > previousBusVoltage) || (busVoltage> TUMANAKO_PRECHARGE_V)) {
      //Voltage is still raising or we have reached target voltage
      previousBusVoltage = busVoltage;
      mSTM32.wait(TUMANAKO_PRECHARGE_ITERATION_WAIT);
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
    
    if (prechargeTimer.getElapsed() < TUMANAKO_MAX_PRECHARGE_TIME) {  //MAX_PRECHARGE_TIME
      mSTM32.wait(TUMANAKO_PRECHARGE_ITERATION_WAIT); //wait and then try again
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
  if ( ! mSTM32.getContactorsInRunStateConfiguration() ) {
    mSTM32.shutdownPower(); //contactors off, timers off, everything off!
    usartWriteChars("\nCONTACTOR_FEEDBACK_ERROR - ',Precharge complete, but final contactor change failed.'");
    return PreCharge_CONTACTOR_FEEDBACK_ERROR;
  } 
  
  printFormat("sI","\r\nPrecharge COMPLETE, elapsed time: ",prechargeTimer.getElapsed());

  return(PreCharge_OK); //return precharge OK.
}

//------------------------------------------------------------------------------
//flash the green RUN LED on the dash
void TumanakoInverter::flash() {
  static bool led_on = false;

  //flash every 250ms
  if (mLastFlashTimer.getElapsed() > 125) {
    if (led_on) {
      if (mFlashRunLED) mSTM32.setRunLED(true);
      led_on=false;
    } else {
      if (mFlashRunLED) mSTM32.setRunLED(false);
      led_on=true;
    }
    mLastFlashTimer.reset();
  }
}

//------------------------------------------------------------------------------
//very crude non timer based delay, used during power up (roughly provides 1 ms delay for each multiple)
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
void enableWatchdog () {
  static bool enabled = false;

  //ONLY ENABLE ONCE
  if (!enabled) {
    //See also stm32.init (contains NVIC setup for WWDG IRQ)
 
    //Setup *watchdog* to ensure we shutdown if main loop gets stuck  
    RCC_APB1ENR |= RCC_APB1ENR_WWDGEN;  //enable watchdog clock
    u32 temp = WWDG_CFR & 0xFFFFFE7F;
    WWDG_CFR = temp | 0x00000180 | 65 ; //watchdog prescaller = div 8 with window = 65
    
    WWDG_SR = 0; //clear early wakeup interupt flag (EWI)
  
    //Use Bit banding to enable EWI (see Cortex M3 programing manual, Doc ID 15491, Section 2.2.5)
    //TODO - move these #defines to libopencm3
    #define WWDG_CFR_EWI_BIT 9
    #define PERIPH_BITBAND_BASE 0x42000000

    u32 bit_word_offset = ((WWDG_CFR - PERIPH_BASE) * 32) + (WWDG_CFR_EWI_BIT * 4);
    (*(volatile u32 *) (PERIPH_BITBAND_BASE + bit_word_offset)) = (u32)1;  //enable EWI

    WWDG_CR = 0x80 | 0x7f;  //Enablei WWDG! 0x80=Watchdog activation bit.  0x40 = min delay (113 us), use 0x7F for max delay (7.28 ms)

    enabled = true;
    usartWriteChars("\n\r  INFO - 'Watchdog Enabled.'");
  }      
}

//------------------------------------------------------------------------------
//Manage IGN off logic (system being turned off via IGN key)
void TumanakoInverter::turnOff() {
      mSTM32.shutdownPower();  //Turn off powerstage
      mState=RunState_IDLE;
      mFlashRunLED = false;
      mSTM32.setRunLED(false);
      mSTM32.systemReset();  //resets STM32 on KiwiAC board  (hence restarts watchdog logic)
}

//Is the bus voltage OK?
bool TumanakoInverter::busVoltageOK(void) {
  unsigned short busVolt = mSTM32.getRawScaledBusVolt();

  //  check bus over voltage
  if ((busVolt >= TUMANAKO_PRECHARGE_V) && (busVolt <= TUMANAKO_MAX_BUS_V ))
    return true;
  else
    return true;
}


//------------------------------------------------------------------------------
void TumanakoInverter::init(void) {
  
  //clear lights
  mSTM32.setRunLED(false);
  mSTM32.setErrorLED(false);
  
  //crude delay to allow powerstage logic to initalise.
  //{otherwise KiwiAC interupt driven sanity checks kick in and 
  //complain that powerstage is sick)
  delay(1000); //roughly 1 sec delay 
    
  mSTM32.init();
  usartInit();
}

//------------------------------------------------------------------------------
//Main application loop
void TumanakoInverter::doIt(void) {
  
  init(); //initiallise all systems  
mSTM32.setRunLED(true);
  
#ifndef TUMANAKO_SHIRE
  Direction_T direction = NET;
#endif
  
  //Timers must be created (or reset) after STM32 init
  MyTimer loopTimer;
  mLastFlashTimer.reset(); //to cater for the fact this was created before STM32 init
  mRunTimer.reset(); //to cater for the fact this was created before STM32 init

//Digital class gives same functionality as these old MACROs (delete the macros!)
//#define TUMANAKO_A (((float)TORQUE_MAX - TORQUE_MIN) / (THROTTLE_POT_MAX - THROTTLE_POT_MIN))
//#define TUMANAKO_B (TORQUE_MIN - TUMANAKO_A * THROTTLE_POT_MIN)
  
  //Scale digital analogue POT input to torque request
  Digital torque(THROTTLE_POT_MIN, THROTTLE_POT_MAX,
                 TORQUE_MIN, TORQUE_MAX);

  //Scale RPM to torque reduction
  Digital speedBasedTorqueLimit(TUMANAKO_TORQUE_LIMIT_RPM_START, TUMANAKO_TORQUE_LIMIT_RPM_END,
                                TUMANAKO_SPEED_BASED_TORQUE_LIMIT_MIN, TUMANAKO_SPEED_BASED_TORQUE_LIMIT_MAX);

  while (1) { //Main loop
    mLoopTime = loopTimer.getElapsed();
    loopTimer.reset();
    mTorqueLimitOccured = false;

    //reset the watchdog timer (do this each time through the loop (and at various other places!)      
    RESET_WATCHDOG;
    
#ifdef TUMANAKO_RUN_TIMER
    if (mRunTimer.getElapsed() > TUMANAKO_MAX_RUNTIME) { //if runtime > 30 sec
      mSTM32.shutdownPower();
      usartWriteChars("\r\nHALT - TUMANAKO_MAX_RUNTIME exceeded.\r\n");
      while (1) {}; //halt
    }
#endif

    flash();  //flashing RUN LED on dash     
    mMotorRPM = mSTM32.getRPM();
    mAbsoluteMotorRPM = TK_ABS(mMotorRPM);
    
#ifdef SPEED_BASED_TORQUE_LIMIT
    if ((mAbsoluteMotorRPM > TUMANAKO_TORQUE_LIMIT_RPM_START) && (mAbsoluteMotorRPM < TUMANAKO_TORQUE_LIMIT_RPM_END)) {
      mSpeedBasedTorqueLimit = speedBasedTorqueLimit.eu(mAbsoluteMotorRPM);  //reduce max torque at upper RPMs
    } else if (mAbsoluteMotorRPM >= TUMANAKO_TORQUE_LIMIT_RPM_END){
      mSpeedBasedTorqueLimit = TUMANAKO_SPEED_BASED_TORQUE_LIMIT_MAX;
    } else { //mAbsoluteMotorRPM < TUMANAKO_TORQUE_LIMIT_RPM_START
      mSpeedBasedTorqueLimit = TUMANAKO_SPEED_BASED_TORQUE_LIMIT_MIN;
    }      
#endif
    
    //Read throttle POT position and calc torque ref
    mRawAcceleratorRef = mSTM32.readADC(ACCEL_AI);
    //mAcceleratorRef = (short)((TUMANAKO_A * mRawAcceleratorRef) + TUMANAKO_B);
      
    //EU.max is dynamically constrained based on DC Bus volt and rpm speed limits
    mAcceleratorRef = torque.eu(mRawAcceleratorRef, (mAccelRetardCount + mSpeedBasedTorqueLimit));  //mAccelRetardCount controlled by DC Bus current
    
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
    if (mRawAcceleratorRef < THROTTLE_POT_MIN) {
      mCountMinThrottleError++;
      if (mCountMinThrottleError > THROTTLE_ERROR_COUNT) {
        if (mState != RunState_ERROR) usartWriteChars("\n\n\r  ERROR - 'POT_MIN exceeded! Check Throttle wires.'");
        mState = RunState_ERROR;
        mAcceleratorRef = 0;
      }
    } else mCountMinThrottleError=0; //reset error count
#endif //NOT TUMANAKO_TEST

    direction = getDirection();
    mDirection = direction; //copy for the logging - PCC
    
#ifndef TUMANAKO_SHIRE //(Shire Van is always in FWD, so dont do this)
    //TODO add speed limit to this logic
    if (direction == REV)
      mAcceleratorRef = -mAcceleratorRef;
    else if (direction == NET) {
      mAcceleratorRef = 0;
    }
#endif
    
    //Constant speed and Regenerative braking logic#1    
    //Disables regen at low RPM (and for ShireVan case, provides crude idle control)
    if (((direction == FWD) && (mAcceleratorRef < 0) && (mMotorRPM < GLIDE_RPM)) || //this includes -ve RPM on purpose
        ((direction == REV) && (mAcceleratorRef > 0) && (mMotorRPM > -GLIDE_RPM))) {  //prevent regen driving vehicle in wrong direction
      //Prevent regen if RPM < START_REGEN_RAMP_RPM (750)
      // - gap between START_REGEN_RAMP_RPM and GLIDE_RPM is special case for shire van (for shire van GLIDE_RPM > START_REGEN_RAMP_RPM)
      // - in other cases, START_REGEN_RAMP_RPM is actually the GLIDE_RPM (assuming it is less than GLIDE_RPM)
      if ( ((direction == FWD) && (mMotorRPM <= START_REGEN_RAMP_RPM)) || 
           ((direction == REV) && (mMotorRPM >= -START_REGEN_RAMP_RPM)) ) {
        mAcceleratorRef = ZERO_THROTTLE_TORQUE;  //could do with a ramp here as well (lack of a ramp is what makes ShireVan motor surge when idleing occasionally)      
      }
#ifdef TUMANAKO_SHIRE //TODO control idle speed with PID
      else if (mMotorRPM <= (START_REGEN_RAMP_RPM+20)) { //Regen Zero band, 750->770 RPM is zero band for torque
        mAcceleratorRef = 0;       
      } 
      //Regen ramp, 770->950 RPM (770->800 RPM actually!?) ramps regen from 0 to mAcceleratorRef (which is -ve here)
      else if (mMotorRPM < END_REGEN_RAMP_RPM) { //FIX - THIS will never execute for RPM >=800. Plus this section looks a bit dodgy - PCC 29/Apr/2012
        mAcceleratorRef = (mMotorRPM - (START_REGEN_RAMP_RPM+20)) * mAcceleratorRef/(END_REGEN_RAMP_RPM - (START_REGEN_RAMP_RPM+20));
      }
#endif
    }
    mBusCurrent = mSTM32.getInstantaniousCurrent();  //required for current limitation  
    mBusCurrentAvg = (99*mBusCurrentAvg + mBusCurrent) / 100;   //smooth version of BusCurrent
    
    //Constant speed and Regenerative braking logic#2 (See also "Regenerative braking logic#1" above)
    //Provide very slight, potentially "additional" regen braking (triggered by brake pedal touch) - Additional due to TORQUE_MIN setting being -ve in some cases
    if ((mSTM32.getRawScaledBusVolt() <= TUMANAKO_MAX_BUS_V) && (mBusCurrentAvg <= TUMANAKO_MAX_REGEN_CURRENT_A)) {  //but only if overvoltage has not occurred (new as off 20120531)
      if (mSTM32.getBrakeOn() && mSTM32.getEnableRegen()) //if brake is on and regen is enabled
      {
        mAcceleratorRef = mAcceleratorRef - BRAKE_TORQUE_REDUCTION;  //Reduce torque request by some amount (relative regen)
        if (mAcceleratorRef < (TORQUE_MIN - BRAKE_TORQUE_REDUCTION)) 
          mAcceleratorRef = TORQUE_MIN - BRAKE_TORQUE_REDUCTION;  //limit absolute regen
        if (mMotorRPM < GLIDE_RPM) mAcceleratorRef = 0;  //don't want regen driving us backwards! and no regen logic in reverse 
      }
    } else {
      //usartWriteChars("\n\n\r  WARNING - 'TUMANAKO_MAX_BUS_V exceeded, regen disabled.'");  //print stuff to RS232
    }
    
#ifndef TUMANAKO_SHIRE //(Shire Van is always in FWD, so dont do this)
    //provide dead spot in pot curve (+/-20 in most cases)
    if ((mAcceleratorRef > -ACCEL_DEAD_SPOT) && (mAcceleratorRef < ACCEL_DEAD_SPOT)) {
      mAcceleratorRef = 0;
    }
#endif

    //Detect crawl mode and reduce torque appropriately (temp disabled)
    //TODO put noise filter on getCrawl impl
    if (mSTM32.getCrawl() == true) {
      mAcceleratorRef/=2; //halve torque available if crawl switch is on (TODO: should also do this if in REV)
      if ((mAbsoluteMotorRPM > TUMANAKO_CRAWL_RPM_LIMIT)) mAcceleratorRef=0;  //and no torque if RPM above limit
    } 

    // === TEMPERATURE AND CURRENT LIMIT LOGIC START ===

    mMotorTemp = mSTM32.motorTemperature();
    mPowerStageTemp = mSTM32.powerStageTemperature();
    short maxMeasuredTemp = TK_MAX(mMotorTemp, mPowerStageTemp);  //take highest of motor and inverter temperatures     
    
    //Manage temperature limits - Duirectly reduce motor power if temperature is too high
    if (maxMeasuredTemp > TUMANAKO_MOTOR_TEMP_LIMIT_START) {
      mTorqueLimitOccured = true;
      if (maxMeasuredTemp >= TUMANAKO_MOTOR_TEMP_LIMIT_END) {
        mAcceleratorRef = 0;  //TODO - we probably want to provide a crawl mode here!
      } else {
        //ensure AcceleratorRef ranges from 100% to 0% between LIMIT_START and LIMIT_END
        mAcceleratorRef = (short)((mAcceleratorRef * 
          ((float)TUMANAKO_MOTOR_TEMP_LIMIT_END - maxMeasuredTemp)) /
          TUMANAKO_MOTOR_TEMP_LIMIT_RANGE);
      }
    }
    
#ifdef DC_BUS_CURRENT_BASED_TORQUE_LIMIT         
    //Manage current limits (PCC new and tested as of 20120523)
    if (mBusCurrentAvg > TUMANAKO_BUS_CURRENT_LIMIT_START) {
      mTorqueLimitOccured = true;
      if (mBusCurrentAvg >= TUMANAKO_BUS_CURRENT_LIMIT_END) {
        mAccelRetardCount = RETARD_COUNT_LIMIT; //e.g. Max Retard Count increment
      } else {
        //AcceleratorRef limited from 100% to (100-TUMANAKO_BUS_CURRENT_LIMIT_EFFECT_MAX)%
        //between LIMIT_START and LIMIT_END (e.g. limit from 100% -> 70%)
        mAccelRetardCount = (short)((RETARD_COUNT_LIMIT * 
          ((float)mBusCurrentAvg - TUMANAKO_BUS_CURRENT_LIMIT_START)) /
          TUMANAKO_BUS_CURRENT_LIMIT_RANGE);
      }
      
      if (mAccelRetardCount > RETARD_COUNT_LIMIT)  //eg LIMIT = 30% of throttle range?
        mAccelRetardCount = RETARD_COUNT_LIMIT;     
    } else {
      if (mAccelRetardCount > 0)
        mAccelRetardCount--; //decrement Retard Count (since we are not exceeding the bus current limit)
    }         
#endif
    
    #ifdef TUMANAKO_TEST
    mAcceleratorRef = TUMANAKO_TEST_TORQUE_REF;  //Allows test bench to run without throttle pot connected
    #endif

    //Smooth accelerator input
    //mAcceleratorRefSmooth = mAcceleratorRef;   //no smoothing
    mAcceleratorRefSmooth = (short)(( ((long)99*mAcceleratorRefSmooth) + mAcceleratorRef )/100);  //smoothing
    
    // === TEMPERATURE AND CURRENT LIMIT LOGIC END ===
    
    
    mFlux = FLUX_MIN;
       
#ifdef TUMANAKO_RPM_BASED_FLUX  //old approach
    if (mMotorRPM < END_REGEN_RAMP_RPM) {
      mFlux = FLUX_MIN;
    } else if (mMotorRPM < FIELD_STREGTHEN_RPM_END) {
      //New code to smooth flux bump
      mFlux = (unsigned short)((TUMANAKO_FLUX_A * mMotorRPM) + TUMANAKO_FLUX_B);
      if ((mFlux < FLUX_MIN) || (mFlux > FLUX_MAX)) mFlux = FLUX_MIN + 73;  //This (FLUX_MIN+73) indicates an error in the calc above
    } else if ((mMotorRPM > FIELD_WEAKENING_RPM_START)&&(mMotorRPM <= FIELD_WEAKENING_RPM_END)){
      mFlux = FLUX_MAX - (mAbsoluteMotorRPM - FIELD_WEAKENING_RPM_START);  //previous (pre 8 Mar 2012) was FLUX_MAX - mAbsoluteMotorRPM
    } else if (mMotorRPM > FIELD_WEAKENING_RPM_END) {
      mFlux = FLUX_MIN;
    } else {
      mFlux = FLUX_MAX;  //default flux 
    }
#else  //New approach (aim is to match torqe to flux for max efficiency)
    
    if (TK_ABS(mAcceleratorRefSmooth) > FLUX_MIN) {
      mFlux = TK_ABS(mAcceleratorRefSmooth);
    }
  
    //TODO - field weakening if getFlux() ~!= mFlux (or if get Torque ~!= mAcceleratorRef
    
#endif
  
    if (mFlux > FLUX_MAX) mFlux = FLUX_MAX; //sanity limit
  
    //Finally, set actual Flux and Torque values for motor control to use
    mSTM32.setFlux(mFlux);     
    mSTM32.setTorque(mAcceleratorRefSmooth);
    
    mPrevAccRef = mAcceleratorRefSmooth;  //TODO, capture and display max change in AcceleratorRef
    stateMachineDo();   //main vehicle control state machine
    dashboard();   //write dashboard type data to 2nd serial port (labled as USART1 on KiwiAC PCB.  NOTE: this is actually USART2 at software level)

    if (mState == RunState_RUN) {
      //Only run these tests if motor on!
      
      //power = mSTM32.getTorque() * mSTM32.getRPM();
      if (!busVoltageOK()) { // || (power > TUMANAKO_MAX_REGEN_POWER))
        //Go to error state and Log error to serial
        mState = RunState_ERROR;
        usartWriteChars("\n\n\r  ERROR - 'TUMANAKO_MAX_REGEN_POWER exceeded!'");  //print stuff to RS232
      }

#ifndef TUMANAKO_TEST  //can't test this easily on the test rig currently
#ifdef TUMANAKO_PRECHARGE_TEST  //do precharge test output

      //Check contactor feedback
      if ( ! mSTM32.getContactorsInRunStateConfiguration() ) {
        //Go to error state and Log error to serial
        printFormat("sbsbsbs","\n\rK1=",mSTM32.getK1(),"\n\rK2=",mSTM32.getK2(),"\n\rK3=",mSTM32.getK3(),"\n\r");

        mState = RunState_ERROR;
        usartWriteChars("\nCONTACTOR_FEEDBACK_ERROR - ',Unexpected Contactor change reported during RunState_RUN.'");
      }
      
#endif //not TUMANAKO_TEST
#endif //TUMANAKO_PRECHARGE_TEST
    }
  }
}


//------------------------------------------------------------------------------
void TumanakoInverter::stateMachineDo(void) {
  //First, check we are not exceeding any limits
  int motorParamError = mSTM32.testVariousMotorParam();
  if ((mState != RunState_ERROR) && (motorParamError != MotorParamTest_OK)) {
    switch (motorParamError) {
    case MotorParamTest_PWR_STG_OVERHEAT:
      mState = RunState_ERROR;
      usartWriteChars("\n\n\r  ERROR - 'testVariousMotorParam PWR_STG_OVERHEAT'");
      break;

    case MotorParamTest_TUMANAKO_MAX_BUS_V:
      mState = RunState_ERROR;
      printFormat("sIsIs","   BusVolt= ",mSTM32.getRawScaledBusVolt()," , MAX_BUS_V= ", TUMANAKO_MAX_BUS_V, "\n\r"); 
      usartWriteChars("\n\n\r  ERROR - 'testVariousMotorParam OVERVOLTAGE'");
      break;

    case MotorParamTest_UNDERVOLTAGE:
      //Comment this out because this case is checked after the contactors are engaged (see below)
      //usartWriteChars("ERROR - 'testVariousMotorParam UNDERVOLTAGE'");
      break;
      
    case MotorParamTest_BRK_HIGH:
      mState = RunState_ERROR;
      usartWriteChars("\n\n\r  ERROR - 'testVariousMotorParam BRK_HIGH (Inverter OVER_CURRENT or a few other potential things)'");
      break;

    default:
      mState = RunState_ERROR;
      usartWriteChars("\n\n\r  ERROR - 'testVariousMotorParam UNKNOWN!!?'");
      break;
    }
  }
  
  //Test for Watchdog Timer EWI (powerstage is already shut down by interupt, but this is to log info about the event)
  if (mSTM32.getWatchdogTimout() == TRUE) {
    usartWriteChars("\n\n\r  ERROR - 'testVariousMotorParam WATCHDOG TIMEOUT!' (red and blue, followed by flahsing) or POWERSTAGE (just solid blue) ERROR!");
    mState = RunState_ERROR;
  }
  
  //Test motor stator temp (max 125 deg C)
  if (mMotorTemp > TUMANAKO_MAX_MOTOR_TEMP) {
      if (mState != RunState_ERROR) usartWriteChars("\n\n\r  ERROR - 'MAX MOTOR TEMPERATURE'");
      mState = RunState_ERROR;
  }

  //Main state machine starts here
  switch (mState) {
    
//-----------------------
  case RunState_IDLE:
//-----------------------
    
#ifdef TUMANAKO_TEST
    
#ifdef TUMANAKO_PRECHARGE_TEST
    if (doPrecharge() != PreCharge_OK) {
#else
    if (false) {  //dont test precharge
#endif  //TUMANAKO_PRECHARGE_TEST
      mState = RunState_ERROR;
      usartWriteChars("\n\n\r  ERROR - 'Precharge failed.'");
    } else {
      usartWriteChars("\r\nTEST - IDLE State");
      mSTM32.motorInit(); //Do the init state
      mState=RunState_READY;  //READY state enables watchdog
      mSTM32.wait(5);
    }
    return;
#else //not TUMANAKO_TEST

    //power on, but nothing happening (waiting for ignition to be pushed)
    if (mSTM32.getIGN() == true) {
      //inidicate Contactors being engaged (all dash lights on)
      mSTM32.setRunLED(true);
      mSTM32.setErrorLED(true);

#ifdef TUMANAKO_PRECHARGE_TEST
      //do precharge (can only happen once per power cycle)
      if (doPrecharge() != PreCharge_OK) {
#else
      if (false) {  //dont test precharge
#endif //TUMANAKO_PRECHARGE_TEST
          //Precharge failed
        mState = RunState_ERROR;
        usartWriteChars("\n\n\r  ERROR - 'Precharge failed.'");
      } else {
        //Precharge Successful, continue with motor startup
        mSTM32.motorInit(); //Do the motor initalisation

        //inidicate Contactors engaged (just Run LED flashing)
        mFlashRunLED = true ;
        mSTM32.setErrorLED(false);

        mState=RunState_READY;
        mOldStart = false;  //prepare debounce logic (need to encapsulate this)
      }
    }
    break;
#endif //TUMANAKO_TEST

//-----------------------
  case RunState_READY:
//-----------------------

#ifdef TUMANAKO_ENABLE_WATCHDOG
    // Now that precharge is complete, we can enable the watchdog
    //(due to timers etc, precharge is hard to monitor with a watchdog)
    enableWatchdog();
#endif

#ifdef TUMANAKO_TEST
    usartWriteChars("\r\nTEST - READY State");
    mSTM32.motorStart();
    mState=RunState_RUN;
    RESET_WATCHDOG;
    mSTM32.wait(5);
    return;
#else

    //If IGN off, return to IDLE
    if ((mSTM32.getIGN() == false) && (mOldIGN == false)) {
      turnOff(); //shutdown power and reset
    }
    mOldIGN = mSTM32.getIGN();

    //TODO encapsulate buffer test (i.e. last 20 readings == true)
    if ((mSTM32.getStart()==true) && (mOldStart == true)) {
      //Check for start fault conditions

#ifndef TUMANAKO_SHIRE  //Shire Van is always in FWD
      if (!mSTM32.getNET()) {
        mState = RunState_ERROR;
        usartWriteChars("\n\n\r  ERROR - 'Must be in netural to start'");
        break;
      }
#endif
      
      if (motorParamError == MotorParamTest_UNDERVOLTAGE) { //UNDERVOLTAGE
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

//-----------------------
  case RunState_RUN:
//-----------------------
    
#ifdef TUMANAKO_TEST
    return;
#else
    mSTM32.motorTestForSpeedError();

    //If IGN off, stop the motor
    if ((mSTM32.getIGN() == false) && (mOldIGN == false)) {
      turnOff(); //shutdown power and reset
    }
    mOldIGN = mSTM32.getIGN();
    
#ifndef TUMANAKO_PETER_UTE
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
#endif
    
    break;
#endif //TUMANAKO_TEST

//-----------------------
    case RunState_ERROR:
//-----------------------

    mSTM32.shutdownPower();  //Turns off PWM and explicitly disengages the contactors
    mSTM32.setRunLED(false);
    mSTM32.setErrorLED(true);
    //Wait here for ever (the user will need to power down system to clear error and restart vehicle)
    usartWriteChars("\n\r  HALT");

    //TODO move all usartWriteChars calls to here to reduce time to shutdown.
    //Use variable to pass appropriate Error message.    

    //halt in an endless loop
    while (1) {
      delay(1);
      RESET_WATCHDOG;
     
      //if Watchdog caused the halt/error, flash the blue and red leds 'fast'!   
      static bool watchdog_flash_red_blue = true;
      if (mSTM32.getWatchdogTimout() == true && (mLastFlashTimer.getElapsed() > 75)) { 
        if (watchdog_flash_red_blue) {
          //PCC red and blue = watchdog timeout!
          mSTM32.setKiwiACBlueLED(true);  //turn KiwiAC blue light on
          mSTM32.setKiwiACRedLED(true);  //turn KiwiAC red light on
          watchdog_flash_red_blue = false;  //toggle red and blue LEDS off (for next time)
        } else{
          mSTM32.setKiwiACBlueLED(false);  //turn KiwiAC blue light off
          mSTM32.setKiwiACRedLED(false);  //turn KiwiAC red light off
          watchdog_flash_red_blue = true;  //toggle red and blue LEDS on (for next time)
        }
        mLastFlashTimer.reset();
      }
    }
    //break;  //commented out to remove compiler warning ('halt' while loop above prevents code ever getting here)

  default:
    //Error - should never get to here!
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
RESET_WATCHDOG;
    //signed short phaseC = getPhaseC(mSTM32.getPhase1(), mSTM32.getPhase2());
    signed short phaseC = mSTM32.getPhase3();
    float current = TK_ABS(mSTM32.getPhase1()) + TK_ABS(mSTM32.getPhase2()) + TK_ABS(phaseC);
    if (mState != RunState_ERROR) {
      usartWriteDisclaimer();
RESET_WATCHDOG;
      printFormat("sIs","       Bus (V): ",mSTM32.busVoltage(), " ");
        printFormat("sIs","pwrStgTemp (C): ",mPowerStageTemp, "\r\n");
      printFormat("sbs","         Brake:  ",mSTM32.getBrakeOn(), " ");
        printFormat("sIs"," motorTemp (C): ",mMotorTemp, "\r\n");
      printFormat("sIs","           RPM: ",mMotorRPM, " ");
      //printFormat("sIs","           IGN: ",(int)mSTM32.getIGN(), "\r\n");


        printFormat("sIs","Loop time (ms): ",mLoopTime, "\r\n");
      printFormat("sIs","   Phase 1 (A): ",mSTM32.getPhase1(), " ");
        printFormat("sIs"," PhAOffset (A): ",mSTM32.getPhaseAOffset(), "\r\n");
RESET_WATCHDOG;
      printFormat("sIs","   Phase 2 (A): ",mSTM32.getPhase2(), " ");
        printFormat("sIs"," PhBOffset (A): ",mSTM32.getPhaseBOffset(), "\r\n");
      printFormat("sIs","   Phase 3 (A): ",phaseC, " ");
        printFormat("sfs","     Total (A): ",current, "\r\n");
      printFormat("sIs","       Bus (A): ",mBusCurrent, " ");
        printFormat("sIs","      SlipFreq: ",mSTM32.getSlipFreq(), "\r\n");

      printFormat("sIs","   Bus Avg (A): ",mBusCurrentAvg, " ");
        printFormat("sIs","  AccRetardCnt: ",mAccelRetardCount, "\r\n");
        
      printFormat("sIs","     FluxAngle: ",convertToDegrees(mSTM32.getFluxAngle()), " ");
        printFormat("sIs"," RunTime (sec): ",mRunTimer.getElapsed()/1000, "\r\n");
//      printFormat("sIs" ," ElectricAngle: ",convertToDegrees(mSTM32.getElectricalAngle()), " ");
      printFormat("sIs","  rpmTorqLimit: ",mSpeedBasedTorqueLimit, " ");
        printFormat("sIs","RotorTimeConst: ",mSTM32.getRotorTimeConstant(), "\r\n");
 RESET_WATCHDOG; 
      // New 10/6/2012
      printFormat("sIs" ,"     Direction: ",mDirection, " ");
        printFormat("sbs","         Regen:  ",mSTM32.getEnableRegen(), "\r\n");
      printFormat("sIs","   RawAccelPOT: ",mRawAcceleratorRef, " "); 
        printFormat("sbs","  Torque Limit:  ",mTorqueLimitOccured, "\r\n");
        //printFormat("sIs","MotorTemp Freq: ",mSTM32.motorTempFreq(), "\r\n");
      printFormat("sIs","       Flux In: ",mFlux, " ");      
        printFormat("sIs","     Torque In: ",mAcceleratorRefSmooth, "\r\n");
      printFormat("sIs" ,"  Flux Out(Id): ",mSTM32.getFlux(), " ");
        printFormat("sIs","Torque Out(Iq): ",mSTM32.getTorque(), "\r\n");
      printFormat("sIs" ,"       Flux Vd: ",mSTM32.getFluxVd()," ");
        printFormat("sI","     Torque Vq: ",mSTM32.getTorqueVq());        
  
      count=0;
    }
  }
  count++;
}
