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
#include "tumanako_inverter.hpp"
#include "tumanako_serial.hpp"

/*----------------------------- Private typedef ------------------------------*/
/*----------------------------- Private define -------------------------------*/
#define TUMANAKO_MAX_REGEN_POWER 40000000
#define EVAL_BOARD_POT 14 //PC4
#define EVAL_BOARD_MIC 15 //PC5

#define THROTTLE_POT_MAX 32000 //53.1% MAX (use 60%)
#define THROTTLE_POT_MIN 1000 //5.5% MIN (use 2.5%)
#define THROTTLE_ERROR_COUNT 6

//------------------------------------------------------------------------------
//Constructor
TumanakoInverter::TumanakoInverter():
  mState(RunState_IDLE),
  mOldIGN(true),
  mOldStart(false),
  mAcceleratorRef(0),
  mPrevAccRef(0),
  mFlashRunLED(false),
  mAcceleratorMIN(10),  //TODO - To be configured via setup software
  mAcceleratorMAX(5000),  //TODO - To be configured via setup software
  mCountMinThrottleError(0),
  mCountMaxThrottleError(0)
{  

}

//------------------------------------------------------------------------------
Direction_T TumanakoInverter::getDirection(void)
{
   if (mSTM32.getFWD() && !mSTM32.getREV()) return(FWD);
   else if (mSTM32.getREV() && !mSTM32.getFWD()) return(REV);
   else if (mSTM32.getNET()) return(NET);
   else
   {
     //TODO: Flault001 - FWD and REV inputs inconsistant (e.g. both FWD and REV selected)
      //mSTM32.setErrorLED(true);  //TODO record the fault number to log port
      return(NET);  //Netural
   }
}

//Check the inputs from the driver and take appropriate actions...
void TumanakoInverter::checkVehcileControlInputs()
{
   //Check direction
   if (getDirection()) return;//doit
}

//flash the LEDs
void TumanakoInverter::flash()
{
  static int led_on=0;
  
  //crude flashng code (toggle a LED), TODO use a timer
  if (led_on==0)
    if (mFlashRunLED) mSTM32.setRunLED(true);
  if (led_on==50)
  {
    if (mFlashRunLED) mSTM32.setRunLED(false);
  }
  led_on += 1;
  if (led_on >= 100) led_on=0;
}


//Main application loop
void TumanakoInverter::doIt(void)
{
   signed long power = 0;
   signed short motorRPM = 0;
   
   //clear lights
   mSTM32.setRunLED(false);
   mSTM32.setErrorLED(false);
   
   //init STM32
   mSTM32.init(); 
   usartInit();

   while(1)
   {
      flash();  //code to manage flashing LEDS on dash when needed.
      
      //Check Vehicle control inputs
      checkVehcileControlInputs();

      //Read throttle POT position
      mRawAcceleratorRef = mSTM32.readADC(EVAL_BOARD_MIC);
      mAcceleratorRef = mRawAcceleratorRef - 3000;  //was /2 -1700
      
      //Check throttle limits (indicates wiring fault)
      if (mRawAcceleratorRef > THROTTLE_POT_MAX) 
      {
        mCountMaxThrottleError++;
        if (mCountMaxThrottleError > THROTTLE_ERROR_COUNT)
        {
          if (mState != RunState_ERROR) usartWriteChars("ERROR - 'POT_MAX exceeded! Check Throttle wires.'");
          mState = RunState_ERROR;
          mAcceleratorRef = 0;
        }
      }
      else mCountMaxThrottleError = 0; //reset error count
        
      if (mRawAcceleratorRef < THROTTLE_POT_MIN) 
      {
        mCountMinThrottleError++;
        if (mCountMinThrottleError > THROTTLE_ERROR_COUNT)
        {
          if (mState != RunState_ERROR) usartWriteChars("ERROR - 'POT_MIN exceeded! Check Throttle wires.'");
          mState = RunState_ERROR;
          mAcceleratorRef = 0;
        }
      } 
      else mCountMinThrottleError=0; //reset error count
      
        
      //prevent regen if RPM < 10
      motorRPM = mSTM32.getRPM();
      if ((mAcceleratorRef < 0) && (motorRPM < 10))
      {
         mAcceleratorRef = 0;
      }

      //provide dead spot in pot curve
      if ((mAcceleratorRef > -20) && (mAcceleratorRef <20))
      {
         mAcceleratorRef = 0;
      }
      
      //Detect crawl mode and reduce torque appropriately
      if (mSTM32.getCrawl() == true) 
      {
        mAcceleratorRef/=4;
        if ((motorRPM > 50)) mAcceleratorRef=0;  //no torque if RPM above limit
      }
            
      if (mSTM32.getREV()) mAcceleratorRef = -mAcceleratorRef;
      else if (mSTM32.getNET()) 
      {  
        mAcceleratorRef = 0;
      }

      //TODO simple traction control logic
      //...
        
      //A little bit of manual tuning to give more current to Torque and less to Flux as RPM increases
      if (motorRPM<50 && mAcceleratorRef>200) 
      {
        mSTM32.setFlux(9107);
      }
      else if (mAcceleratorRef==0)
      {
        mSTM32.setFlux(3000);  //keep stationary flux low (reduce power consumption and cool down!)
      }
      else if (motorRPM > 200)//RPM > 200 or mAcceleratorRef < 200 (but not 0)
      {
        mSTM32.setFlux(5000);
      }
      else //RPM >50 but < 200 or mAcceleratorRef < 200 (but not 0)
      {
        mSTM32.setFlux(6000);
      }
      

      mSTM32.setTorque(mAcceleratorRef);
      mPrevAccRef = mAcceleratorRef;

      stateMachineDo();   //new open source main loop
      
      if (mState != RunState_ERROR) usartWrite(motorRPM, mSTM32.getFlux(), mAcceleratorRef, mSTM32.getSpeed(), mSTM32.getTorque());  //print stuff to RS232
      
      //mSTM32.wait(50);
      
      if (mState == RunState_RUN) 
      {
        //Only test if motor on!
        power = mSTM32.getTorque() * mSTM32.getRPM();
        if (!mSTM32.busVoltageOK())// || (power > TUMANAKO_MAX_REGEN_POWER))
        {
           //Go to error state and Log error to serial
           mState = RunState_ERROR;
           usartWriteChars("ERROR - 'TUMANAKO_MAX_REGEN_POWER exceeded!'");  //print stuff to RS232
           //break;
        }
      }
   }
}


void TumanakoInverter::stateMachineDo(void)
{
  //Check we are not exceeding any limits
  int motorParamError = mSTM32.testVariousMotorParam();
  if ((mState != RunState_ERROR) && (motorParamError != 0))
  {
    switch (motorParamError)
    {
    case 1:
    mState = RunState_ERROR;
    usartWriteChars("ERROR - 'testVariousMotorParam OVERHEAT'");
    break;
    
    case 2:
    mState = RunState_ERROR;
    usartWriteChars("ERROR - 'testVariousMotorParam OVERVOLTAGE'");
    break;
    
    case 3:
    //This case in checked after the contactors are engaged
    //usartWriteChars("ERROR - 'testVariousMotorParam UNDERVOLTAGE'");
    break;
    
    default:
    mState = RunState_ERROR;
    usartWriteChars("ERROR - 'testVariousMotorParam UNKNOWN!!?'");
    break;
    } 
  }
  
  switch (mState)
  {
  case RunState_IDLE:
    //power on, but nothing happening (waiting for ignition to be pushed)
    if (mSTM32.getIGN() == true)
    {
      //inidicate Contactors being engaged (all dash lights on)
      mSTM32.setRunLED(true);
      mSTM32.setErrorLED(true);
      
      //do precharge (only do it once though!  State machine needed here)
      mSTM32.doPrecharge();
      
      mSTM32.motorInit(); //Do the init state
      
      //inidicate Contactors engaged (just Run LED flashing)
      mFlashRunLED = true ;
      mSTM32.setErrorLED(false);
      
      mState=RunState_READY;
      mOldStart = false;  //prepare debounce logic (need to encapsulate this)
    }
    break;

  case RunState_READY:
    //Precharge begins (and contactors engaged! Undervoltage check confirms this)
    
    //If IGN off, return to IDLE
    if ((mSTM32.getIGN() == false) && (mOldIGN == false))
    {
      mState=RunState_IDLE;
      mFlashRunLED = false;  
      mSTM32.setRunLED(false);
    }
    mOldIGN = mSTM32.getIGN();

    if ((mSTM32.getStart()==true) && (mOldStart == true))
    {
      //Check for start fault conditions
      if (!mSTM32.getNET())
      {
        mState = RunState_ERROR;
        usartWriteChars("ERROR - 'Must be in netural to start'");
        break;
      }
      
      if (motorParamError == 3) //UNDERVOLTAGE - TODO enum!!!
      {
        mState = RunState_ERROR;
        usartWriteChars("ERROR - 'testVariousMotorParam UNDERVOLTAGE'");
        break;
      }
    
      if (mAcceleratorRef != 0)  //throttle must be zero when start button pushed
      {
        mState=RunState_ERROR;
        usartWriteChars("ERROR - 'throttle must be zero when start button pushed'."); 
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
    
  case RunState_RUN:
    mSTM32.motorTestForSpeedError();
      
    //If IGN off, stop the motor
    if ((mSTM32.getIGN() == false) && (mOldIGN == false))
    {
      mSTM32.shutdownPower();
      mState=RunState_IDLE;
      mSTM32.setRunLED(false);
    }
    mOldIGN = mSTM32.getIGN();
    break;
    
  case RunState_ERROR:
      mSTM32.shutdownPower();  //TODO - need to explicitly disengaged contactors
      mSTM32.setErrorLED(true);
      //TODO move all usartWriteChars calls to here to reduce time to shutdown.
      //Use variable to pass appropriate Error message.
    break;

  default:
    //Error - should not get here!
    usartWriteChars("ERROR - 'Unknown State!!! Going to Error State now.'");
    mState=RunState_ERROR;
    break;
  }
}