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
//------------------------------------------------------------------------------

/*******************************************************************************
 * File Name          : STM32_interface.h
 * Author             : Philip Court
 * Date First Issued  : 11/Aug/2009
 * Description        : STM32 lib Main program interface (wraps all ST licensed
 *                      code).  This is actually a generic wrapper that can wrap
 *                      any motor control lib.  TODO change file name.
 *******************************************************************************
 * History:
 * 11/Aug/09 v1.0 - PCC: First Cut
 *  7/Dec/09 v1.1 - PCC: Prepared for first release.
 ******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32_MAIN_H
#define __STM32_MAIN_H

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
typedef enum {Input_IGN, Input_START, Input_CRAWL, Input_FWD, Input_REV} Input_T;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

class STM32Interface {
public:

  //STM32Interface::STM32Interface(){};

  /**IO from the vehcile loom*/
  
  //Returns the selected Input with debounce logic applied (implemented in open source without function pointers)
  bool getDebouncedInput(Input_T input);
  
  //Physical INPUT
  bool getIGN(void);  //Ignition switch on or off?
  bool getStart(void);  //Start button engaged?
  bool getCrawl(void);  //Crawl switch on or off?
  bool getFWD(void);   //Forward selected?
  bool getREV(void);  //Reverse selected?
  
  bool getNET(void);  //Netural (neither FWD or REV selected)?
  bool getContactorsEngaged(void);  //Are the contactors engaged (Physical test)?
  bool getEmergencyStop(void);  //Has the emergency stop been activated?
  
  //Physical OUTPUT
  void setErrorLED(bool value); //Show red error light
  void setRunLED(bool value); //Show green run light
  
  /** Reads the value of the specified ADC channel */
  unsigned short readADC(unsigned char channel);  //:TODO: needs to be encapsulated into +ve torque and -ve torque etc
  
  /** Used to init the Motor Control libraries and in particular every thing needed by mainLoop*/
  int init(void);
  
  /** Main service fuinction that services the motor control logic (to be called in a loop) */
  int mainLoop(void); //TODO - remove this as this main loop is now implemented in Tumanako vehicle control code
  
  // Getter/Setters for the current torque setting (Iq)
  signed short getTorque(void); //read current value of Torque
  void setTorque(signed short); //Set the target Torque value (used in torque control algorithm)
  
  // Getter/Setters for the current torque setting (Iq)
  signed short getSpeed(void); //read current value of Torque
  void setSpeed(signed short); //Set the target Torque value (used in torque control algorithm)
 
  unsigned long getGlobalFlags(void);
    
  // Getter/Setters for the current flux setting (Id)
  signed short getFlux(void); //read current value of rotor Flux
  void setFlux(signed short); //Set the target rotor Flux value (used in torque control algorithm)
  
  // Getter/Setters for the current speed (RPM) setting
  signed short getRPM(void);  //read current value of motor RPM
  void setRPM(signed short);  //Set the target RPM (speed control loop only - not currently supported by Tumanako)
  
  //Is the bus voltage OK? (TODO - more work needed here.  Must expose limits etc...)
  bool busVoltageOK(void);
 
  //Get current bus voltage (TODO - define scales etc)
  short busVoltage(void);
  
  //returns 0 if no issues 
  short testVariousMotorParam(void);
  
  //Contactor controls (See diagram here: http://liionbms.com/php/precharge.php)
  void setK1(bool status);
  void setK2(bool status);
  void setK3(bool status);
  
  //initiate precharge logic (TODO, extract this to Tumanako level)
  bool doPrecharge(void);
  
  void wait(unsigned short time);

  //Sanity checks (TODO paramatise this)
  void checkPowerStageLimits();
  
  //Prep motor for start (TODO document details)
  void motorInit();
  
  //Start motor (TODO document details)
  void motorStart();

  //test to be executed in the main loop when motor is running (TODO document details)
  void motorTestForSpeedError();
  
  //test to check for motor fault (TODO document this)
  bool motorDoesFaultStillExist();
  
  //Shutdown motor control and power stage
  void shutdownPower();
  
  void evalBoardFeatures();   //TODO remove (this is STM32 motor kit specific)
};
#endif /* __STM32_MAIN_H */
/*******************************************************END OF FILE************/
