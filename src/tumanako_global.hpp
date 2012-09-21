//------------------------------------------------------------------------------
//   TumanakoVC - Electric Vehicle and Motor control software
//   Copyright (C) 2010 Philip Court <philip@greenstage.co.nz>
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
#ifndef TUMANAKO_GLOBAL_HPP
#define TUMANAKO_GLOBAL_HPP

#define TUMANKAO_VERSION "0.98.16 open" //'open' denotes a version with 100% open source code

#define TUMANAKO_MAX_REGEN_CURRENT_A 6000  //Max regen charge currnet x 100 (i.e. a value of 6000 limits regen to 60A)
#define TUMANAKO_CRAWL_RPM_LIMIT 4000  //Max RPM when crawl switch is activated

#define TUMANAKO_MIN_PRECHARGE_TIME 1000 //1 sec - No matter what happens a successful precharge will take at least this long
#define TUMANAKO_MAX_PRECHARGE_TIME 3000 //3 sec - If a precharge takes longer than this, precharge is considered a failure.
#define TUMANAKO_PRECHARGE_FEEDBACK_TIME 45 //25 millisec (gigavac), 45 (kilovac) - Allow time for contactors to change
#define TUMANAKO_RUN_TIMER //enforce a max runtime
//#define TUMANAKO_MAX_RUNTIME 7*24*60*60*1000 //7 day max runtime
#define TUMANAKO_MAX_RUNTIME 30*1000 //30 sec max runtime (useful for testing)
//#define TUMANAKO_ENABLE_WATCHDOG  //comment out to disable watchdog (e.g. for testing contactors)

//Configured so that 115 deg C = Half system power (torque and flux)
#define TUMANAKO_MOTOR_TEMP_LIMIT_END 120 //120 Deg C = zero power
#define TUMANAKO_MOTOR_TEMP_LIMIT_START 110 //110 Dec C = full power
#define TUMANAKO_MOTOR_TEMP_LIMIT_RANGE (TUMANAKO_MOTOR_TEMP_LIMIT_END - TUMANAKO_MOTOR_TEMP_LIMIT_START)
#define TUMANAKO_MAX_MOTOR_TEMP 125 //125 deg C - Shutdown system (Final safety test)

//#define TUMANAKO_RPM_BASED_FLUX  //old approach to flux control (comment out for new torque/flux matching approach)

#define DC_BUS_CURRENT_BASED_TORQUE_LIMIT  //turns on DC Bus current based torque limit logic 
#define SPEED_BASED_TORQUE_LIMIT  //turns on speed based torque limit logic

//Which Motor parameters - comment all of these for the test rig (or uncomment the appropriate alternative application)
//#define TUMANAKO_WS28  //uncomment this to build for the WS28 motor in the Saker
//#define TUMANAKO_WS20  //uncomment this to build for the 1 PV5133-4WS20 W11 motor in the Shire Van
//#define TUMANAKO_SWITCH_EV  //uncomment this to build for the Holden Ute Motor built by Peter Sewel (Switch EV)

//Which Vehicle Control paramters - comment all of these for the test rig (or uncomment the appropriate alternative application)
//#define TUMANAKO_STANDARD   //uncomment this to build standard vehicle control
//#define TUMANAKO_SHIRE   //uncomment this to build for the Shire Van (fixed in FWD, runs at 1000 RPM during idle to run pumps etc) 
//#define TUMANAKO_PETER_UTE   //uncomment this to build for PETES UTE Vehicle control
//#define TUMANAKO_EDS_TEST_BENCH   //uncomment this to build for EDS TEST BENCH

//Define which temperature sensor motor has (only define one)
#define TUMANAKO_MT_USE_KTY84  //Used in standard WS20 (and WS28?)
//#define TUMANAKO_MT_USE_PT100  //Used in Shire Van modified WS20
//#define TUMANAKO_MT_USE_DUAL_LINEAR_EQ  //increases accuracy - only relevant for PT100 currently

//Which powerstage - uncomment for SKAI (comment out for SKiiP or PowerEx)
//#define TUMANAKO_SKAI

#define TUMANAKO_BRK_HIGH //Shire Van, board 002, 001 and all the others...
//
//Which control board - uncomment for KiwiAC
#define TUMANAKO_KIWIAC //uncomment this to use with KiwiAC control board (as apposed to orginal ST Eval board and Tumanako interface board)

//#define TUMANAKO_THREE_RELAY //uncomment for old style contactor board with 3 gigavacs (as apposed to the new 2 contactor 1 IGBT soln)
#define TUMANAKO_PRECHARGE_ITERATION_WAIT 100  //number of millisec to wait between precharge iterative tests

//These should only ever be *UNCOMMENTED* on the test bench!
//#define TUMANAKO_TEST //uncomment this to run automated test (engage contactors and run for 30 sec) - TODO rename TUMANAKO_RUN_TEST
//#define TUMANAKO_TEST_TORQUE_REF 100
//#define TK_DYNAMIC_TIMECONSTANT //uncomment to allow motor specific (runtime changes to the Rotor Time Constant are disabled now (via Crawl digital input - HACK)

//These should only ever be *COMMENTED OUT* on the test bench!
//#define TUMANAKO_PRECHARGE_TEST //comment this to remove automated precharge test (useful during test becnch testing)
//#define TUMANAKO_UNDERVOLTAGE_TEST //NOT IMPLEMENTED YET!!! - comment this to remove undervoltage test (useful during test becnch testing)

#endif
