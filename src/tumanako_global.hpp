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

#define TUMANKAO_VERSION "0.96.0 open" //'open' denotes a version with 100% open source code

#define TUMANAKO_PRECHARGE_V 290  //voltage precharge is required to reach before main contactors engage (ShireVan specific)
#define TUMANAKO_MAX_BUX_V 400  //Max bus voltage we ever expect to see
#define TUMANAKO_MIN_PRECHARGE_TIME 1000 //1 sec - No matter what happens a successful precharge will take at least this long
#define TUMANAKO_MAX_PRECHARGE_TIME 3000 //3 sec - If a precharge takes longer than this, precharge is considered a failure.
#define TUMANAKO_PRECHARGE_FEEDBACK_TIME 45 //25 millisec (gigavac), 45 (kilovac) - Allow time for contactors to change
#define TUMANAKO_RUN_TIMER //enforce a max runtime
//#define TUMANAKO_MAX_RUNTIME 7*24*60*60*1000 //7 day max runtime
#define TUMANAKO_MAX_RUNTIME 30*1000 //30 sec max runtime (useful for testing)
#define TUMANAKO_USE_FILTER //Turns the software filters for digital signals on
#define TUMANAKO_HALF_POWER_MOTOR_TEMP 115 //115 deg C - Half system power
#define TUMANAKO_MAX_MOTOR_TEMP 125 //125 deg C - Shutdown system

//Which Motor parameters - comment all of these for the test rig (or uncomment the appropriate alternative application)
//#define TUMANAKO_WS28  //uncomment this to build for the WS28 motor in the Saker
//#define TUMANAKO_WS20  //uncomment this to build for the 1 PV5133-4WS20 W11 motor in the Shire Van

//Which Vehicle Control paramters - comment all of these for the test rig (or uncomment the appropriate alternative application)
//#define TUMANAKO_STANDARD   //uncomment this to build standard vehicle control
//#define TUMANAKO_SHIRE   //uncomment this to build for the Shire Van (fixed in FWD, runs at 1000 RPM during idle to run pumps etc) 

//Which powerstage - uncomment for SKAI (comment out for SKiiP)
//#define TUMANAKO_SKAI

#define TUMANAKO_BRK_HIGH //Shire Van and board 002) also 001 by the looks!?

//Which control board - uncomment for KiwiAC
#define TUMANAKO_KIWIAC //uncomment this to use with KiwiAC control board (as apposed to orginal ST Eval board and Tumanako interface board)

//#define TUMANAKO_THREE_RELAY //uncomment for old style contactor board with 3 gigavacs (as apposed to the new 2 contactor 1 IGBT soln)
#define TUMANAKO_PRECHARGE_ITERATION_WAIT 100  //number of millisec to wait between precharge iterative tests

//These should only ever be UNCOMMENTED on the test bench!
//#define TUMANAKO_TEST //uncomment this to run automated test (engage contactors and run for 30 sec)
#define TUMANAKO_TEST_TORQUE_REF 500

//These should only ever be COMMENTED OUT on the test bench!
//#define TUMANAKO_PRECHARGE_TEST //comment this to remove automated precharge test (useful during test becnch testing)
//#define TUMANAKO_UNDERVOLTAGE_TEST //comment this to remove undervoltage test (useful during test becnch testing)

#endif

