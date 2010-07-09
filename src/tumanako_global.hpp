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

#define TUMANKAO_VERSION "0.90.6 ShireVan"

#define TUMANAKO_PRECHARGE_V 250  //voltage precharge is required to reach before main contactors engage (ShireVan specific)

//Which Motor parameters - comment all of these for the test rig (or uncomment the appropriate alternative application)
//#define TUMANAKO_WS28  //uncomment this to build for the WS28 motor in the Saker
#define TUMANAKO_WS20  //uncomment this to build for the 1 PV5133-4WS20 W11 motor in the Shire Van

//Which Vehicle Control paramters - comment all of these for the test rig (or uncomment the appropriate alternative application)
//#define TUMANAKO_STANDARD   //uncomment this to build standard vehicle control
#define TUMANAKO_SHIRE   //uncomment this to build for the Shire Van (fixed in FWD, runs at 1000 RPM during idle to run pumps etc) 

//Which powerstage - uncomment for SKiiP
#define TUMANAKO_SKAI

//Which control board - uncomment for KiwiAC
#define TUMANAKO_KIWIAC //uncomment this to use with KiwiAC control board (as apposed to orginal ST Eval board and Tumanako interface board)

//#define TUMANAKO_TEST //uncomment this to run automated test (engage contactors and run for 30 sec)

#define TK_DYNAMIC_TIMECONSTANT //uncomment to allow runtime changes to the Rotor Time Constant (via Crawl digital input - HACK)


#endif