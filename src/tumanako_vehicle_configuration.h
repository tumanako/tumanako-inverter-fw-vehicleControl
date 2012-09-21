//------------------------------------------------------------------------------
//   TumanakoVC - Electric Vehicle and Motor control software
//   Copyright (C) 2012 Philip Court <philip@greenstage.co.nz>
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
//   Contains various defines which characterise the target vehicle
//   TODO A software configuration utility needs to be built to replace this
//
// HISTORY:
//   Philip Court 16/Sept/2012 - Extracted from tumanako_vehicle.cpp (note that
//   some of the vehicles below are missing the latest defines)
//------------------------------------------------------------------------------
#ifndef TUMANAKO_VEHICLE_CONFIGURATION_H
#define TUMANAKO_VEHICLE_CONFIGURATION_H

#include "tumanako_global.hpp"

#ifdef TUMANAKO_STANDARD  //E.g. Saker etc

#define THROTTLE_POT_MAX 63000
#define THROTTLE_POT_MIN 0
#define TORQUE_MAX 19000
#define TORQUE_MIN 0
#define ZERO_THROTTLE_TORQUE 0
#define START_REGEN_RAMP_RPM 750 //engine braking regen ends here
#define END_REGEN_RAMP_RPM 1950 //full regen (i.e. engine braking) starts here
#define GLIDE_RPM 5
#define ACCEL_DEAD_SPOT 20
#define BRAKE_TORQUE_REDUCTION 100  //How much torque is reduced by when brake is applied

#elif defined(TUMANAKO_SHIRE)

#define TUMANAKO_PRECHARGE_V 290  //voltage precharge is required to reach before main contactors engage (ShireVan specific)
#define TUMANAKO_MAX_BUS_V 400  //Max bus voltage we ever expect to see

#define THROTTLE_POT_MAX 22100 //measured 2371 (P1 180 Ohm)
#define THROTTLE_POT_MIN 3610 //measured 398  (P1 180 Ohm)
#define TORQUE_MAX 25000  //was 19000
#define TORQUE_MIN -500   //was -50 (-ve values provide engine braking style regen)
#define ZERO_THROTTLE_TORQUE 4900 //want to run at ~800 RPM (to be tuned) - was 1200 pre 28/4/2012
#define START_REGEN_RAMP_RPM 750 //engine braking regen ends here
#define END_REGEN_RAMP_RPM 950 //full regen (i.e. engine braking) starts here
#define GLIDE_RPM 800
#define ACCEL_DEAD_SPOT 20
#define BRAKE_TORQUE_REDUCTION 600  //How much torque is reduced by when brake is applied

#elif defined(TUMANAKO_PETER_UTE)

#define TUMANAKO_PRECHARGE_V 250  //voltage precharge is required to reach before main contactors engage
#define TUMANAKO_MAX_BUS_V 365  //Max bus voltage we ever expect to see

#define THROTTLE_POT_MAX 44000
#define THROTTLE_POT_MIN 200
#define TORQUE_MAX 24000  //was 24000 (10/6/2012) - was 20000 (21/8/2012), increased due to rpm based torque limit code
#define TORQUE_MIN -7500
#define ZERO_THROTTLE_TORQUE 0
#define START_REGEN_RAMP_RPM 350 //engine braking regen ends here (regen only provided above this RPM)
#define END_REGEN_RAMP_RPM 1950 //full regen (i.e. engine braking) starts here
#define GLIDE_RPM 220  //80 was to small
#define ACCEL_DEAD_SPOT 20
#define BRAKE_TORQUE_REDUCTION 4000  //How much torque is reduced by when brake is applied
// Speed based torque reduction paramters 40km/h = 3000rpm
#define TUMANAKO_TORQUE_LIMIT_RPM_START 2600
#define TUMANAKO_TORQUE_LIMIT_RPM_END 8600
#define TUMANAKO_SPEED_BASED_TORQUE_LIMIT_MIN 0
#define TUMANAKO_SPEED_BASED_TORQUE_LIMIT_MAX 12700 //scale is 20000 (Pete's Ute) to 1200 (test bench)
// DC Bus based torque reduction parameters - manage bus current limits (added 20120525)
// NOTE scale is A * 100
#define TUMANAKO_BUS_CURRENT_LIMIT_START 19000   //190 A - Pete's Ute (was 180A)
#define TUMANAKO_BUS_CURRENT_LIMIT_END 29000  //290 A  - Pete's Ute (was 260A)
#define TUMANAKO_BUS_CURRENT_LIMIT_EFFECT_MAX 5 //i.e. limit torque by a maximum of 10%

#elif defined(TUMANAKO_EDS_TEST_BENCH)

#define THROTTLE_POT_MAX 42500
#define THROTTLE_POT_MIN 0
#define TORQUE_MAX 19000
#define TORQUE_MIN -50
#define ZERO_THROTTLE_TORQUE 0
#define START_REGEN_RAMP_RPM 750 //engine braking regen ends here
#define END_REGEN_RAMP_RPM 1950 //full regen (i.e. engine braking) starts here
#define GLIDE_RPM 5
#define ACCEL_DEAD_SPOT 20
#define BRAKE_TORQUE_REDUCTION 100  //How much torque is reduced by when brake is applied

#else //i.e. Test bench by default

#define TUMANAKO_PRECHARGE_V 22  //voltage precharge is required to reach before main contactors engage
#define TUMANAKO_MAX_BUS_V 120  //Max bus voltage we ever expect to see

#define THROTTLE_POT_MAX 63000
#define THROTTLE_POT_MIN 0
#define TORQUE_MAX 1200
#define TORQUE_MIN -150
#define ZERO_THROTTLE_TORQUE 0
#define START_REGEN_RAMP_RPM 750
#define END_REGEN_RAMP_RPM 950
#define GLIDE_RPM 40  //increase to prevent coasting backwards on regen torque
#define ACCEL_DEAD_SPOT 10
#define BRAKE_TORQUE_REDUCTION 10  //How much torque is reduced by when brake is applied
// Speed based torque reduction paramters
#define TUMANAKO_TORQUE_LIMIT_RPM_START 40
#define TUMANAKO_TORQUE_LIMIT_RPM_END 90
#define TUMANAKO_SPEED_BASED_TORQUE_LIMIT_MIN 0
#define TUMANAKO_SPEED_BASED_TORQUE_LIMIT_MAX 80

// DC Bus based torque reduction parameters - manage bus current limits (added 20120525)
// NOTE scale is A * 100
#define TUMANAKO_BUS_CURRENT_LIMIT_START 100   //1.00 A - Test Bench
#define TUMANAKO_BUS_CURRENT_LIMIT_END 420  //4.20 A - Test Bench
#define TUMANAKO_BUS_CURRENT_LIMIT_EFFECT_MAX 80 //i.e. limit torque by a maximum of 80%

#endif

#endif //TUMANAKO_VEHICLE_CONFIGURATION_H
