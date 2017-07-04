#ifndef HEXAPOD_H
#define HEXAPOD_H
//------------------------------------------------------------------------------
// Stewart Platform v2 - Supports RUMBA 6-axis motor shield
// dan@marginallycelver.com 2013-09-20
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/RotaryStewartPlatform2 for more information.


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "configuration.h"


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------

// measurements based on computer model of robot
#define NUM_TOOLS            (6)
#define BICEP_LENGTH         (6.03f) // changed from 11.5 2017-06-19 J. Craver
#define FOREARM_LENGTH       (36.83f)

// individually correct each switch for best calibration.
// default value taken from the 3D model.
#define SWITCH_ANGLE1        (90.0-18.0)
#define SWITCH_ANGLE2        (90.0-18.0)
#define SWITCH_ANGLE3        (90.0-18.0)
#define SWITCH_ANGLE4        (90.0-18.0)
#define SWITCH_ANGLE5        (90.0-18.0)
#define SWITCH_ANGLE6        (90.0-18.0)

// top center to wrist hole (relative): X7.635 Y+/-0.553 Z0.87
// in Robot Overlord this is the "WRIST_TO_FINGER_" value
#define T2W_X                ( 2.1f)
#define T2W_Y                (15.561f)
#define T2W_Z                ( 3.4f)
// base center to shoulder hole (relative): X8.093 Y+/-2.15 Z7.831
// in Robot Overlord this is the "BASE_TO_SHOULDER_" value
#define B2S_X                (22.534f)  // was (20.534f) changed 2017-06-19 J. Craver
#define B2S_Y                ( 5.45f )
#define B2S_Z                ( 4.925f)

#define STARTING_FINGER_Z    (43.7608f)
#define BASE_ADJUST_Z        (8.0952f)

//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------

struct EndEffector  {
  Vector3 up;
  Vector3 left;
  Vector3 forward;
  Vector3 pos;
  Vector3 relative;
  float r,p,y;  // roll, pitch, yaw
};


struct Arm {
  Vector3 shoulder;
  Vector3 elbow;
  Vector3 shoulder_to_elbow;
  Vector3 wrist;

  float angle;
  int new_step;

  int motor_step_pin;
  int motor_dir_pin;
  int motor_enable_pin;
  int motor_scale;  // 1 or -1

  char limit_switch_pin;
  int limit_switch_state;
};


struct Hexapod {
  Arm arms[NUM_AXIES];
  EndEffector ee;
  Vector3 base;
  Vector3 tool_offset[NUM_TOOLS];
  int current_tool;
  
  Vector3 v;  // max XYZ vel
  Vector3 destination;  // target XYZ position
  
  long steps_to_zero[NUM_AXIES];
};


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
extern Hexapod h;


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
void hexapod_setup();
void hexapod_update_ik(Vector3 &mov,Vector3 &rpy);
void hexapod_line(float newx,float newy,float newz,float newu,float newv,float neww,float new_feed_rate);


/**
* This file is part of Stewart Platform v2.
*
* Stewart Platform v2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Stewart Platform v2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with Stewart Platform v2. If not, see <http://www.gnu.org/licenses/>.
*/
#endif
