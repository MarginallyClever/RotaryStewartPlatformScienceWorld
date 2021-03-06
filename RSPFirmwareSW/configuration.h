#ifndef CONFIGURATION_H
#define CONFIGURATION_H
//------------------------------------------------------------------------------
// Stewart Platform v2 - Supports RUMBA 6-axis motor shield
// dan@marginallycelver.com 2013-09-20
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/RotaryStewartPlatform2 for more information.


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
#define VERBOSE              (0)  // increase this number to get more output

// display extra information about switches while operating normally
//#define DEBUG_SWITCHES

// move each stepper back and forth a short distance to confirm wiring is OK.  Used only in assembly.
//#define TEST_STEPPERS

// stream state of switches so they can be manually activated and confirmed OK.  Used only in assembly.
//#define TEST_SWITCHES

// Comms
#define BAUD                 (57600)  // How fast is the Arduino talking?
#define MAX_BUF              (64)  // What is the longest message we can store?

// Stepper motors
#define STEPS_PER_TURN       (200.0)  // depends on your stepper motor.  most are 200.
#define MICROSTEPS           (128.0)
#define MICROSTEPS_PER_TURN  (STEPS_PER_TURN*MICROSTEPS)
#define MICROSTEP_PER_DEGREE (MICROSTEPS_PER_TURN/360.0)

// speed and acceleration
#define MAX_FEEDRATE         (3000.0)  // depends on timer interrupt & hardware
#define MIN_FEEDRATE         (1.0)
#define DEFAULT_FEEDRATE     (3000.0)
#define DEFAULT_ACCELERATION (1500.0)

// misc
#define NUM_AXIES            (6)

// related to number of instructions that can be buffered.  must be a power of two > 1.
#define MAX_SEGMENTS         (32)  // must be a power of 2
// for splitting lines and look-ahead
#define MM_PER_SEGMENT       (3)

// convenience macros
#define TWOPI                (PI*2.0)
#define DEG2RAD              (PI/180.0)
#define RAD2DEG              (180.0/PI)

#define HAS_LCD

//------------------------------------------------------------------------------
// TIMER 
//------------------------------------------------------------------------------
// time passed with no instruction?  Make sure PC knows we are waiting.
#define TIMEOUT_OK           (1000)
// timing
#define CLOCK_FREQ           (16000000L)
#define MAX_COUNTER          (65536L)


// optimize code, please
#define FORCE_INLINE         __attribute__((always_inline)) inline

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START  unsigned char _sreg = SREG;  cli();
  #define CRITICAL_SECTION_END    SREG = _sreg;
#endif //CRITICAL_SECTION_START



//------------------------------------------------------------------------------
// hardware/software pins
//------------------------------------------------------------------------------
#define MOTOR_0_DIR_PIN    (16)
#define MOTOR_0_STEP_PIN   (17)
#define MOTOR_0_ENABLE_PIN (48)
#define MOTOR_0_LIMIT_PIN  (37)

#define MOTOR_1_DIR_PIN    (47)
#define MOTOR_1_STEP_PIN   (54)
#define MOTOR_1_ENABLE_PIN (55)
#define MOTOR_1_LIMIT_PIN  (36)

#define MOTOR_2_DIR_PIN    (56)
#define MOTOR_2_STEP_PIN   (57)
#define MOTOR_2_ENABLE_PIN (62)
#define MOTOR_2_LIMIT_PIN  (35)

#define MOTOR_3_DIR_PIN    (22)
#define MOTOR_3_STEP_PIN   (23)
#define MOTOR_3_ENABLE_PIN (27)
#define MOTOR_3_LIMIT_PIN  (34)

#define MOTOR_4_DIR_PIN    (25)
#define MOTOR_4_STEP_PIN   (26)
#define MOTOR_4_ENABLE_PIN (24)
#define MOTOR_4_LIMIT_PIN  (33)

#define MOTOR_5_DIR_PIN    (28)
#define MOTOR_5_STEP_PIN   (29)
#define MOTOR_5_ENABLE_PIN (39)
#define MOTOR_5_LIMIT_PIN  (32)

#define ESTOP_PIN          (6)


//------------------------------------------------------------------------------
// LCD
//------------------------------------------------------------------------------

#ifdef HAS_LCD
#define HAS_SD
#endif

// SD card settings
#define SDPOWER            -1
#define SDSS               53
#define SDCARDDETECT       49

#define LCD_HEIGHT         4
#define LCD_WIDTH          20

#define BLEN_C             2
#define BLEN_B             1
#define BLEN_A             0
#define encrot0            0
#define encrot1            2
#define encrot2            3
#define encrot3            1

// Smart controller settings
#define BEEPER             44
#define LCD_PINS_RS        19
#define LCD_PINS_ENABLE    42
#define LCD_PINS_D4        18
#define LCD_PINS_D5        38
#define LCD_PINS_D6        41
#define LCD_PINS_D7        40

// Encoder rotation values
#define BTN_EN1            11
#define BTN_EN2            12
#define BTN_ENC            43

//------------------------------------------------------------------------------
// EEPROM MEMORY MAP
//------------------------------------------------------------------------------
#define EEPROM_VERSION   2                         // Increment EEPROM_VERSION when adding new variables

#define ADDR_VERSION     0                         // address of the version number (one byte)
#define ADDR_GUID        (ADDR_VERSION+1)          // address of the UUID (long - 4 bytes)

// sensor adjustments
#define ADDR_ANGLE1      (ADDR_GUID  +4)
#define ADDR_ANGLE2      (ADDR_ANGLE1+4)
#define ADDR_ANGLE3      (ADDR_ANGLE2+4)
#define ADDR_ANGLE4      (ADDR_ANGLE3+4)
#define ADDR_ANGLE5      (ADDR_ANGLE4+4)
#define ADDR_ANGLE6      (ADDR_ANGLE5+4)


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
extern long robot_uid;


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
