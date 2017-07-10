//------------------------------------------------------------------------------
// RSPFirmwareSW.ino
// Rotary Stewart Platform Firmware Science World
//  - Supports RUMBA 6-axis motor shield
// dan@marginallycelver.com 2017-04-07
// First time you run the robot send R70 and R71 to initialize the motor angles.
//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/RotaryStewartPlatform2 for more information.


//------------------------------------------------------------------------------
// INCLUDES
//------------------------------------------------------------------------------
#include "configuration.h"

#include <SPI.h>  // pkm fix for Arduino 1.5

#include "vector3.h"
#include "segment.h"
#include "hexapod.h"
#include "sdcard.h"


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
// speeds
float feed_rate = DEFAULT_FEEDRATE; // how fast the EE moves in cm/s
float acceleration = DEFAULT_ACCELERATION;

// settings
char mode_abs = 1; // absolute mode?

// misc
long robot_uid = 0;
const int displayOutputPin = 20;  // high/low signal to trigger Science World's LED countdown displays


#define MAX_ANIMATIONS 3
unsigned long countDownTimer = 5000;  //5*1000;  // ms
unsigned long countDownStart;
int mode=0;
int animation=0;


//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------
/**
    finds angle of dy/dx as a value from 0...2PI
    @return the angle
*/
float atan3(float dy, float dx) {
  float a = atan2(dy, dx);
  if (a < 0) a = (PI * 2.0) + a;
  return a;
}


/**
   Delay for the appropriate time. delayMicroseconds() doesn't work for ms values > ~16k.
   @input us how many microseconds to wait
*/
void pause(long us) {
  delay(us / 1000);
  delayMicroseconds(us % 1000);
}


/**
   Set the feedrate (speed motors will move)
   @input nfr the new speed in steps/second
*/
float feedrate(float nfr) {
  if (feed_rate == nfr) return nfr; // same as last time?  quit now.

  if (nfr > MAX_FEEDRATE) {
    Serial.print(F("Feedrate set to maximum ("));
    Serial.print(MAX_FEEDRATE);
    Serial.println(F("steps/s)"));
    nfr = MAX_FEEDRATE;
  }
  if (nfr < MIN_FEEDRATE) { // don't allow crazy feed rates
    Serial.print(F("Feedrate set to minimum ("));
    Serial.print(MIN_FEEDRATE);
    Serial.println(F("steps/s)"));
    nfr = MIN_FEEDRATE;
  }
  feed_rate = nfr;

  return feed_rate;
}


/**
   display helpful information
*/
void help() {
  Serial.print(F("\n\nHELLO WORLD! I AM STEWART PLATFORM V4.2 #"));
  Serial.println(robot_uid, DEC);
  sayVersionNumber();
  Serial.println(F("== http://www.marginallyclever.com/ =="));
  Serial.println(F("I understand the following commands:"));
  Serial.println(F("Nx [A]*y (this is line number x, with checksum y)\n"\
                   "Where A is one of the following:\n"\
                   "G0 [Xx] [Yy] [Zz] (move to XYZ)\n"\
                   "G1 [Xx] [Yy] [Zz] (move to XYZ)\n"\
                   "G2 [Xx] [Yy] [Zz] (move anticlockwise in XY plane to XYZ)\n"\
                   "G3 [Xx] [Yy] [Zz] (move clockwise in XY plane to to XYZ)\n"\
                   "G4 [Pp] [Ss] (wait P seconds and S milliseconds)\n"\
                   "G28 (find home)\n"\
                   "G54-59 [Xx] [Yy] [Zz] (adjust tool offset)\n"\
                   "G90 (absolute movement mode)\n"\
                   "G91 (relative movement mode)\n"\
                   "M17 (enable motors)\n"\
                   "M18 (disable motors)\n"\
                   "M100 (help)\n"\
                   "M110 N* (set line number to *)\n"\
                   "M114 (where)\n"\
                   "UID * (write robot UID * to EEPROM)\n"\
                   "R60 [Aa] [Bb] [Cc] [Dd] [Ed] (set sensors adjustment to new value [abcde])\n"\
                   "R61 (display switch adjustment)\n"\
                   "R70 (write sensor adjustments to memory)\n"\
                   "R71 (reset switch adjustmentles to factory default)\n"));
  // See hexapod_position() for note about why G92 is removed
}


void sayVersionNumber() {
  char versionNumber = loadVersion();

  Serial.print(F("Firmware v"));
  Serial.println(versionNumber, DEC);
}


/**
   First thing this machine does on startup.  Runs only once.
*/
void setup() {
  loadConfig();

  pinMode(displayOutputPin,OUTPUT);

  Serial.begin(BAUD);  // open coms

  motor_setup();
  segment_setup();

  SD_init();
  LCD_init();

  hexapod_setup();
  feedrate(DEFAULT_FEEDRATE);  // set default speed
  
  help();  // say hello
  parser_ready();
}



/**
 * After setup() this machine will repeat loop() forever.
 */
void loop() {
  #ifdef TEST_STEPPERS
    for(int i=0; i<6; ++i) {
      Serial.print("motor ");
      Serial.println(i);
      test_motor(i);
    }
  #endif

  #ifdef TEST_SWITCHES
    test_switches();
  #endif
  
    parser_listen();  
    SD_check();
    
  #ifdef HAS_LCD
    LCD_update();
  #endif
  
  #ifdef HAS_SD
    if(sd_inserted==false) {
      mode=0;
    }
  else {
    switch(mode) {
      case 0: // mode 0 reset countdown
        // set countdown
        countDownStart=millis();
        digitalWrite(displayOutputPin,LOW);
        mode=1;
      case 1:
        // count down 90 seconds
        if(millis() - countDownStart >= countDownTimer ) {
          digitalWrite(displayOutputPin,HIGH);
          // launch next animation
          switch(animation) {
            case 0:  earthquakeVertical();  break;
            case 1:  earthquakeHorizontal();  break;
            case 2:  earthquakeWave();  break;
          }
          mode=2;
        }
        break;
      case 2:
        // when recording is done, restart the countdown timer.
        if(sd_printing_now == false) {
          animation = ( animation + 1 ) % MAX_ANIMATIONS;
          mode=0;
        }
        break;
    }
  }
  #endif
}


void earthquakeVertical  () {  SD_StartPrintingFile("waveU.ngc");  }
void earthquakeHorizontal() {  SD_StartPrintingFile("waveV.ngc");  }
void earthquakeWave      () {  SD_StartPrintingFile("waveZ.ngc");  }


/**
  This file is part of Stewart Platform v2.

  Stewart Platform v2 is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Stewart Platform v2 is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Stewart Platform v2. If not, see <http://www.gnu.org/licenses/>.
*/
