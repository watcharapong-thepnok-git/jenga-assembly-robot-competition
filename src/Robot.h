#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <Bounce2.h>
#include <Adafruit_NeoPixel.h>

#include "TCA9548A_Switch.h"
#include "VL6180_Sensor.h"
#include "MovementControl.h"
#include "RobotController.h"

namespace Robot {

/* Mission */
inline bool missionRedTeam = false;
inline bool missionBlueTeam = false;

/* Neopixel */
const uint8_t neoPixelPin = 23;    
const uint16_t neoPixelCount = 6;

inline Adafruit_NeoPixel pixels(neoPixelCount, neoPixelPin, NEO_GRB + NEO_KHZ800);

/* Robot Buttons */ 
const uint8_t buttonGreenPin = 39;
const uint8_t buttonYellowPin = 40;
const uint8_t buttonRedPin = 41;

inline Bounce2::Button buttonGreen = Bounce2::Button();
inline Bounce2::Button buttonYellow = Bounce2::Button();
inline Bounce2::Button buttonRed = Bounce2::Button();

/* I2C Switches */ 
inline TCA9548A_Switch i2cSwitch1(&Wire, 0x70);
inline TCA9548A_Switch i2cSwitch2(&Wire, 0x71);

/* ToF Sensors */ 
inline VL6180_Sensor tofFront1(&Wire, []() {i2cSwitch1.deselectAllChannels(true); i2cSwitch2.selectSingleChannel(4, true);}); // 2-4 *
inline VL6180_Sensor tofFront2(&Wire, []() {i2cSwitch2.deselectAllChannels(true); i2cSwitch1.selectSingleChannel(3, true);}); // 1-3 *

inline VL6180_Sensor tofRight1(&Wire, []() {i2cSwitch2.deselectAllChannels(true); i2cSwitch1.selectSingleChannel(5, true);}); // 1-5 !
inline VL6180_Sensor tofRight2(&Wire, []() {i2cSwitch2.deselectAllChannels(true); i2cSwitch1.selectSingleChannel(6, true);}); // 1-6 *

inline VL6180_Sensor tofBack1(&Wire, []() {i2cSwitch2.deselectAllChannels(true); i2cSwitch1.selectSingleChannel(7, true);});  // 1-7 *
inline VL6180_Sensor tofBack2(&Wire, []() {i2cSwitch1.deselectAllChannels(true); i2cSwitch2.selectSingleChannel(7, true);});  // 2-7 *

inline VL6180_Sensor tofLeft1(&Wire, []() {i2cSwitch1.deselectAllChannels(true); i2cSwitch2.selectSingleChannel(6, true);});  // 2-6 *
inline VL6180_Sensor tofLeft2(&Wire, []() {i2cSwitch1.deselectAllChannels(true); i2cSwitch2.selectSingleChannel(5, true);});  // 2-5 !

inline VL6180_Sensor tofGrip1(&Wire, []() {i2cSwitch1.deselectAllChannels(true); i2cSwitch2.selectSingleChannel(3, true);});  // 2-3 *
inline VL6180_Sensor tofGrip2(&Wire, []() {i2cSwitch2.deselectAllChannels(true); i2cSwitch1.selectSingleChannel(4, true);});  // 1-4 *

/* Servo Motors */ 
inline Servo servoLift1; /* PIN: 36   RANGE: 0-180   LIFTING: WRITE(180)   STOP: WRITE(90) */ 
inline Servo servoLift2; /* PIN: 37   RANGE: 0-180   LIFTING: WRITE(0)     STOP: WRITE(90) */
inline Servo servoLift3; /* PIN: 13   RANGE: 0-180   LIFTING: WRITE(0)     STOP: WRITE(90) */
inline Servo servoLift4; /* PIN: 14   RANGE: 0-180   LIFTING: WRITE(180)   STOP: WRITE(90) */

inline Servo servoGripLower; /* PIN: 15   RANGE: 0-180   GRIPPING: WRITE(0)   RELEASING: WRITE(180) */
inline Servo servoGripUpper; /* PIN: 33   RANGE: 0-180   GRIPPING: WRITE(0)   RELEASING: WRITE(180) */

/* Movement System */ 
inline MovementSystemConfig movementConfig = {
  .drv_in1_l = 7,   /* Left Motor  */ 
  .drv_in2_l = 6,

  .drv_in1_r = 8,   /* Right Motor */ 
  .drv_in2_r = 9,

  .drv_in1_b = 11,  /* Back Motor  */ 
  .drv_in2_b = 10,

  .kp_l = 8.0f,   /* Left Motor  */ 
  .ki_l = 0.05f,
  .kd_l = 2.4f,

  .kp_r = 7.95f,  /* Right Motor */ 
  .ki_r = 0.05f,
  .kd_r = 2.4f,

  .kp_b = 7.95f,  /* Back Motor  */ 
  .ki_b = 0.05f,
  .kd_b = 2.4f,

  .encoder_pin_a_l = 4,
  .encoder_pin_b_l = 5,  /* Left Encoder  */

  .encoder_pin_a_r = 1,
  .encoder_pin_b_r = 0,  /* Right Encoder */ 

  .encoder_pin_a_b = 3,
  .encoder_pin_b_b = 2,  /* Back Encoder  */ 

  .maximum_allow_wheel_speed = 151.843f,  /* centimeter/s (wheel linear velocity) */ 
  .wheel_radius = 2.9f,                   /* centimeter */ 
  .radius_center_to_wheel = 9.2f,         /* centimeter */ 
  .control_interval = 1000                /* Loop interval in microseconds (1 ms, 1khz) */ 
};

inline MovementControl movement(movementConfig);

/* Robot Controller */
inline RobotController robotController(&movement);

/* Main Functions */ 
void setup();
void loop();

/* Helper Functions */
inline void gripLowerGripper() { servoGripLower.write(0); };
inline void gripUpperGripper() { servoGripUpper.write(0); };
inline void releaseLowerGripper() { servoGripLower.write(180); };
inline void releaseUpperGripper() { servoGripUpper.write(130); };

inline void startLifters() { servoLift1.write(180); servoLift2.write(0);  servoLift3.write(0);  servoLift4.write(180); };
inline void stopLifters() { servoLift1.write(90);  servoLift2.write(90); servoLift3.write(90); servoLift4.write(90); };

inline void setNeoPixelColor(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
  pixels.setPixelColor(index, pixels.Color(r, g, b));
}

inline void showNeoPixel() {
  pixels.show();
}

inline void setAllNeoPixels(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < neoPixelCount; i++) {
    pixels.setPixelColor(i, pixels.Color(r, g, b));
  }
  
  pixels.show();
}

inline void clearNeoPixel() {
  pixels.clear();
  pixels.show();
}

inline void printTof(const char* name, VL6180_Sensor& tof) {
  Serial.print(name);
  Serial.print(": ");

  if (!tof.isRangeTimeout(120)) {
    Serial.print(tof.getRange());
  } else {
    Serial.print("Timeout.");
  }

  Serial.println();
}

inline void printTof() {
  printTof("F1", tofFront1);
  printTof("F2", tofFront2);
  printTof("R1", tofRight1);
  printTof("R2", tofRight2);
  printTof("B1", tofBack1);
  printTof("B2", tofBack2);
  printTof("L1", tofLeft1);
  printTof("L2", tofLeft2);
  printTof("G1", tofGrip1);
  printTof("G2", tofGrip2);
}

} /* namespace Robot */ 

#endif