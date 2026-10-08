#include "Robot.h"
#include "Mission1.h"
#include "Mission2.h"

namespace Robot {
  Mission1 mission1; /* Blue Team */
  Mission2 mission2; /* Red Team */
} /* namespace Robot */

void Robot::setup() {
  /* I2C */ 
  Wire.begin();
  Wire.setClock(400000L);

  Wire2.begin();
  Wire2.setClock(400000L);

  /* Neopixel */
  pixels.begin();          
  pixels.setBrightness(50);
  clearNeoPixel();   

  /* Robot Buttons */ 
  buttonGreen.attach(buttonGreenPin, INPUT_PULLUP);
  buttonGreen.interval(25);
  buttonGreen.setPressedState(LOW); 

  buttonYellow.attach(buttonYellowPin, INPUT_PULLUP);
  buttonYellow.interval(25);
  buttonYellow.setPressedState(LOW); 

  buttonRed.attach(buttonRedPin, INPUT_PULLUP);
  buttonRed.interval(25);
  buttonRed.setPressedState(LOW); 

  /* ToF Sensors */
  tofFront1.begin(1); tofFront1.setAutoStarting(false); delay(50);
  tofFront2.begin(1); tofFront2.setAutoStarting(false); delay(50);

  tofRight1.begin(1); tofRight1.setAutoStarting(true); delay(50);
  tofRight2.begin(1); tofRight2.setAutoStarting(true); delay(50);

  tofBack1.begin(1); tofBack1.setAutoStarting(true); delay(50);
  tofBack2.begin(1); tofBack2.setAutoStarting(true); delay(50);

  tofLeft1.begin(1); tofLeft1.setAutoStarting(true); delay(50);
  tofLeft2.begin(1); tofLeft2.setAutoStarting(true); delay(50);

  tofGrip1.begin(1); tofGrip1.setAutoStarting(true); delay(50);
  tofGrip2.begin(1); tofGrip2.setAutoStarting(true); delay(50);

  /* Servo Motors */ 
  servoLift1.attach(36);
  servoLift2.attach(37);
  servoLift3.attach(13);
  servoLift4.attach(14);

  servoGripLower.attach(15);
  servoGripUpper.attach(33);

  stopLifters(); releaseLowerGripper(); gripUpperGripper();

  /* Movement System */ 
  movement.setup();
  
  attachInterrupt(digitalPinToInterrupt(movementConfig.encoder_pin_a_l), []() { movement.robot_encoder_l_isr(); }, RISING);
  attachInterrupt(digitalPinToInterrupt(movementConfig.encoder_pin_a_r), []() { movement.robot_encoder_r_isr(); }, RISING);
  attachInterrupt(digitalPinToInterrupt(movementConfig.encoder_pin_a_b), []() { movement.robot_encoder_b_isr(); }, RISING);

  /* Robot Controllers */
  robotController.setModeX(RobotController::AxisMode::DISABLED);
  robotController.setModeY(RobotController::AxisMode::DISABLED);
  robotController.setModeHeading(RobotController::AxisMode::DISABLED);

  robotController.getPositionXController().setGain(2.3f, 0.2f);
  robotController.getPositionXController().setOutputVelocityMin(-130.0f);
  robotController.getPositionXController().setOutputVelocityMax(+130.0f);

  robotController.getPositionYController().setGain(2.3f, 0.2f);
  robotController.getPositionYController().setOutputVelocityMin(-130.0f);
  robotController.getPositionYController().setOutputVelocityMax(+130.0f);

  robotController.getPositionHeadingController().setGain(17.0f, 0.0f); 
  robotController.getPositionHeadingController().setOutputVelocityMin(-240.0f);
  robotController.getPositionHeadingController().setOutputVelocityMax(+240.0f);

  setAllNeoPixels(255, 255, 255); delay(500); setAllNeoPixels(0, 0, 0); 
}


void Robot::loop() {
  /* Robot Buttons */ 
  buttonGreen.update(); buttonYellow.update(); buttonRed.update();

  /* Movement System */ 
  movement.update_heading();
  movement.update_encoder();

  /* Robot Controller */
  robotController.update();

  /* Movement System */ 
  movement.robot_movement_controller();

  /* ToF Sensors */ 
  tofFront1.update();
  tofFront2.update();

  tofRight1.update();
  tofRight2.update();

  tofBack1.update();
  tofBack2.update();

  tofLeft1.update();
  tofLeft2.update();
    
  tofGrip1.update();
  tofGrip2.update();

  /* Robot Mission */
  if (!missionBlueTeam && !missionRedTeam) {
    if (buttonYellow.pressed()) {
      missionBlueTeam = true;
    } else if (buttonRed.pressed()) {
      missionRedTeam = true;
    }

    printTof();
  }

  if (missionBlueTeam) { 
    mission1.update(); 
  } else if (missionRedTeam) {
    mission2.update();
  }
}
