#include "Mission2.h"

namespace Robot {

Mission2::Mission2() : currentState(&Mission2::stateStartZone) {}

void Mission2::transitionTo(StateFunc nextState) {
  currentState = nextState;  
  step.reset(); cycle.reset(); clearNeoPixel();
}

void Mission2::update() {
  if (currentState) {
    (this->*currentState)();
  }
}

/* State Functions */
using AxisMode = RobotController::AxisMode;

void Mission2::stateRotationTest() {
  if (step.equal(0)) {
    movement.set_odometry_pos_x_current(0.0f);
    movement.set_odometry_pos_y_current(0.0f);

    robotController.setModeX(AxisMode::POSITION);
    robotController.setModeY(AxisMode::POSITION);
    robotController.setModeHeading(AxisMode::POSITION);

    step.set(2);
  
  return;
  }

  if (step.equal(1)) {
    if (buttonRed.pressed()) {
      robotController.setTargetPositionX(0.0f);
      robotController.setTargetPositionY(0.0f);
      robotController.setTargetPositionHeading(0.0f);

      step.next();
    }

  return;
  }

  if (step.equal(2)) {
    if (buttonGreen.pressed()) {
      step.next(); clock.tick();
    }

  return;
  }

  if (step.equal(3)) {
    if (clock.hasElapsed(250)) {
      robotController.setTargetPositionX(0.0f);
      robotController.setTargetPositionY(0.0f);
      robotController.setTargetPositionHeading(-90.0f);

      step.set(1);
    }
    
  return;
  }
}

void Mission2::stateTranslationTest() {
  if (step.equal(0)) {
    movement.set_odometry_pos_x_current(0.0f);
    movement.set_odometry_pos_y_current(0.0f);

    robotController.setModeX(AxisMode::VELOCITY);
    robotController.setModeY(AxisMode::VELOCITY);
    robotController.setModeHeading(AxisMode::POSITION);

    step.set(2);
  
  return;
  }

  if (step.equal(1)) {
    if (buttonRed.pressed()) {
      robotController.setTargetVelocityX(0.0f);
      robotController.setTargetVelocityY(0.0f);
      robotController.setTargetPositionHeading(0.0f);

      step.next();
    }

  return;
  }

  if (step.equal(2)) {
    if (buttonGreen.pressed()) {
      step.next(); clock.tick();
    }

  return;
  }

  if (step.equal(3)) {
    if (clock.hasElapsed(250)) {
      robotController.setTargetVelocityX(-18.0f);
      robotController.setTargetVelocityY(0.0f);
      robotController.setTargetPositionHeading(0.0f);

      step.set(1);
    }
    
  return;
  }
}

void Mission2::stateStartZone() {
  if (step.equal(0)) {
    setAllNeoPixels(255, 0, 0); 

    movement.set_odometry_pos_x_current(12.50f);
    movement.set_odometry_pos_y_current(-12.50f);

    robotController.setModeX(AxisMode::POSITION);
    robotController.setModeY(AxisMode::POSITION);
    robotController.setModeHeading(AxisMode::POSITION);

    step.next();
  
  return;
  }

  if (step.equal(1)) {
    if (buttonGreen.pressed()) {
      setAllNeoPixels(0, 255, 0); 
      step.next(); clock.tick();
    }

  return;
  }

  if (step.equal(2)) {
    if (clock.hasElapsed(250)) {
      transitionTo(&Mission2::stateStartToLoadZone); return;
    }
    
  return;
  }
}

void Mission2::stateStartToLoadZone() {
  if (step.equal(0)) {
    releaseUpperGripper();

    robotController.getPositionXController().setOutputVelocityMin(40.0f);
    robotController.getPositionYController().setOutputVelocityMax(-45.0f);

    robotController.setModeX(AxisMode::POSITION);
    robotController.setModeY(AxisMode::POSITION);
    robotController.setModeHeading(AxisMode::POSITION);

    robotController.setTargetPositionX(24.9f);
    robotController.setTargetPositionY(-104.9f);
    robotController.setTargetPositionHeading(0.0f);

    step.next(); clock.tick();

  return;
  }
  
  if (step.equal(1)) {
    setAllNeoPixels(255, 255, 255);

    bool foundbackWall = (!tofBack1.isRangeTimeout(120) && tofBack1.getRange() <= 40) || 
                         (!tofBack2.isRangeTimeout(120) && tofBack2.getRange() <= 40);

    if (foundbackWall) {
      setAllNeoPixels(0, 0, 255);
      gripUpperGripper();

      robotController.getPositionXController().setOutputVelocityMin(45.0f);
      robotController.getPositionYController().setOutputVelocityMax(NAN);

      robotController.setTargetPositionX(200.0f);
      robotController.setTargetPositionY(movement.get_odometry_pos_y_current());
      
      step.next(); clock.tick();

    } else if (clock.hasElapsed(5000)) { /* Fallback */
      setAllNeoPixels(255, 0, 0);
    }

  return;
  }

  if (step.equal(2)) {
    bool foundbackWall = (!tofBack1.isRangeTimeout(120) && tofBack1.getRange() <= 40) || 
                         (!tofBack2.isRangeTimeout(120) && tofBack2.getRange() <= 40);

    bool foundRightWall = (!tofRight1.isRangeTimeout(120) && tofRight1.getRange() <= 20) || 
                          (!tofRight2.isRangeTimeout(120) && tofRight2.getRange() <= 20);

    if (foundbackWall) {
      robotController.setTargetPositionY(movement.get_odometry_pos_y_current());
    }

    if (foundRightWall) {
      setAllNeoPixels(0, 255, 0);
      robotController.getPositionXController().setOutputVelocityMin(NAN);
      
      step.next(); clock.tick();

    } else if (clock.hasElapsed(7000)) { /* Fallback */
      setAllNeoPixels(255, 0, 0);
    }
    
  return;
  }

  if (step.equal(3)) {
    if (clock.hasElapsed(50)) {
      robotController.getPositionXController().setOutputVelocityMin(NAN);
      robotController.getPositionYController().setOutputVelocityMax(NAN);

      robotController.setModeX(AxisMode::VELOCITY);
      robotController.setModeY(AxisMode::VELOCITY);

      robotController.setTargetVelocityX(0.0f);
      robotController.setTargetVelocityY(0.0f);

      transitionTo(&Mission2::stateLoadZone); return;
    }
    
  return;
  }
}

void Mission2::stateLoadZone() {
  static const uint32_t loadingDuration[4] = { 1400, 1500, 1500, 1500 };
  static const float loadingVelocity[4] = { 13.0f, 12.0f, 12.0f, 12.0f };

  static bool foundJengaFlag = false;
  static bool grippedFlag = false;

  if (step.equal(0)) {
    foundJengaFlag = false;
    grippedFlag = false;

    setAllNeoPixels(255, 255, 255);
    startLifters();

    robotController.setModeX(AxisMode::VELOCITY);
    robotController.setModeY(AxisMode::VELOCITY);
    robotController.setModeHeading(AxisMode::POSITION);
    
    robotController.setTargetVelocityX(0.0f);
    robotController.setTargetVelocityY(loadingVelocity[cycle.current()]);
    robotController.setTargetPositionHeading(-5.0f);

    step.next(); clock.tick();

  return;
  }

  if (step.equal(1)) {
    bool foundJenga = (!tofGrip1.isRangeTimeout(120) && tofGrip1.getRange() <= 60);

    if (foundJenga && !foundJengaFlag) {
      foundJengaFlag = true;
      clock.tick();
    }

    if (foundJengaFlag && !grippedFlag && clock.hasElapsed(100)) {
      grippedFlag = true;
      gripLowerGripper();
      
      clock2.tick();
    }

    if (cycle.equal(3)) { /* Fourth Jenga */
      if (grippedFlag && clock2.hasElapsed(250)) {
        robotController.setTargetVelocityX(0.0f);
        robotController.setTargetVelocityY(0.0f);

        step.next();
      }

    } else if (grippedFlag && clock2.hasElapsed(700)) { /* First, Second, Third Jenga */
      robotController.setTargetVelocityX(0.0f);
      robotController.setTargetVelocityY(0.0f);

      step.next();

    } else if (!foundJengaFlag && clock.hasElapsed(5000)) { /* Fallback */
      setAllNeoPixels(255, 0, 0);
    }
    
  return;
  }

  if (step.equal(2)) {
    if (cycle.equal(3)) { /* Fourth Jenga */
      if (clock2.hasElapsed(400)) {
        setAllNeoPixels(0, 0, 255);
        stopLifters(); 

        transitionTo(&Mission2::stateLoadToAssemblyZone); return;
      }

    } else if (clock2.hasElapsed(900)) { /* First, Second, Third Jenga */
      releaseLowerGripper();
      step.next(); clock2.tick();
    }

  return;
  }

  if (step.equal(3)) {
    if (clock2.hasElapsed(250)) {
      step.set(0); cycle.next();
    }

  return;
  }
}

void Mission2::stateLoadToAssemblyZone() {
  if (step.equal(0)) {
    setAllNeoPixels(0, 255, 255);

    robotController.setModeX(AxisMode::VELOCITY);
    robotController.setModeY(AxisMode::VELOCITY);
    robotController.setModeHeading(AxisMode::POSITION);
    
    robotController.setTargetVelocityX(-12.0f);
    robotController.setTargetVelocityY(20.0f);
    robotController.setTargetPositionHeading(-90.0f);

    step.next();

  return;
  }

  if (step.equal(1)) {
    if (robotController.getAbsErrorTargetPositionHeading() < 20.0f) {
      robotController.setTargetVelocityX(0.0f);
      robotController.setTargetVelocityY(40.0f);

      step.next(); clock.tick();
    }
    
  return;
  }

  if (step.equal(2)) {
    bool foundLeftWall = (!tofLeft1.isRangeTimeout(120) && tofLeft1.getRange() <= 40) || 
                         (!tofLeft2.isRangeTimeout(120) && tofLeft2.getRange() <= 40);

    if (foundLeftWall) {
      setAllNeoPixels(0, 255, 255);

      robotController.setModeX(AxisMode::POSITION);
      robotController.setModeY(AxisMode::POSITION);
      
      robotController.setTargetPositionX(movement.get_odometry_pos_x_current() + 60.0f);
      robotController.setTargetPositionY(movement.get_odometry_pos_y_current());
        
      step.next();

    } else if (clock.hasElapsed(2000)) {
      setAllNeoPixels(255, 0, 0);
    }

  return;
  }

  if (step.equal(3)) {
    if (robotController.getAbsErrorTargetPositionX() < 10.0f) {
      robotController.setModeX(AxisMode::VELOCITY);

      robotController.setTargetVelocityX(0.0f);
      robotController.setTargetPositionY(movement.get_odometry_pos_y_current() - 110.0f);
      robotController.setTargetPositionHeading(-180.0f);

      step.next();
    }

  return;
  }

  if (step.equal(4)) {
    if (robotController.getAbsErrorTargetPositionY() < 7.0f) {
      robotController.setModeX(AxisMode::VELOCITY);
      robotController.setModeY(AxisMode::VELOCITY);

      robotController.setTargetVelocityX(0.0f);
      robotController.setTargetVelocityY(0.0f);

      step.next(); clock.tick();
    }
  
  return;
  }

  if (step.equal(5)) {
    if (clock.hasElapsed(250)) {
      robotController.setTargetPositionHeading(movement.get_imu_heading_x());
      transitionTo(&Mission2::stateAssemblyZone); return;
    }

  return;
  }
}

void Mission2::stateAssemblyZone() {
  static int servoPosition = 0;

  if (step.equal(0)) {
    servoPosition = 0;

    setAllNeoPixels(255, 255, 255); 
    servoGripLower.write(60);
    servoGripUpper.write(60);

    step.next(); clock.tick();

  return;
  }

  if (step.equal(1)) {
    if (clock.hasElapsed(600)) { 
      servoGripLower.write(0); 
      servoGripUpper.write(0);

      step.next(); clock.tick();
    }

  return;
  }

  if (step.equal(2)) {
    if (clock.hasElapsed(1000)) {
      servoGripLower.write(130);
      servoGripUpper.write(130);
      
      servoPosition++;
    }

    if (servoPosition >= 130) {
      step.next(); clock.tick();
    }

  return;
  }

  if (step.equal(3)) {
    if (clock.hasElapsed(1000)) {
      robotController.setModeX(AxisMode::VELOCITY);
      robotController.setModeY(AxisMode::VELOCITY);

      robotController.setTargetVelocityX(0.0f);
      robotController.setTargetVelocityY(12.0f);

      step.next(); clock.tick();
    }

  return;
  }

  if (step.equal(4)) {
    if (clock.hasElapsed(1200)) { 
      setAllNeoPixels(255, 255, 0); 

      robotController.setModeX(AxisMode::DISABLED);
      robotController.setModeY(AxisMode::DISABLED);

      step.next();
    }

  return;
  }

  /* End mission */
}

} /* namespace Robot */