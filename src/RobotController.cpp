#include "RobotController.h"

/* RobotController Implementation */ 

RobotController::RobotController(MovementControl* movement)
  : movement(movement),
    positionXController(PositionController::Mode::LINEAR),
    positionYController(PositionController::Mode::LINEAR),
    positionHeadingController(PositionController::Mode::ANGULAR_DEG) {}

void RobotController::setModeX(AxisMode mode) { modeX = mode; }
void RobotController::setModeY(AxisMode mode) { modeY = mode; }
void RobotController::setModeHeading(AxisMode mode) { modeHeading = mode; }

void RobotController::setTargetPositionX(float odometryPositionX) { targetPositionX = odometryPositionX; }
void RobotController::setTargetPositionY(float odometryPositionY) { targetPositionY = odometryPositionY; }
void RobotController::setTargetPositionHeading(float imuHeading)  { targetPositionHeading = imuHeading - 360.0f * ceilf((imuHeading - 180.0f) / 360.0f); } /* (-180, 180] */ 

void RobotController::setTargetVelocityX(float velocityX, Frame velocityFrame) { targetVelocityX = velocityX; targetVelocityFrameX = velocityFrame; }
void RobotController::setTargetVelocityY(float velocityY, Frame velocityFrame) { targetVelocityY = velocityY; targetVelocityFrameY = velocityFrame; }
void RobotController::setTargetVelocityHeading(float velocityHeading) { targetVelocityHeading = velocityHeading; }

void RobotController::update() {
  if (!movement) { return; }

  uint32_t currentTime = micros();
  
  if (setupFlag) { updateTime = currentTime; setupFlag = false; return; }

  float updateTimeInterval = (float)(currentTime - updateTime) * 1e-6f;

  float currentHeadingDeg = movement->get_imu_heading_x();
  float currentHeadingRad = currentHeadingDeg * DEG_TO_RAD;

  float cosCurrentHeading = cosf(currentHeadingRad);
  float sinCurrentHeading = sinf(currentHeadingRad);

  float globalVelocityXTemp = 0.0f;
  float globalVelocityYTemp = 0.0f;

  float robotVelocityXTemp = 0.0f;
  float robotVelocityYTemp = 0.0f;
  float robotVelocityHeadingTemp = 0.0f;

  if (modeX == AxisMode::POSITION && !isnan(targetPositionX)) {
    positionXController.updateOutputVelocity(targetPositionX, movement->get_odometry_pos_x_current(), updateTimeInterval);
    globalVelocityXTemp = positionXController.getOutputVelocity();
  } else {
    positionXController.resetErrorIntegral(0.0f);
  }

  if (modeY == AxisMode::POSITION && !isnan(targetPositionY)) {
    positionYController.updateOutputVelocity(targetPositionY, movement->get_odometry_pos_y_current(), updateTimeInterval);
    globalVelocityYTemp = positionYController.getOutputVelocity();
  } else {
    positionYController.resetErrorIntegral(0.0f);
  }

  if (modeHeading == AxisMode::POSITION && !isnan(targetPositionHeading)) {
    positionHeadingController.updateOutputVelocity(targetPositionHeading, movement->get_imu_heading_x(), updateTimeInterval);
    robotVelocityHeadingTemp = positionHeadingController.getOutputVelocity();
  } else {
    positionHeadingController.resetErrorIntegral(0.0f);
  }

  if (modeHeading == AxisMode::VELOCITY) {
    robotVelocityHeadingTemp = targetVelocityHeading;
  }

  if (modeX == AxisMode::VELOCITY) {
    if (targetVelocityFrameX == Frame::GLOBAL) { globalVelocityXTemp += targetVelocityX; } 
    if (targetVelocityFrameX == Frame::ROBOT)  { robotVelocityXTemp  += targetVelocityX; }
  }

  if (modeY == AxisMode::VELOCITY) {
    if (targetVelocityFrameY == Frame::GLOBAL) { globalVelocityYTemp += targetVelocityY; } 
    if (targetVelocityFrameY == Frame::ROBOT)  { robotVelocityYTemp  += targetVelocityY; }
  }
  
  commandGlobalVelocityX = globalVelocityXTemp + (robotVelocityXTemp * cosCurrentHeading - robotVelocityYTemp * sinCurrentHeading);
  commandGlobalVelocityY = globalVelocityYTemp + (robotVelocityXTemp * sinCurrentHeading + robotVelocityYTemp * cosCurrentHeading);

  commandRobotVelocityX = robotVelocityXTemp + ( globalVelocityXTemp * cosCurrentHeading + globalVelocityYTemp * sinCurrentHeading);
  commandRobotVelocityY = robotVelocityYTemp + (-globalVelocityXTemp * sinCurrentHeading + globalVelocityYTemp * cosCurrentHeading);
  commandRobotVelocityHeading = robotVelocityHeadingTemp;

  updateTime = currentTime;
  movement->robot_movement_cmd(commandRobotVelocityX, commandRobotVelocityY, commandRobotVelocityHeading); 
}

float RobotController::getErrorTargetPositionX() {
  if (!movement || isnan(targetPositionX)) { return NAN; }
  return targetPositionX - movement->get_odometry_pos_x_current(); 
}

float RobotController::getErrorTargetPositionY() {
  if (!movement || isnan(targetPositionY)) { return NAN; }
  return targetPositionY - movement->get_odometry_pos_y_current(); 
}

float RobotController::getErrorTargetPositionHeading() {
  if (!movement || isnan(targetPositionHeading)) { return NAN; }

  float errorTargetPositionHeading = targetPositionHeading - movement->get_imu_heading_x(); 
  return errorTargetPositionHeading - 360.0f * ceilf((errorTargetPositionHeading - 180.0f) / 360.0f); /* (-180, 180] */
}

float RobotController::getAbsErrorTargetPositionX() { return fabsf(getErrorTargetPositionX()); }
float RobotController::getAbsErrorTargetPositionY() { return fabsf(getErrorTargetPositionY()); }
float RobotController::getAbsErrorTargetPositionHeading() { return fabsf(getErrorTargetPositionHeading()); }

float RobotController::getCommandVelocityX(Frame velocityFrame) { 
  if (velocityFrame == Frame::GLOBAL) { return commandGlobalVelocityX; }
  if (velocityFrame == Frame::ROBOT)  { return commandRobotVelocityX;  }

  return NAN;
}

float RobotController::getCommandVelocityY(Frame velocityFrame) { 
  if (velocityFrame == Frame::GLOBAL) { return commandGlobalVelocityY; }
  if (velocityFrame == Frame::ROBOT)  { return commandRobotVelocityY;  }

  return NAN;
}

float RobotController::getCommandVelocityHeading() { return commandRobotVelocityHeading; }
