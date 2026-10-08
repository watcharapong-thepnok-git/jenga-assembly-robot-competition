#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <Arduino.h>
#include <math.h>
#include "MovementControl.h"
#include "PositionController.h"

class RobotController {
public:
  enum class AxisMode {
    DISABLED,
    POSITION,
    VELOCITY,
  };

  enum class Frame {
    GLOBAL,
    ROBOT,
  };

  RobotController(MovementControl* movement);

  void setModeX(AxisMode mode);
  void setModeY(AxisMode mode);
  void setModeHeading(AxisMode mode);

  void setTargetPositionX(float odometryPositionX);
  void setTargetPositionY(float odometryPositionY);
  void setTargetPositionHeading(float imuHeading);

  void setTargetVelocityX(float velocityX, Frame velocityFrame = Frame::GLOBAL);
  void setTargetVelocityY(float velocityY, Frame velocityFrame = Frame::GLOBAL);
  void setTargetVelocityHeading(float velocityHeading);
  
  void update();

  PositionController& getPositionXController() { return positionXController; } /* Derivative term does not applied */
  PositionController& getPositionYController() { return positionYController; } /* Derivative term does not applied */
  PositionController& getPositionHeadingController() { return positionHeadingController; }

  float getErrorTargetPositionX();
  float getErrorTargetPositionY();
  float getErrorTargetPositionHeading();

  float getAbsErrorTargetPositionX(); 
  float getAbsErrorTargetPositionY(); 
  float getAbsErrorTargetPositionHeading(); 

  float getCommandVelocityX(Frame velocityFrame = Frame::GLOBAL);
  float getCommandVelocityY(Frame velocityFrame = Frame::GLOBAL);
  float getCommandVelocityHeading();

private:
  MovementControl* movement;

  PositionController positionXController;
  PositionController positionYController;
  PositionController positionHeadingController;

  AxisMode modeX = AxisMode::DISABLED;
  AxisMode modeY = AxisMode::DISABLED;
  AxisMode modeHeading = AxisMode::DISABLED;

  float targetPositionX = NAN;
  float targetPositionY = NAN;
  float targetPositionHeading = NAN;

  Frame targetVelocityFrameX = Frame::GLOBAL;
  Frame targetVelocityFrameY = Frame::GLOBAL;

  float targetVelocityX = 0.0f;
  float targetVelocityY = 0.0f;
  float targetVelocityHeading = 0.0f;

  float commandGlobalVelocityX = 0.0f;
  float commandGlobalVelocityY = 0.0f;

  float commandRobotVelocityX = 0.0f;
  float commandRobotVelocityY = 0.0f;
  float commandRobotVelocityHeading = 0.0f;

  bool setupFlag = true;
  uint32_t updateTime = 0;
};

#endif