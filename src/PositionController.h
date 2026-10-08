#ifndef POSITION_CONTROLLER_H
#define POSITION_CONTROLLER_H

#include <Arduino.h>
#include <math.h>

class PositionController {
public:
  enum class Mode {
    LINEAR,
    ANGULAR_DEG,
  };

  PositionController(Mode mode);

  void setGain(float kP, float kI, float kD = 0.0f);

  void setOutputVelocityMin(float outputVelocityMin);
  void setOutputVelocityMax(float outputVelocityMax);

  void setErrorIntegralMin(float errorIntegralMin);
  void setErrorIntegralMax(float errorIntegralMax);
  
  void resetErrorIntegral(float errorIntegral = 0.0f);
  void resetErrorIntegral(float outputVelocity, float setpointPosition, float measPosition, float setpointDerivative = 0.0f, float measDerivative = 0.0f);

  void updateOutputVelocity(float setpointPosition, float measPosition, float timestep, float setpointDerivative = 0.0f, float measDerivative = 0.0f);

  float getErrorIntegral();
  float getOutputVelocity();

private:
  Mode mode;

  float kP = 0.0f;
  float kI = 0.0f;
  float kD = 0.0f;

  float outputVelocityMin = NAN;
  float outputVelocityMax = NAN;

  float errorIntegralMin = NAN;
  float errorIntegralMax = NAN;

  float errorIntegral = 0.0f;
  float outputVelocity = 0.0f;
};

#endif