#include "PositionController.h"

PositionController::PositionController(Mode mode)
  : mode(mode) {}

void PositionController::setGain(float kP, float kI, float kD) {
  if (kI != 0.0f) {
    errorIntegral = errorIntegral * (this->kI / fabsf(kI));
  }

  this->kP = fabsf(kP);
  this->kI = fabsf(kI);
  this->kD = fabsf(kD);
}

void PositionController::setOutputVelocityMin(float outputVelocityMin) { this->outputVelocityMin = outputVelocityMin; }
void PositionController::setOutputVelocityMax(float outputVelocityMax) { this->outputVelocityMax = outputVelocityMax; }

void PositionController::setErrorIntegralMin(float errorIntegralMin) { this->errorIntegralMin = errorIntegralMin; }
void PositionController::setErrorIntegralMax(float errorIntegralMax) { this->errorIntegralMax = errorIntegralMax; }

void PositionController::resetErrorIntegral(float errorIntegral) { this->errorIntegral = errorIntegral; }
void PositionController::resetErrorIntegral(float outputVelocity, float setpointPosition, float measPosition, float setpointDerivative, float measDerivative) {
  if (kI != 0.0f) {
    float errorTemp = setpointPosition - measPosition;

    if (mode == Mode::ANGULAR_DEG) {
      errorTemp = errorTemp - 360.0f * ceilf((errorTemp - 180.0f) / 360.0f);
    }

    float errorIntegralTemp = (outputVelocity - (kP * errorTemp) - (kD * (setpointDerivative - measDerivative))) / kI;
    
    if (!isnan(errorIntegralMin)) { errorIntegralTemp = fmaxf(errorIntegralTemp, errorIntegralMin); }
    if (!isnan(errorIntegralMax)) { errorIntegralTemp = fminf(errorIntegralTemp, errorIntegralMax); }
  
    errorIntegral = errorIntegralTemp;
  }
}

void PositionController::updateOutputVelocity(float setpointPosition, float measPosition, float timestep, float setpointDerivative, float measDerivative) {
  float errorTemp = setpointPosition - measPosition;
  float outputVelocityTemp = 0.0f;

  if (mode == Mode::ANGULAR_DEG) {
    errorTemp = errorTemp - 360.0f * ceilf((errorTemp - 180.0f) / 360.0f); /* (-180, 180] */
  }

  float errorIntegralTemp = errorIntegral + errorTemp * fmaxf(timestep, 0.0f);

  if (!isnan(errorIntegralMin)) {
    errorIntegralTemp = fmaxf(errorIntegralTemp, errorIntegralMin);
  }

  if (!isnan(errorIntegralMax)) {
    errorIntegralTemp = fminf(errorIntegralTemp, errorIntegralMax);
  }

  outputVelocityTemp = kP * errorTemp + kI * errorIntegralTemp + kD * (setpointDerivative - measDerivative);

  bool saturationMax = (!isnan(outputVelocityMax) && (outputVelocityTemp > outputVelocityMax));
  bool saturationMin = (!isnan(outputVelocityMin) && (outputVelocityTemp < outputVelocityMin));

  if (!(saturationMin && errorTemp < 0) && !(saturationMax && errorTemp > 0)) {
    errorIntegral = errorIntegralTemp;
  }

  outputVelocityTemp = kP * errorTemp + kI * errorIntegral + kD * (setpointDerivative - measDerivative);

  if (!isnan(outputVelocityMin)) {
    outputVelocityTemp = fmaxf(outputVelocityTemp, outputVelocityMin);
  }

  if (!isnan(outputVelocityMax)) {
    outputVelocityTemp = fminf(outputVelocityTemp, outputVelocityMax);
  }

  outputVelocity = outputVelocityTemp;
}

float PositionController::getErrorIntegral()  { return errorIntegral;  }
float PositionController::getOutputVelocity() { return outputVelocity; }
