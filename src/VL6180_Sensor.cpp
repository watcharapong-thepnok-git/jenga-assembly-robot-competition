#include "VL6180_Sensor.h"

VL6180_Sensor::VL6180_Sensor(TwoWire* bus, void (*i2cCallback)())
  : bus(bus), i2cCallback(i2cCallback) {}

void VL6180_Sensor::begin(uint8_t scaling) {
  if (i2cCallback != nullptr) {
    (*i2cCallback)();
  }

  sensor.setBus(bus);
  sensor.init();
  sensor.configureDefault();
  sensor.setScaling(scaling);

  sensor.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01 | 0x04);

  return;
}

void VL6180_Sensor::setScaling(uint8_t scaling) {
  requiredScaling = scaling;

  return;
}

void VL6180_Sensor::setAutoStarting(bool enable) {
  enableAutoStarting = enable;

  return;
}

uint16_t VL6180_Sensor::getRange() { return range; }
bool VL6180_Sensor::isRangeTimeout(uint32_t timeout) { return (millis() - lastMeasSuccessTime >= timeout) || (range == UINT16_MAX); }

VL6180_Sensor::Result VL6180_Sensor::update() {
  Result result = { .event = Event::NONE, .range = UINT16_MAX, .measPeriod = 0 };

  bool repeat;

  do {
    repeat = false;

    switch (nextTask) {
      case Task::IDLE:
        {
          if (!enableAutoStarting) {
            break;
          }

          nextTask = Task::STARTING;

          [[fallthrough]];
        }

      case Task::STARTING:
        {
          if (i2cCallback != nullptr) {
            (*i2cCallback)();
          }

          uint8_t resultRangeStatus = sensor.readReg(VL6180X::RESULT__RANGE_STATUS);

          if ((resultRangeStatus & 0x01) != 0x01) {
            if (millis() - clearIntTime >= clearIntDelay) {
              sensor.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01 | 0x04);
              clearIntTime = millis();
            }

            break;
          }

          if (sensor.getScaling() != requiredScaling) {
            sensor.setScaling(requiredScaling);
            break;
          }

          sensor.writeReg(VL6180X::SYSRANGE__START, 0x01);

          startPreparingTime = millis();
          startMeasTime = startPreparingTime;

          nextTask = Task::PREPARING;

          break;
        }

      case Task::PREPARING:
        {
          if (millis() - startPreparingTime < maxPreparingTime) {
            break;
          }

          startPollingTime = millis();
          nextTask = Task::POLLING;

          [[fallthrough]];
        }

      case Task::POLLING:
        {
          if (i2cCallback != nullptr) {
            (*i2cCallback)();
          }

          if (millis() - startPollingTime >= maxPollingTime) {
            sensor.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01 | 0x04);
            previousMeasSuccess = false;

            nextTask = Task::IDLE;

            break;
          }

          uint8_t resultIntStatus = sensor.readReg(VL6180X::RESULT__INTERRUPT_STATUS_GPIO);

          if ((resultIntStatus & 0x04) != 0x04) {
            startCooldownTime = millis();
            nextTask = Task::COOLDOWN;

            break;
          }

          uint8_t resultRangeStatus = sensor.readReg(VL6180X::RESULT__RANGE_STATUS);
          uint8_t errorCode = resultRangeStatus & 0xF0;

          // 0x00 = No Error, 0xC0 = Raw Algo Underflow, 0xE0 = Algo Underflow
          if (errorCode == 0x00 || errorCode == 0xC0 || errorCode == 0xE0) {
            // If no error, read actual register value; otherwise clamp underflow to 0
            result.range = (errorCode == 0x00) 
              ? (uint16_t)(sensor.getScaling()) * sensor.readReg(VL6180X::RESULT__RANGE_VAL) 
              : 0;

            result.event = Event::NEW_RANGE;

            this->range = result.range;
            this->lastMeasSuccessTime = millis();

            if (previousMeasSuccess) {
              result.measPeriod = startMeasTime - startMeasPreviousTime;
            }

            startMeasPreviousTime = startMeasTime;
            previousMeasSuccess = true;

          } else {
            // Other error conditions (overflow, hardware error, no target ignore, etc.)
            previousMeasSuccess = false;
          }

          sensor.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01 | 0x04);
          nextTask = Task::IDLE;

          break;
        }

      case Task::COOLDOWN:
        {
          if (i2cCallback != nullptr) {
            (*i2cCallback)();
          }

          if (millis() - startPollingTime >= maxPollingTime) {
            sensor.writeReg(VL6180X::SYSTEM__INTERRUPT_CLEAR, 0x01 | 0x04);
            previousMeasSuccess = false;

            nextTask = Task::IDLE;

            break;
          }

          if (millis() - startCooldownTime >= maxCooldownTime) {
            repeat = true;
            nextTask = Task::POLLING;

            break;
          }

          break;
        }
    }
  } while (repeat);

  return result;
}