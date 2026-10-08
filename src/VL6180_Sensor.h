#ifndef VL6180_SENSOR_H
#define VL6180_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <VL6180X.h>

class VL6180_Sensor {
public:
  VL6180_Sensor(TwoWire* bus, void (*i2cCallback)() = nullptr);
  VL6180X sensor;

  enum class Task {
    IDLE,
    STARTING,
    PREPARING,
    POLLING,
    COOLDOWN,
  };

  enum class Event {
    NONE,
    NEW_RANGE,
  };

  struct Result {
    Event event;
    uint16_t range;
    unsigned long measPeriod;
  };

  void begin(uint8_t scaling);

  void setScaling(uint8_t scaling);
  void setAutoStarting(bool enable);

  uint16_t getRange();
  bool isRangeTimeout(uint32_t timeout);

  Result update();

private:
  Task nextTask = Task::IDLE;

  TwoWire* bus;
  void (*i2cCallback)();

  bool enableAutoStarting = false;
  uint8_t requiredScaling = 1;

  uint16_t range = UINT16_MAX;
  unsigned long lastMeasSuccessTime = 0;

  unsigned long clearIntTime = 0;
  unsigned long clearIntDelay = 50;
  
  unsigned long startPreparingTime;
  unsigned long startPollingTime;
  unsigned long startCooldownTime;

  bool previousMeasSuccess = false;
  unsigned long startMeasTime;
  unsigned long startMeasPreviousTime;

  const unsigned long maxPreparingTime = 7;
  const unsigned long maxPollingTime = 100;
  const unsigned long maxCooldownTime = 2;
};

#endif