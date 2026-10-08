#ifndef TCA9548A_SWITCH_H
#define TCA9548A_SWITCH_H

#include <Arduino.h>
#include <Wire.h>

class TCA9548A_Switch {
public:
  TCA9548A_Switch(TwoWire* bus, uint8_t address);

  void selectSingleChannel(uint8_t channel, bool checkCache);
  void deselectAllChannels(bool checkCache);

  void resetCache();
  
private:
  TwoWire* bus;

  uint8_t address;
  uint16_t channelBits = 0xFFFF;
};

#endif