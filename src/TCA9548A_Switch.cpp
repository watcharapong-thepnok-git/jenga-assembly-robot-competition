#include "TCA9548A_Switch.h"

TCA9548A_Switch::TCA9548A_Switch(TwoWire* bus, uint8_t address)
  : bus(bus), address(address) {}

void TCA9548A_Switch::selectSingleChannel(uint8_t channel, bool checkCache) {
  if (channel > 7) {
    return;
  }

  uint8_t channelMask = (1 << channel);

  if (checkCache && (channelBits == channelMask)) {
    return;
  }

  bus->beginTransmission(address);
  bus->write(channelMask);

  if (bus->endTransmission() != 0) {
    resetCache();

    return;
  }

  channelBits = channelMask;

  return;
}

void TCA9548A_Switch::deselectAllChannels(bool checkCache) {
  uint8_t channelMask = 0x00;

  if (checkCache && (channelBits == channelMask)) {
    return;
  }

  bus->beginTransmission(address);
  bus->write(channelMask);

  if (bus->endTransmission() != 0) {
    resetCache();
    
    return;
  }

  channelBits = channelMask;

  return;
}

void TCA9548A_Switch::resetCache() {
  channelBits = 0xFFFF;

  return;
}
