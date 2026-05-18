#include "Peripherals/DAC/DACChannel.h"

#include <cmath>

#include "Utils/FastGpio.h"

namespace {
constexpr byte kWriteAndUpdateDacCommand = 0x10;
constexpr byte kReadbackCommand = 0x90;
constexpr byte kWriteControlRegisterCommand = 0x20;
constexpr byte kUnclampDacFromGround = 0x02;
constexpr int kPositiveFullScaleCode = 524287;
constexpr int kNegativeFullScaleCode = 524288;
constexpr int kTwosComplementSpan = 1048576;
constexpr int kDataByteMask = 0xFF;
constexpr int kDataHighNibbleMask = 0x0F;
}  // namespace

DACChannel::DACChannel(int cs_pin, int channel_index)
    : gain_error(1.0),
      gain_error_inverse(1.0),
      offset_error(0.0),
      cs_pin(cs_pin),
      channel_index(channel_index),
      voltage_upper_bound(10.0),
      voltage_lower_bound(-10.0),
      commsController(cs_pin) {}

void DACChannel::setChannelIndex(int index) {
  this->channel_index = index;
}

int DACChannel::getChannelIndex() const {
  return channel_index;
}

int DACChannel::getCsPin() const {
  return cs_pin;
}

void DACChannel::initialize() {
  byte bytesToSend[3] = {kWriteControlRegisterCommand, 0,
                         kUnclampDacFromGround};
  commsController.transferDAC(bytesToSend, 3);
  setVoltage(0.0);
}

void DACChannel::setup() {
  pinMode(cs_pin, OUTPUT);
  FastGpio::digitalWrite(cs_pin, true);
}

float DACChannel::setVoltage(float v) {
  byte packet[3];
  if (!encodeVoltagePacket(v, packet) || !writeAndLatchPacket(packet)) {
    return NAN;
  }

  return gain_error *
         (threeByteToVoltage(packet[0], packet[1], packet[2]) + offset_error);
}

bool DACChannel::setVoltageNoTransactionNoLdac(float v) {
  byte bytesToSend[3];
  if (!encodeVoltagePacket(v, bytesToSend)) {
    return false;
  }

  return writeVoltagePacketNoLdac(bytesToSend);
}

bool DACChannel::encodeVoltagePacket(float v, byte packet[3]) {
  if (v > DACLimits::upper_voltage_limit[channel_index] ||
      v < DACLimits::lower_voltage_limit[channel_index]) {
    return false;
  }

  voltageToDecimal(v * gain_error_inverse - offset_error, &packet[0],
                   &packet[1], &packet[2]);
  return true;
}

bool DACChannel::writeVoltagePacketNoLdac(const byte packet[3]) {
  byte bytesToSend[3] = {packet[0], packet[1], packet[2]};
  return commsController.transferDACNoTransaction(bytesToSend, 3);
}

bool DACChannel::writeAndLatchPacket(const byte packet[3]) {
  byte bytesToSend[3] = {packet[0], packet[1], packet[2]};
  if (!commsController.transferDAC(bytesToSend, 3)) {
    return false;
  }
  FastGpio::pulseLowHigh(ldac);
  return true;
}

void DACChannel::setCalibration(float offset, float gain) {
  if (!std::isfinite(static_cast<double>(offset))) {
    offset = 0.0f;
  }
  if (!std::isfinite(static_cast<double>(gain)) ||
      std::fabs(static_cast<double>(gain)) < 1e-6) {
    gain = 1.0f;
  }
  this->offset_error = offset;
  this->gain_error = gain;
  this->gain_error_inverse = 1.0 / gain;
  voltage_upper_bound = full_scale * gain_error + offset_error;
  voltage_lower_bound = -full_scale * gain_error + offset_error;
}

void DACChannel::setFullScale(float full_scale) {
  this->full_scale = full_scale;
  voltage_upper_bound = full_scale * gain_error + offset_error;
  voltage_lower_bound = -full_scale * gain_error + offset_error;
}

float DACChannel::getHardwareLowerBound() {
  return voltage_lower_bound;
}

float DACChannel::getHardwareUpperBound() {
  return voltage_upper_bound;
}

float DACChannel::getOffsetError() {
  return offset_error;
}

float DACChannel::getGainError() {
  return gain_error;
}

float DACChannel::sendCode(int decimal) {
  byte packet[3];
  intToThreeBytes(decimal, &packet[0], &packet[1], &packet[2]);

  if (!writeAndLatchPacket(packet)) {
    return NAN;
  }

  return gain_error *
         (threeByteToVoltage(packet[0], packet[1], packet[2]) + offset_error);
}

float DACChannel::getVoltage() {
  byte bytesToSend[3] = {kReadbackCommand, 0, 0};
  byte data[3] = {0, 0, 0};
  if (!commsController.transferDACRead(bytesToSend, 3) ||
      !commsController.transferDACRead(data, 3)) {
    return NAN;
  }

  const float voltage = threeByteToVoltage(data[0], data[1], data[2]);
  return gain_error * (voltage + offset_error);
}

void DACChannel::voltageToDecimal(float v, byte* DB1, byte* DB2, byte* DB3) {
  int decimal;
  if (v >= 0) {
    decimal = v * kPositiveFullScaleCode / full_scale;
  } else {
    decimal =
        v * kNegativeFullScaleCode / full_scale + kTwosComplementSpan;
  }
  intToThreeBytes(decimal, DB1, DB2, DB3);
}

void DACChannel::intToThreeBytes(int decimal, byte* DB1, byte* DB2,
                                 byte* DB3) {
  *DB1 = static_cast<byte>((decimal >> 16) | kWriteAndUpdateDacCommand);
  *DB2 = static_cast<byte>((decimal >> 8) & kDataByteMask);
  *DB3 = static_cast<byte>(decimal & kDataByteMask);
}

int DACChannel::threeByteToInt(byte DB1, byte DB2, byte DB3) {
  return (((DB1 & kDataHighNibbleMask) << 8 | DB2) << 8) | DB3;
}

float DACChannel::threeByteToVoltage(byte DB1, byte DB2, byte DB3) {
  const int decimal = threeByteToInt(DB1, DB2, DB3);
  if (decimal <= kPositiveFullScaleCode) {
    return decimal * full_scale / kPositiveFullScaleCode;
  }
  return -(kTwosComplementSpan - decimal) * full_scale /
         kNegativeFullScaleCode;
}
