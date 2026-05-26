#pragma once

#include <Arduino.h>

#include "Config.h"
#include "Utils/OperationResult.h"

namespace AdcRegister {

constexpr uint8_t kRead = 1u << 6;
constexpr uint8_t kWrite = 0u;
constexpr uint8_t kIo = 0x1;
constexpr uint8_t kRevision = 0x2;
constexpr uint8_t kAdcStatus = 0x4;
constexpr uint8_t kAdcZeroScaleCal = 0x6;

constexpr uint8_t channelData(int ch) {
  return static_cast<uint8_t>(0x08 + ch);
}
constexpr uint8_t channelZeroScaleCal(int ch) {
  return static_cast<uint8_t>(0x10 + ch);
}
constexpr uint8_t channelFullScaleCal(int ch) {
  return static_cast<uint8_t>(0x18 + ch);
}
constexpr uint8_t channelSetup(int ch) {
  return static_cast<uint8_t>(0x28 + ch);
}
constexpr uint8_t channelConversionTime(int ch) {
  return static_cast<uint8_t>(0x30 + ch);
}
constexpr uint8_t mode(int ch) {
  return static_cast<uint8_t>(0x38 + ch);
}

constexpr uint8_t kMode24 = 1u << 1;
constexpr uint8_t kBitMode = kMode24;
constexpr uint8_t kIdleMode = (0u << 5) | kBitMode;
constexpr uint8_t kContinuousConversionMode = (1u << 5) | kBitMode;
constexpr uint8_t kSingleConversionMode = (2u << 5) | kBitMode;
constexpr uint8_t kZeroScaleSelfCalMode = (4u << 5) | kBitMode;
constexpr uint8_t kChannelZeroScaleSystemCalMode = (6u << 5) | kBitMode;
constexpr uint8_t kChannelFullScaleSystemCalMode = (7u << 5) | kBitMode;
constexpr uint8_t kEnableContinuousConversion = 1u << 3;

constexpr double kAdcResolution24 = 16777215.0;
constexpr double kFullScaleRange = 20.0;

inline double toDouble(uint32_t value) {
  return kFullScaleRange *
         (static_cast<double>(value) - (kAdcResolution24 / 2.0)) /
         kAdcResolution24;
}

}  // namespace AdcRegister

namespace ADCController {

void setup();
void initialize();
void resetToPreviousConversionTimes();
bool isChannelIndexValid(int channelIndex);

OperationResult readChannelVoltage(int channel_index);
float getVoltage(int channel_index);
double getVoltageData(int adc_channel);
double getVoltageDataNoTransaction(int adc_channel);

void startContinuousConversion(int adc_channel);
OperationResult idleMode(int adc_channel);
OperationResult setRDYFN(int adc_channel);
OperationResult unsetRDYFN(int adc_channel);

int getDataReadyPin(int board_index);
int getCsPin(int adc_channel);
bool buildConversionDataRead(int adc_channel, byte packet[4]);
double conversionDataPacketToVoltage(const byte packet[4]);

OperationResult setConversionTime(int adc_channel, float time_us);
float presetConversionTime(int adc_channel, int time_us,
                           bool isMoreThanOneChannelActive);
float getConversionTimeFloat(int adc_channel);
float getConversionTimeFloat(int adc_channel,
                             bool isMoreThanOneChannelActive);

OperationResult getChZeroScaleCalibration(int channel_index);
OperationResult getChFullScaleCalibration(int channel_index);
OperationResult applyChZeroScaleCalibration(int channel_index, uint32_t value);
OperationResult applyChFullScaleCalibration(int channel_index, uint32_t value);

OperationResult hardResetAllADCBoards();

}  // namespace ADCController
