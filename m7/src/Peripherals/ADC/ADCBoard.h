#pragma once

#include <Arduino.h>
#include <Peripherals/PeripheralCommsController.h>

#include <vector>

#include "Utils/shared_memory.h"

#include "Config.h"

namespace AdcRegister {

constexpr uint8_t kRead = 1u << 6;
constexpr uint8_t kWrite = 0u;

constexpr uint8_t kCom = 0x0;
constexpr uint8_t kIo = 0x1;
constexpr uint8_t kRevision = 0x2;
constexpr uint8_t kTest = 0x3;
constexpr uint8_t kAdcStatus = 0x4;
constexpr uint8_t kChecksum = 0x5;
constexpr uint8_t kAdcZeroScaleCal = 0x6;
constexpr uint8_t kAdcFullScale = 0x7;

constexpr uint8_t kDumpMode = 1u << 3;

constexpr uint8_t channelData(int adcChannel) {
  return static_cast<uint8_t>(0x08 + adcChannel);
}

constexpr uint8_t channelZeroScaleCal(int adcChannel) {
  return static_cast<uint8_t>(0x10 + adcChannel);
}

constexpr uint8_t channelFullScaleCal(int adcChannel) {
  return static_cast<uint8_t>(0x18 + adcChannel);
}

constexpr uint8_t channelStatus(int adcChannel) {
  return static_cast<uint8_t>(0x20 + adcChannel);
}

constexpr uint8_t channelSetup(int adcChannel) {
  return static_cast<uint8_t>(0x28 + adcChannel);
}

constexpr uint8_t channelConversionTime(int adcChannel) {
  return static_cast<uint8_t>(0x30 + adcChannel);
}

constexpr uint8_t mode(int adcChannel) {
  return static_cast<uint8_t>(0x38 + adcChannel);
}

constexpr uint8_t kMode16 = 0u;
constexpr uint8_t kMode24 = 1u << 1;
constexpr uint8_t kBitMode = kMode24;

constexpr uint8_t kIdleMode = (0u << 5) | kBitMode;
constexpr uint8_t kContinuousConversionMode = (1u << 5) | kBitMode;
constexpr uint8_t kSingleConversionMode = (2u << 5) | kBitMode;
constexpr uint8_t kPowerDownMode = (3u << 5) | kBitMode;
constexpr uint8_t kZeroScaleSelfCalMode = (4u << 5) | kBitMode;
constexpr uint8_t kChannelZeroScaleSystemCalMode = (6u << 5) | kBitMode;
constexpr uint8_t kChannelFullScaleSystemCalMode = (7u << 5) | kBitMode;
constexpr uint8_t kEnableContinuousConversion = 1u << 3;

constexpr double kAdcResolution16 = 65535.0;
constexpr double kAdcResolution24 = 16777215.0;
constexpr double kFullScaleRange = 20.0;

inline double toDouble(uint32_t value) {
  return kFullScaleRange *
         (static_cast<double>(value) - (kAdcResolution24 / 2.0)) /
         kAdcResolution24;
}

}  // namespace AdcRegister

class ADCBoard {
 public:
  bool chopEnabled = true;
 private:
  int cs_pin;
  int data_ready_pin;
  int reset_pin;
  int board_idx;
  PeripheralCommsController commsController;

  void waitDataReady();
  uint8_t readRegister8(uint8_t address);
  uint32_t readRegister24(uint8_t address, bool noTransaction = false);
  void writeRegister8(uint8_t address, uint8_t value);
  void writeRegister24(uint8_t address, uint32_t value);

 public:
  bool data_ready = false;

  ADCBoard(int cs_pin, int data_ready_pin, int reset_pin, int board_idx);

  void setup();

  void RDY_ISR();

  void initialize();

  uint32_t getZeroScaleCalibration(int adc_channel);

  uint32_t getFullScaleCalibration(int adc_channel);

  void setZeroScaleCalibration(int adc_channel, uint32_t value);

  void setFullScaleCalibration(int adc_channel, uint32_t value);

  void resetToPreviousConversionTimes();

  int getDataReadyPin() const;
  int getBoardIndex() const;
  int getCsPin() const;

  void setReadyFlag();
  void clearReadyFlag();

  double readVoltage(int channel_index);

  // return ADC status register, pg. 16
  uint8_t getADCStatus();

  void setRDYFN();

  void unsetRDYFN();

  void channelSetup(int adc_channel, uint8_t flags);

  // tells the ADC to start a single conversion on the passed channel
  void startSingleConversion(int adc_channel);

  // tells the ADC to start a continous conversion on the passed channel
  void startContinuousConversion(int adc_channel);

  uint8_t getRevisionRegister();

  void setConversionTime(int adc_channel, int chop, int fw);

  uint32_t getConversionData(int adc_channel);

  uint32_t getConversionDataNoTransaction(int adc_channel);

  std::vector<double> continuousConvert(int channel_index, uint32_t period_us,
                                        uint32_t duration);

  void idleMode(int adc_channel);

  bool isChannelActive(int adc_channel);

  void hardReset();

  void restoreCalibrationFromFlash();

  void reset();

  uint8_t talkADC(byte command);

  float setConversionTime(int channel, float time_us);

  float setConversionTimeFloat(int channel, float time_us,
                               bool moreThanOneChannelActive);

  float setConversionTime(int channel, bool chop, byte fw,
                          bool moreThanOneChannelActive);

  float setConversionTimeFW(int channel, int filter_word);

  float getConversionTime(int channel);

  float getConversionTime(int channel, bool moreThanOneChannelActive);

  float calculateConversionTime(byte b, bool moreThanOneChannelActive);

  byte calculateFilterWord(float time_us, bool chop,
                           bool moreThanOneChannelActive);

  bool isMoreThanOneChannelActive();

  void zeroScaleSelfCalibration();

  void zeroScaleChannelSystemSelfCalibration(int channel);

  void fullScaleChannelSystemSelfCalibration(int channel);
};
