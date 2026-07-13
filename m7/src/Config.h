#pragma once
#include <Arduino.h>

struct SequentialPins {
  int first;
  constexpr int operator[](int index) const { return first + index; }
};

using AdcBoardMask = uint32_t;

constexpr int NUM_CHANNELS_PER_DAC_BOARD = 4;
constexpr int NUM_CHANNELS_PER_ADC_BOARD = 4;
constexpr int NUM_DAC_BOARDS = 2;
constexpr int NUM_ADC_BOARDS = 2;
constexpr int NUM_DAC_CHANNELS = NUM_DAC_BOARDS * NUM_CHANNELS_PER_DAC_BOARD;
constexpr int NUM_ADC_CHANNELS = NUM_ADC_BOARDS * NUM_CHANNELS_PER_ADC_BOARD;
constexpr int NUM_DAC_CALIBRATION_CHANNELS = NUM_DAC_CHANNELS;
constexpr int NUM_ADC_CALIBRATION_CHANNELS = NUM_ADC_CHANNELS;
constexpr SequentialPins adc_cs_pins{39};
constexpr SequentialPins dac_cs_pins{23};
constexpr int ldac = 22;
constexpr SequentialPins reset{43};
constexpr SequentialPins drdy{47};
static_assert(NUM_DAC_BOARDS > 0 && NUM_ADC_BOARDS > 0,
              "At least one DAC and ADC board must be configured");
static_assert(NUM_ADC_BOARDS <= static_cast<int>(sizeof(AdcBoardMask) * 8),
              "AdcBoardMask cannot represent all configured ADC boards");

constexpr int GPIO_0 = 52;
constexpr int GPIO_1 = 53;
constexpr int GPIO_2 = 5;
constexpr int GPIO_3 = 4;

constexpr int adc_sync = 51;

constexpr int led = 7;
constexpr int err = 11;
constexpr uint32_t DAC_SPI_FREQUENCY_HZ = 18000000;
constexpr uint8_t DAC_SPI_MODE = 1;
constexpr uint8_t DAC_READ_SPI_MODE = 3;
constexpr uint32_t ADC_SPI_FREQUENCY_HZ = 5000000;
constexpr uint8_t ADC_SPI_MODE = 0;

namespace DACLimits {
extern float upper_voltage_limit[NUM_DAC_CHANNELS];
extern float lower_voltage_limit[NUM_DAC_CHANNELS];
extern bool limits_initialized;

inline void initializeLimits() {
  if (!limits_initialized) {
    for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
      upper_voltage_limit[i] = 10.0f;
      lower_voltage_limit[i] = -10.0f;
    }
    limits_initialized = true;
  }
}
}  // namespace DACLimits
