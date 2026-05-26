#pragma once
#include <Arduino.h>

constexpr int NUM_CHANNELS_PER_DAC_BOARD = 4;
constexpr int NUM_CHANNELS_PER_ADC_BOARD = 4;
constexpr int NUM_DAC_BOARDS = 2;
constexpr int NUM_ADC_BOARDS = 2;
constexpr int NUM_DAC_CHANNELS = NUM_DAC_BOARDS * NUM_CHANNELS_PER_DAC_BOARD;
constexpr int NUM_ADC_CHANNELS = NUM_ADC_BOARDS * NUM_CHANNELS_PER_ADC_BOARD;
constexpr int NUM_DAC_CALIBRATION_CHANNELS = NUM_DAC_CHANNELS;
constexpr int NUM_ADC_CALIBRATION_CHANNELS = NUM_ADC_CHANNELS;
constexpr int adc_cs_pins[NUM_ADC_BOARDS] = {39, 40};
constexpr int dac_cs_pins[NUM_DAC_CHANNELS] = {23, 24, 25, 26, 27, 28, 29, 30};
constexpr int ldac = 22;
constexpr int reset[NUM_ADC_BOARDS] = {43, 44};
constexpr int drdy[NUM_ADC_BOARDS] = {47, 48};

constexpr int GPIO_0 = 52;
constexpr int GPIO_1 = 53;
constexpr int GPIO_2 = 5;
constexpr int GPIO_3 = 4;

constexpr int adc_sync = 51;

constexpr int led = 7;
constexpr int data_pin = 6;
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
