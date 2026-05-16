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
const int adc_cs_pins[NUM_ADC_BOARDS] = {39, 40};
const int dac_cs_pins[NUM_DAC_CHANNELS] = {23, 24, 25, 26, 27, 28, 29, 30};
#define ldac 22
const int reset[NUM_ADC_BOARDS] = {43, 44};
const int drdy[NUM_ADC_BOARDS] = {47, 48};

#define GPIO_0 52
#define GPIO_1 53
#define GPIO_2 5
#define GPIO_3 4

#define adc_sync 51

#define led 7 // indicator LED
#define data_pin 6 // data indicator LED
#define err 11 // error indicator LED
constexpr uint32_t DAC_SPI_FREQUENCY_HZ = 20000000;
constexpr uint8_t DAC_SPI_MODE = 1;
constexpr uint8_t DAC_READ_SPI_MODE = 3;
constexpr uint32_t ADC_SPI_FREQUENCY_HZ = 14990000;
constexpr uint8_t ADC_SPI_MODE = 0;

// Global DAC voltage limits - channel specific arrays
namespace DACLimits {
  extern float upper_voltage_limit[NUM_DAC_CHANNELS];
  extern float lower_voltage_limit[NUM_DAC_CHANNELS];
  extern bool limits_initialized;
  
  // Initialize all channel limits to default values
  inline void initializeLimits() {
    if (!limits_initialized) {
      for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
        upper_voltage_limit[i] = 10.0;
        lower_voltage_limit[i] = -10.0;
      }
      limits_initialized = true;
    }
  }
}
