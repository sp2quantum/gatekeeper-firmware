#pragma once

#include <Arduino.h>

#include "stm32h7xx.h"

struct TimingUtil {
  static volatile uint8_t adcFlag;
  static volatile bool dacFlag;
  static volatile uint32_t dacSpiMisstepEvents;
  static volatile uint32_t adcSpiMisstepEvents;
  static volatile uint32_t adcConversionMisstepEvents;
  static volatile uint8_t adcConversionInProgressMask;
  static volatile uint8_t adcConversionWatchMask;
  static volatile bool adcConversionStartedFlag;

  static void resetTimers();
  static void resetTimingWatchdog(uint8_t adc_watch_mask = 0);
  static void stopAndResetAdcTimer();
  static void startAdcTimer();
  static void setupTimerOnlyDac(uint32_t period_us);
  static void setupTimersOnlyADC(uint32_t adc_period_us);
  static void setupTimersTimeSeries(uint32_t dac_period_us,
                                    uint32_t adc_period_us,
                                    uint8_t adc_watch_mask);
  static void setupTimersTimeSeriesRamp(uint32_t dac_period_us,
                                        uint32_t adc_period_us,
                                        uint8_t adc_watch_mask);
  static void stopTimeSeriesTimers();
  static void setupTimersDacLed(uint64_t period_us, uint64_t phase_shift_us,
                                uint8_t adc_watch_mask);
  static void disableDacInterrupt();
  static void disableAdcInterrupt();
  static bool consumeDacFlag();
  static bool consumeAdcFlag(uint8_t expectedMask);
  static bool consumeAnyAdcFlag();
  static bool consumeAdcConversionStartedFlag();

  template <int boardIndex>
  static void adcSyncISR() {
    adcConversionInProgressMask &= static_cast<uint8_t>(~(1u << boardIndex));
    adcFlag |= 1 << boardIndex;
    __SEV();
  }
};
