#pragma once

#include <Arduino.h>

#include "stm32h7xx.h"

namespace TimingUtil {
extern volatile uint8_t adcFlag;
extern volatile bool dacFlag;
extern volatile uint32_t dacSpiMisstepEvents;
extern volatile uint32_t adcSpiMisstepEvents;
extern volatile uint32_t adcConversionMisstepEvents;
extern volatile uint8_t adcConversionInProgressMask;
extern volatile uint8_t adcConversionWatchMask;
extern volatile bool adcConversionStartedFlag;

void resetTimers();
void resetTimingWatchdog(uint8_t adc_watch_mask = 0);
void stopAndResetAdcTimer();
void startAdcTimer();
void setupTimerOnlyDac(uint32_t period_us);
void setupTimersOnlyADC(uint32_t adc_period_us);
void setupTimersTimeSeries(uint32_t dac_period_us,
                           uint32_t adc_period_us,
                           uint8_t adc_watch_mask);
void setupTimersTimeSeriesRamp(uint32_t dac_period_us,
                               uint32_t adc_period_us,
                               uint8_t adc_watch_mask);
void stopTimeSeriesTimers();
void setupTimersDacLed(uint64_t period_us, uint64_t phase_shift_us,
                       uint8_t adc_watch_mask);
void disableDacInterrupt();
void disableAdcInterrupt();
bool consumeDacFlag();
bool consumeAdcFlag(uint8_t expectedMask);
bool consumeAnyAdcFlag();
bool consumeAdcConversionStartedFlag();

template <int boardIndex>
void adcSyncISR() {
  adcConversionInProgressMask &= static_cast<uint8_t>(~(1u << boardIndex));
  adcFlag |= 1 << boardIndex;
  __SEV();
}
}  // namespace TimingUtil
