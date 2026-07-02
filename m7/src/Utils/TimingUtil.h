#pragma once

#include <Arduino.h>

#include "Config.h"
#include "stm32h7xx.h"

namespace TimingUtil {
extern volatile AdcBoardMask adcFlag;
extern volatile bool dacFlag;
extern volatile uint32_t dacSpiMisstepEvents;
extern volatile uint32_t adcSpiMisstepEvents;
extern volatile uint32_t adcConversionMisstepEvents;
extern volatile AdcBoardMask adcConversionInProgressMask;
extern volatile AdcBoardMask adcConversionWatchMask;
extern volatile bool adcConversionStartedFlag;

void resetTimers();
void resetTimingWatchdog(AdcBoardMask adc_watch_mask = 0);
void stopAndResetAdcTimer();
void startAdcTimer();
void setupTimerOnlyDac(uint32_t period_us);
void setupTimersOnlyADC(uint32_t adc_period_us);
void setupTimersTimeSeries(uint32_t dac_period_us,
                           uint32_t adc_period_us,
                           AdcBoardMask adc_watch_mask);
void setupTimersTimeSeriesRamp(uint32_t dac_period_us,
                               uint32_t adc_period_us,
                               AdcBoardMask adc_watch_mask);
void stopTimeSeriesTimers();
void setupTimersDacLed(uint64_t period_us, uint64_t phase_shift_us,
                       AdcBoardMask adc_watch_mask);
void disableDacInterrupt();
void disableAdcInterrupt();
bool consumeDacFlag();
bool consumeAdcFlag(AdcBoardMask expectedMask);
bool consumeAnyAdcFlag();
bool consumeAdcConversionStartedFlag();

constexpr AdcBoardMask adcBoardBit(int boardIndex) {
  return static_cast<AdcBoardMask>(1) << boardIndex;
}

template <int boardIndex>
void adcSyncISR() {
  const AdcBoardMask bit = adcBoardBit(boardIndex);
  adcConversionInProgressMask &= ~bit;
  adcFlag |= bit;
  __SEV();
}
}  // namespace TimingUtil
