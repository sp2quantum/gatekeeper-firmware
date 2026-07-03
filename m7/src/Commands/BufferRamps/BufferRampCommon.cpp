#include "Commands/BufferRamps/BufferRampCommon.h"

#include <algorithm>

#include "Peripherals/ADC/ADCController.h"
#include "Peripherals/DAC/DACController.h"
#include "Utils/TimingUtil.h"
#include "shared_memory.h"

namespace BufferRampCommon {

namespace {

constexpr uint32_t kInvalidTiming = 0;
constexpr uint32_t kMinDacLedSettlingUs = 20;
constexpr uint32_t kMinAdcConversionUs = 82;

// Empirical buffers calibrated on hardware via _SUDO timing sweeps
// (2026-07-03); see test_outputs/timing_calibration_20260703/DERIVATION.md.
// "Busiest-board conversion sum" is the largest per-ADC-board sum of the
// selected channels' actual conversion times.
//
// Time series: conversions free-run on the ADC's own clock, so the sample
// interval needs a relative margin over the busiest-board sum before every
// sample is a fresh conversion.
constexpr float kTimeSeriesConversionMarginRatio = 1.05f;
constexpr uint32_t kTimeSeriesConversionMarginUs = 5;
// DAC-led: conversion start is timer-gated each cycle, so the buffers are
// fixed. The LDAC/settle path needs settling + sum + latch buffer; the
// readout path needs sum + per-register-read cost (reads overlap the next
// settling window).
constexpr uint32_t kDacLedLatchBufferUs = 12;
constexpr uint32_t kAdcReadoutPerSampleUs = 15;
constexpr uint32_t kAdcReadoutBaseUs = 15;
// AWG with ADC: one conversion per DAC step plus readout and DAC writes.
constexpr uint32_t kAwgWithAdcPerDacUs = 5;
constexpr uint32_t kAwgWithAdcBaseUs = 25;

void sortedAdcBoardDepths(const int* adcChannels, int numAdcChannels,
                          uint8_t boardDepth[NUM_ADC_BOARDS]) {
  std::fill(boardDepth, boardDepth + NUM_ADC_BOARDS, 0);
  for (int i = 0; i < numAdcChannels; i++) {
    const int channel = adcChannels[i];
    if (channel < 0 || channel >= NUM_ADC_CHANNELS) continue;
    const uint8_t board = adcBoardForChannel(channel);
    if (board >= NUM_ADC_BOARDS) continue;
    boardDepth[board]++;
  }
  std::sort(boardDepth, boardDepth + NUM_ADC_BOARDS,
            [](uint8_t a, uint8_t b) { return a > b; });
}

uint32_t lookupTiming(const uint16_t table[5][5], const int* adcChannels,
                      int numAdcChannels) {
  uint8_t boardDepth[NUM_ADC_BOARDS] = {};
  sortedAdcBoardDepths(adcChannels, numAdcChannels, boardDepth);
  for (int i = 0; i < NUM_ADC_BOARDS; i++) {
    // The timing tables only cover 0-4 selected channels per board.
    if (boardDepth[i] > 4) {
      return kInvalidTiming;
    }
  }

  uint32_t timing = table[boardDepth[0]][NUM_ADC_BOARDS > 1 ? boardDepth[1] : 0];
  for (int i = 2; i < NUM_ADC_BOARDS && boardDepth[i] > 0; i++) {
    timing += table[boardDepth[i]][0];
  }
  return timing;
}

uint32_t ceilBusiestBoardConversionSumUs(const int* adcChannels,
                                         int numAdcChannels) {
  const float sum = maxAdcConversionTimePerBoard(adcChannels, numAdcChannels);
  if (sum <= 0.0f) return 0;
  return static_cast<uint32_t>(sum + 0.999f);
}

OperationResult validateMinimumTiming(const char* label, float actual,
                                      uint32_t minimum) {
  if (minimum == kInvalidTiming) {
    return OperationResult::Failure("Invalid ADC channel timing split");
  }
  if (actual < static_cast<float>(minimum)) {
    return OperationResult::Failure(String(label) + " too short (" +
                                    String(actual, 3) +
                                    " us < minimum " + String(minimum) +
                                    " us)");
  }
  return OperationResult::Success();
}

String formatAdcTimingDetails(const int* adcChannels, int numAdcChannels) {
  float boardConversionTimeUs[NUM_ADC_BOARDS] = {};
  String channels = "selected ADC conversion times: ";
  for (int i = 0; i < numAdcChannels; i++) {
    const int channel = adcChannels[i];
    if (i > 0) channels += ", ";
    channels += "ch" + String(channel) + "=";
    if (channel < 0 || channel >= NUM_ADC_CHANNELS) {
      channels += "invalid";
      continue;
    }
    const float conversionUs =
        ADCController::getConversionTimeFloat(channel);
    boardConversionTimeUs[adcBoardForChannel(channel)] += conversionUs;
    channels += String(conversionUs, 3) + " us";
  }

  channels += "; same-board conversion sums: ";
  for (int board = 0; board < NUM_ADC_BOARDS; board++) {
    if (board > 0) channels += ", ";
    channels += "board" + String(board) + "=" +
                String(boardConversionTimeUs[board], 3) + " us";
  }
  return channels;
}

OperationResult validateMinimumTimingWithDetails(
    const char* label, float actual, uint32_t minimum, const char* rule,
    const int* adcChannels, int numAdcChannels) {
  if (minimum == kInvalidTiming) {
    return OperationResult::Failure("Invalid ADC channel timing split");
  }
  if (actual >= static_cast<float>(minimum)) {
    return OperationResult::Success();
  }
  return OperationResult::Failure(
      String(label) + " too short (" + String(actual, 3) + " us < minimum " +
      String(minimum) + " us). " + rule + " " +
      formatAdcTimingDetails(adcChannels, numAdcChannels));
}

}  // namespace

bool isValidDacChannelCount(int count) {
  return count >= 1 && count <= NUM_DAC_CHANNELS;
}

bool isValidAdcChannelCount(int count) {
  return count >= 1 && count <= NUM_ADC_CHANNELS;
}

bool isUint32AtLeast(float value, uint32_t minimum) {
  return value >= static_cast<float>(minimum) &&
         static_cast<double>(value) <= 4294967295.0;
}

uint8_t adcBoardForChannel(int channel) {
  return static_cast<uint8_t>(channel / NUM_CHANNELS_PER_ADC_BOARD);
}

OperationResult validateDacChannels(const int* channels, int count,
                                    bool rejectDuplicates) {
  bool seen[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < count; i++) {
    if (!DACController::isChannelIndexValid(channels[i])) {
      return OperationResult::Failure("Invalid DAC channel index " +
                                      String(channels[i]));
    }
    if (rejectDuplicates && seen[channels[i]]) {
      return OperationResult::Failure("Duplicate DAC channel index " +
                                      String(channels[i]));
    }
    seen[channels[i]] = true;
  }
  return OperationResult::Success();
}

OperationResult validateAdcChannels(const int* channels, int count,
                                    bool rejectDuplicates) {
  bool seen[NUM_ADC_CHANNELS] = {};
  for (int i = 0; i < count; i++) {
    if (!ADCController::isChannelIndexValid(channels[i])) {
      return OperationResult::Failure("Invalid ADC channel index " +
                                      String(channels[i]));
    }
    if (rejectDuplicates && seen[channels[i]]) {
      return OperationResult::Failure("Duplicate ADC channel index " +
                                      String(channels[i]));
    }
    seen[channels[i]] = true;
  }
  return OperationResult::Success();
}

OperationResult validateRampChannels(const int* dacChannels,
                                     int numDacChannels,
                                     const int* adcChannels,
                                     int numAdcChannels,
                                     bool rejectDuplicateDacChannels,
                                     bool rejectDuplicateAdcChannels) {
  OperationResult dacValidation = validateDacChannels(
      dacChannels, numDacChannels, rejectDuplicateDacChannels);
  if (!dacValidation.isSuccess()) {
    return dacValidation;
  }
  return validateAdcChannels(adcChannels, numAdcChannels,
                             rejectDuplicateAdcChannels);
}

OperationResult finishRampTimingWatchdog(bool includeAdcConversionMissteps) {
  const uint32_t dacSpiMissteps = TimingUtil::dacSpiMisstepEvents;
  const uint32_t adcSpiMissteps = TimingUtil::adcSpiMisstepEvents;
  const uint32_t adcConversionMissteps =
      TimingUtil::adcConversionMisstepEvents;
  if (dacSpiMissteps == 0 && adcSpiMissteps == 0 &&
      (!includeAdcConversionMissteps || adcConversionMissteps == 0)) {
    return OperationResult::Success();
  }

  String message =
      "Ramp timing misstep during ramp dac_spi_missteps=" +
      String(dacSpiMissteps) + " adc_spi_missteps=" +
      String(adcSpiMissteps);
  if (includeAdcConversionMissteps) {
    message += " adc_conversion_missteps=" + String(adcConversionMissteps);
  }
  return OperationResult::Failure(message);
}

OperationResult dacWriteFailure(int channel, double voltage) {
  String message = "DAC write failed ch=" + String(channel) +
                   " v=" + String(voltage, 9);

  if (!DACController::isChannelIndexValid(channel)) {
    return OperationResult::Failure(message + " source=invalid_channel");
  }

  const float lowerBound = DACController::getLowerBound(channel);
  const float upperBound = DACController::getUpperBound(channel);
  if (voltage < lowerBound || voltage > upperBound) {
    message += " source=bounds bounds=[" + String(lowerBound, 9) + "," +
               String(upperBound, 9) + "]";
    return OperationResult::Failure(message);
  }

  return OperationResult::Failure(message + " source=spi");
}

OperationResult dacSetWriteFailure(int numDacChannels, const int* dacChannels,
                                   const double* voltages) {
  for (int i = 0; i < numDacChannels; i++) {
    const int channel = dacChannels[i];
    if (!DACController::isChannelIndexValid(channel)) {
      return dacWriteFailure(channel, voltages[i]);
    }
    if (voltages[i] < DACController::getLowerBound(channel) ||
        voltages[i] > DACController::getUpperBound(channel)) {
      return dacWriteFailure(channel, voltages[i]);
    }
  }
  return dacWriteFailure(dacChannels[0], voltages[0]);
}

int maxSelectedAdcChannelsPerBoard(const int* adcChannels,
                                   int numAdcChannels) {
  int boardDepth[NUM_ADC_BOARDS] = {};
  for (int i = 0; i < numAdcChannels; i++) {
    const int channel = adcChannels[i];
    if (channel < 0 || channel >= NUM_ADC_CHANNELS) {
      continue;
    }
    boardDepth[adcBoardForChannel(channel)]++;
  }
  return *std::max_element(boardDepth, boardDepth + NUM_ADC_BOARDS);
}

float maxAdcConversionTimePerBoard(const int* adcChannels,
                                   int numAdcChannels) {
  float boardConversionTimeUs[NUM_ADC_BOARDS] = {};
  for (int i = 0; i < numAdcChannels; i++) {
    const int channel = adcChannels[i];
    if (channel < 0 || channel >= NUM_ADC_CHANNELS) {
      continue;
    }
    boardConversionTimeUs[adcBoardForChannel(channel)] +=
        ADCController::getConversionTimeFloat(channel);
  }
  return *std::max_element(boardConversionTimeUs,
                           boardConversionTimeUs + NUM_ADC_BOARDS);
}

uint32_t minimumDacLedIntervalUs(uint32_t dacSettlingTimeUs,
                                 const int* adcChannels, int numAdcChannels,
                                 int numAdcAverages) {
  const uint32_t boardSum =
      ceilBusiestBoardConversionSumUs(adcChannels, numAdcChannels);
  if (boardSum == 0) return kInvalidTiming;

  const uint64_t readsPerCycle =
      static_cast<uint64_t>(numAdcChannels) *
      static_cast<uint64_t>(numAdcAverages < 1 ? 1 : numAdcAverages);
  const uint64_t latchPath =
      static_cast<uint64_t>(dacSettlingTimeUs) + boardSum +
      kDacLedLatchBufferUs;
  const uint64_t readoutPath =
      boardSum + kAdcReadoutPerSampleUs * readsPerCycle + kAdcReadoutBaseUs;
  const uint64_t minimum = std::max(latchPath, readoutPath);
  return minimum > 0xFFFFFFFFULL ? 0xFFFFFFFFUL
                                 : static_cast<uint32_t>(minimum);
}

uint32_t minimumAwgWithAdcIntervalUs(int numDacChannels,
                                     const int* adcChannels,
                                     int numAdcChannels) {
  const uint32_t boardSum =
      ceilBusiestBoardConversionSumUs(adcChannels, numAdcChannels);
  if (boardSum == 0) return kInvalidTiming;
  return boardSum + kAdcReadoutPerSampleUs * numAdcChannels +
         kAwgWithAdcPerDacUs * numDacChannels + kAwgWithAdcBaseUs;
}

uint32_t minimumDacOnlyIntervalUs(int numDacChannels) {
  if (numDacChannels <= 0) return kInvalidTiming;
  if (numDacChannels <= 4) return 20;
  return 40;
}

uint32_t minimumTimeSeriesAdcIntervalUs(const int* adcChannels,
                                        int numAdcChannels,
                                        TimeSeriesTimingMode mode) {
  static constexpr uint16_t kOneD[5][5] = {
      {0, 80, 80, 100, 160},
      {80, 80, 80, 80, 160},
      {80, 80, 80, 180, 140},
      {100, 120, 120, 200, 240},
      {160, 160, 300, 240, 160},
  };
  static constexpr uint16_t kTwoDNormal[5][5] = {
      {0, 105, 102, 150, 131},
      {105, 119, 107, 156, 204},
      {102, 107, 114, 163, 208},
      {150, 156, 163, 171, 434},
      {131, 204, 208, 434, 452},
  };
  static constexpr uint16_t kTwoDRetrace[5][5] = {
      {0, 105, 102, 151, 197},
      {105, 119, 107, 156, 204},
      {102, 107, 114, 163, 208},
      {151, 156, 163, 171, 434},
      {197, 204, 208, 434, 448},
  };
  static constexpr uint16_t kTwoDSnake[5][5] = {
      {0, 107, 101, 148, 131},
      {107, 119, 107, 156, 204},
      {101, 107, 114, 164, 209},
      {148, 156, 164, 170, 442},
      {131, 204, 209, 442, 454},
  };

  uint32_t tableFloor = kInvalidTiming;
  switch (mode) {
    case TimeSeriesTimingMode::OneD:
      tableFloor = lookupTiming(kOneD, adcChannels, numAdcChannels);
      break;
    case TimeSeriesTimingMode::TwoDNormal:
      tableFloor = lookupTiming(kTwoDNormal, adcChannels, numAdcChannels);
      break;
    case TimeSeriesTimingMode::TwoDRetrace:
      tableFloor = lookupTiming(kTwoDRetrace, adcChannels, numAdcChannels);
      break;
    case TimeSeriesTimingMode::TwoDSnake:
      tableFloor = lookupTiming(kTwoDSnake, adcChannels, numAdcChannels);
      break;
  }
  if (tableFloor == kInvalidTiming) return kInvalidTiming;

  // The ADCs free-run at their configured conversion times; sampling faster
  // than the busiest board updates just re-reads stale conversions.
  const float boardSum =
      maxAdcConversionTimePerBoard(adcChannels, numAdcChannels);
  if (boardSum <= 0.0f) return kInvalidTiming;
  const uint32_t conversionMinimum =
      static_cast<uint32_t>(boardSum * kTimeSeriesConversionMarginRatio +
                            0.999f) +
      kTimeSeriesConversionMarginUs;
  return std::max(tableFloor, conversionMinimum);
}

uint32_t minimumBoxcarConversionTimeUs(const int* adcChannels,
                                       int numAdcChannels) {
  uint8_t boardDepth[NUM_ADC_BOARDS] = {};
  sortedAdcBoardDepths(adcChannels, numAdcChannels, boardDepth);
  const uint8_t busiest = boardDepth[0];
  const uint8_t second = NUM_ADC_BOARDS > 1 ? boardDepth[1] : 0;
  if (busiest == 0 || busiest > 4 || second > 4) return kInvalidTiming;
  // Measured minimum clean conversion time by (busiest, second-busiest)
  // card depth. The pass/fail boundary is set by readout SPI phase alignment
  // against the conversion timer and is NOT monotonic in the conversion time
  // (padded values were measured to fail where these exact values pass), so
  // the table holds the exact hardware-verified minima.
  static constexpr uint16_t kByCardDepths[5][5] = {
      {0, 0, 0, 0, 0},
      {160, 82, 0, 0, 0},
      {120, 120, 200, 0, 0},
      {200, 200, 200, 200, 0},
      {300, 800, 300, 300, 1600},
  };
  return kByCardDepths[busiest][second];
}

OperationResult validateDacLedTiming(float dacIntervalArg,
                                     float dacSettlingTimeArg,
                                     const int* adcChannels,
                                     int numAdcChannels,
                                     int numAdcAverages) {
  OperationResult settlingResult = validateMinimumTiming(
      "DAC settling time", dacSettlingTimeArg, kMinDacLedSettlingUs);
  if (!settlingResult.isSuccess()) return settlingResult;
  const uint32_t settlingUs = static_cast<uint32_t>(dacSettlingTimeArg);
  return validateMinimumTimingWithDetails(
      "DAC interval", dacIntervalArg,
      minimumDacLedIntervalUs(settlingUs, adcChannels, numAdcChannels,
                              numAdcAverages),
      "Minimum is max(settling + busiest-board conversion sum + 12us, "
      "conversion sum + 15us*(numAdcChannels*numAdcAverages) + 15us).",
      adcChannels, numAdcChannels);
}

OperationResult validateTimeSeriesTiming(float adcIntervalArg,
                                         const int* adcChannels,
                                         int numAdcChannels,
                                         TimeSeriesTimingMode mode) {
  return validateMinimumTimingWithDetails(
      "ADC interval", adcIntervalArg,
      minimumTimeSeriesAdcIntervalUs(adcChannels, numAdcChannels, mode),
      "Minimum is the larger of the empirical floor for the selected "
      "ADC-board split and 1.05x the busiest-board conversion sum + 5us "
      "(sampling faster than the ADCs convert repeats stale samples).",
      adcChannels, numAdcChannels);
}

OperationResult validateAwgWithAdcTiming(float dacIntervalArg,
                                         int numDacChannels,
                                         const int* adcChannels,
                                         int numAdcChannels) {
  return validateMinimumTimingWithDetails(
      "DAC interval", dacIntervalArg,
      minimumAwgWithAdcIntervalUs(numDacChannels, adcChannels,
                                  numAdcChannels),
      "Minimum is busiest-board conversion sum + 15us*numAdcChannels + "
      "5us*numDacChannels + 25us.",
      adcChannels, numAdcChannels);
}

OperationResult validateDacOnlyTiming(float dacIntervalArg,
                                      int numDacChannels) {
  return validateMinimumTiming("DAC interval", dacIntervalArg,
                               minimumDacOnlyIntervalUs(numDacChannels));
}

OperationResult validateBoxcarTiming(float adcConversionTimeArg,
                                     const int* adcChannels,
                                     int numAdcChannels) {
  OperationResult hardwareMinimum = validateMinimumTiming(
      "ADC conversion time", adcConversionTimeArg, kMinAdcConversionUs);
  if (!hardwareMinimum.isSuccess()) return hardwareMinimum;
  return validateMinimumTiming(
      "ADC conversion time", adcConversionTimeArg,
      minimumBoxcarConversionTimeUs(adcChannels, numAdcChannels));
}

bool sendVoltageFrame(const double* packets, size_t length) {
  if (sendVoltageFrameToGateway(packets, length)) {
    return true;
  }
  requestWorkerStop();
  return false;
}

bool encodeDacVoltagePackets(int numDacChannels, const int* dacChannels,
                             const double* voltages,
                             byte packets[NUM_DAC_CHANNELS][3]) {
  for (int i = 0; i < numDacChannels; i++) {
    if (!DACController::encodeVoltagePacket(
            dacChannels[i], static_cast<float>(voltages[i]), packets[i])) {
      return false;
    }
  }
  return true;
}

bool writeDacPackets(int numDacChannels, const int* dacChannels,
                     byte packets[NUM_DAC_CHANNELS][3]) {
  for (int i = 0; i < numDacChannels; i++) {
    if (!DACController::writeVoltagePacketNoLdac(dacChannels[i],
                                                 packets[i])) {
      return false;
    }
  }
  return true;
}

}  // namespace BufferRampCommon
