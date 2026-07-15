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
// Time series: conversions free-run, and individual update spacings jitter
// around the nominal conversion time (chop-phase alternation), so the sample
// interval needs a small fixed margin over the busiest-board sum. A fine
// offset scan measured duplicate-free sampling at S + 8us for every
// conversion time from 82us to 5.2ms; the margin is fixed, not relative,
// because the jitter does not grow with the conversion time.
constexpr uint32_t kTimeSeriesConversionMarginUs = 15;
// DAC-led: conversion start is timer-gated once per DAC point. ADC averaging
// collects distinct continuous-conversion rounds while that DAC point remains
// fixed, then reads the final round and stops conversion before the next point.
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

uint32_t lookupTimingByDepths(const uint16_t table[5][5],
                              const uint8_t boardDepth[NUM_ADC_BOARDS]) {
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

uint32_t timeSeriesFloorByDepths(TimeSeriesTimingMode mode,
                                 const uint8_t boardDepth[NUM_ADC_BOARDS]) {
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

  switch (mode) {
    case TimeSeriesTimingMode::OneD:
      return lookupTimingByDepths(kOneD, boardDepth);
    case TimeSeriesTimingMode::TwoDNormal:
      return lookupTimingByDepths(kTwoDNormal, boardDepth);
    case TimeSeriesTimingMode::TwoDRetrace:
      return lookupTimingByDepths(kTwoDRetrace, boardDepth);
    case TimeSeriesTimingMode::TwoDSnake:
      return lookupTimingByDepths(kTwoDSnake, boardDepth);
  }
  return kInvalidTiming;
}

uint32_t boxcarFloorByDepths(const uint8_t boardDepth[NUM_ADC_BOARDS]) {
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

uint32_t ceilConversionSumUs(float sumUs) {
  if (sumUs <= 0.0f) return 0;
  return static_cast<uint32_t>(sumUs + 0.999f);
}

uint32_t timeSeriesConversionTermUs(float busiestBoardSumUs) {
  if (busiestBoardSumUs <= 0.0f) return kInvalidTiming;
  return ceilConversionSumUs(busiestBoardSumUs) +
         kTimeSeriesConversionMarginUs;
}

uint32_t dacLedMinimumForSum(uint32_t dacSettlingTimeUs, uint32_t ceilSumUs,
                             int numAdcChannels, int numAdcAverages) {
  if (ceilSumUs == 0) return kInvalidTiming;
  if (numAdcChannels < 1 || numAdcAverages < 1) return kInvalidTiming;

  const uint64_t readoutPerRound =
      kAdcReadoutPerSampleUs * static_cast<uint64_t>(numAdcChannels) +
      kAdcReadoutBaseUs;
  // Intermediate averaged conversions are still running continuously. Each
  // result must be read before the next board scan completes; a longer DAC
  // interval cannot make an impossible per-round readout safe.
  if (numAdcAverages > 1 && readoutPerRound >= ceilSumUs) {
    return kInvalidTiming;
  }

  const uint64_t conversionRounds =
      static_cast<uint64_t>(ceilSumUs) *
      static_cast<uint64_t>(numAdcAverages);
  const uint64_t minimum = static_cast<uint64_t>(dacSettlingTimeUs) +
                           conversionRounds + readoutPerRound +
                           kDacLedLatchBufferUs;
  return minimum > 0xFFFFFFFFULL ? 0xFFFFFFFFUL
                                 : static_cast<uint32_t>(minimum);
}

uint32_t awgWithAdcMinimumForSum(uint32_t ceilSumUs, int numDacChannels,
                                 int numAdcChannels) {
  if (ceilSumUs == 0) return kInvalidTiming;
  return ceilSumUs + kAdcReadoutPerSampleUs * numAdcChannels +
         kAwgWithAdcPerDacUs * numDacChannels + kAwgWithAdcBaseUs;
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
  return channels + ".";
}

String splitText(const uint8_t boardDepth[NUM_ADC_BOARDS]) {
  String text;
  for (int board = 0; board < NUM_ADC_BOARDS; board++) {
    if (board > 0) text += "/";
    text += String(boardDepth[board]);
  }
  return text;
}

bool sameDepths(const uint8_t a[NUM_ADC_BOARDS],
                const uint8_t b[NUM_ADC_BOARDS]) {
  for (int board = 0; board < NUM_ADC_BOARDS; board++) {
    if (a[board] != b[board]) return false;
  }
  return true;
}

// The most even redistribution of the selected channels (with their current
// conversion times) across the ADC boards: balanced counts, with the slowest
// conversions spread greedily so the busiest-board sum is minimized.
struct BalancedDistribution {
  uint8_t depths[NUM_ADC_BOARDS];
  float busiestSumUs;
  bool valid;
};

BalancedDistribution balancedAdcDistribution(const int* adcChannels,
                                             int numAdcChannels) {
  BalancedDistribution result = {};
  float times[NUM_ADC_CHANNELS] = {};
  int selected = 0;
  for (int i = 0; i < numAdcChannels && selected < NUM_ADC_CHANNELS; i++) {
    const int channel = adcChannels[i];
    if (channel < 0 || channel >= NUM_ADC_CHANNELS) continue;
    times[selected++] = ADCController::getConversionTimeFloat(channel);
  }
  if (selected == 0) return result;

  const int base = selected / NUM_ADC_BOARDS;
  const int remainder = selected % NUM_ADC_BOARDS;
  uint8_t capacity[NUM_ADC_BOARDS];
  for (int board = 0; board < NUM_ADC_BOARDS; board++) {
    capacity[board] = static_cast<uint8_t>(base + (board < remainder ? 1 : 0));
    if (capacity[board] > NUM_CHANNELS_PER_ADC_BOARD) return result;
    result.depths[board] = capacity[board];
  }

  std::sort(times, times + selected, [](float a, float b) { return a > b; });
  float sums[NUM_ADC_BOARDS] = {};
  uint8_t counts[NUM_ADC_BOARDS] = {};
  for (int i = 0; i < selected; i++) {
    int best = -1;
    for (int board = 0; board < NUM_ADC_BOARDS; board++) {
      if (counts[board] >= capacity[board]) continue;
      if (best < 0 || sums[board] < sums[best]) best = board;
    }
    if (best < 0) return result;
    sums[best] += times[i];
    counts[best]++;
  }
  result.busiestSumUs = *std::max_element(sums, sums + NUM_ADC_BOARDS);
  result.valid = true;
  return result;
}

String redistributionSuggestion(uint32_t currentMinimum,
                                uint32_t balancedMinimum,
                                const uint8_t currentDepths[NUM_ADC_BOARDS],
                                const uint8_t balancedDepths[NUM_ADC_BOARDS]) {
  if (balancedMinimum == kInvalidTiming ||
      balancedMinimum >= currentMinimum ||
      sameDepths(currentDepths, balancedDepths)) {
    return "";
  }
  return " Splitting the selected ADC channels evenly across the ADC boards "
         "(" + splitText(balancedDepths) + " split) would lower the minimum "
         "to " + String(balancedMinimum) + " us.";
}

OperationResult timingTooShortFailure(const char* label, float actual,
                                      uint32_t minimum, const String& rule,
                                      const String& details,
                                      const String& suggestion) {
  String message = String(label) + " too short (" + String(actual, 3) +
                   " us < minimum " + String(minimum) + " us). " + rule;
  if (details.length() > 0) {
    message += " " + details;
  }
  return OperationResult::Failure(message + suggestion);
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

bool isTimerPeriodUs(float value, uint32_t minimum) {
  if (!isUint32AtLeast(value, minimum)) return false;
  return TimingUtil::isTimerPeriodRepresentable(static_cast<uint32_t>(value));
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
  const uint32_t ceilSum = ceilConversionSumUs(
      maxAdcConversionTimePerBoard(adcChannels, numAdcChannels));
  return dacLedMinimumForSum(dacSettlingTimeUs, ceilSum, numAdcChannels,
                             numAdcAverages);
}

uint32_t minimumAwgWithAdcIntervalUs(int numDacChannels,
                                     const int* adcChannels,
                                     int numAdcChannels) {
  const uint32_t ceilSum = ceilConversionSumUs(
      maxAdcConversionTimePerBoard(adcChannels, numAdcChannels));
  return awgWithAdcMinimumForSum(ceilSum, numDacChannels, numAdcChannels);
}

uint32_t minimumDacOnlyIntervalUs(int numDacChannels) {
  if (numDacChannels <= 0) return kInvalidTiming;
  if (numDacChannels <= 4) return 20;
  return 40;
}

uint32_t minimumTimeSeriesAdcIntervalUs(const int* adcChannels,
                                        int numAdcChannels,
                                        TimeSeriesTimingMode mode) {
  uint8_t boardDepth[NUM_ADC_BOARDS] = {};
  sortedAdcBoardDepths(adcChannels, numAdcChannels, boardDepth);
  const uint32_t tableFloor = timeSeriesFloorByDepths(mode, boardDepth);
  if (tableFloor == kInvalidTiming) return kInvalidTiming;

  const uint32_t conversionTerm = timeSeriesConversionTermUs(
      maxAdcConversionTimePerBoard(adcChannels, numAdcChannels));
  if (conversionTerm == kInvalidTiming) return kInvalidTiming;
  return std::max(tableFloor, conversionTerm);
}

uint32_t minimumBoxcarConversionTimeUs(const int* adcChannels,
                                       int numAdcChannels) {
  uint8_t boardDepth[NUM_ADC_BOARDS] = {};
  sortedAdcBoardDepths(adcChannels, numAdcChannels, boardDepth);
  return boxcarFloorByDepths(boardDepth);
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
  const uint32_t minimum = minimumDacLedIntervalUs(
      settlingUs, adcChannels, numAdcChannels, numAdcAverages);
  if (minimum == kInvalidTiming) {
    return OperationResult::Failure(
        "ADC readout is too slow to collect distinct averaged conversions");
  }
  if (dacIntervalArg >= static_cast<float>(minimum)) {
    return OperationResult::Success();
  }

  uint8_t boardDepth[NUM_ADC_BOARDS] = {};
  sortedAdcBoardDepths(adcChannels, numAdcChannels, boardDepth);
  const BalancedDistribution balanced =
      balancedAdcDistribution(adcChannels, numAdcChannels);
  const uint32_t balancedMinimum =
      balanced.valid
          ? dacLedMinimumForSum(
                settlingUs, ceilConversionSumUs(balanced.busiestSumUs),
                numAdcChannels, numAdcAverages)
          : kInvalidTiming;

  const String rule =
      "Minimum for the selected " + splitText(boardDepth) +
      " ADC-board split is settling + numAdcAverages*busiest-board "
      "conversion sum + 15us*numAdcChannels + 27us.";
  return timingTooShortFailure(
      "DAC interval", dacIntervalArg, minimum, rule,
      formatAdcTimingDetails(adcChannels, numAdcChannels),
      redistributionSuggestion(minimum, balancedMinimum, boardDepth,
                               balanced.depths));
}

OperationResult validateTimeSeriesTiming(float adcIntervalArg,
                                         const int* adcChannels,
                                         int numAdcChannels,
                                         TimeSeriesTimingMode mode) {
  uint8_t boardDepth[NUM_ADC_BOARDS] = {};
  sortedAdcBoardDepths(adcChannels, numAdcChannels, boardDepth);
  const uint32_t tableFloor = timeSeriesFloorByDepths(mode, boardDepth);
  const uint32_t conversionTerm = timeSeriesConversionTermUs(
      maxAdcConversionTimePerBoard(adcChannels, numAdcChannels));
  if (tableFloor == kInvalidTiming || conversionTerm == kInvalidTiming) {
    return OperationResult::Failure("Invalid ADC channel timing split");
  }
  const uint32_t minimum = std::max(tableFloor, conversionTerm);
  if (adcIntervalArg >= static_cast<float>(minimum)) {
    return OperationResult::Success();
  }

  const BalancedDistribution balanced =
      balancedAdcDistribution(adcChannels, numAdcChannels);
  uint32_t balancedMinimum = kInvalidTiming;
  if (balanced.valid) {
    const uint32_t balancedFloor =
        timeSeriesFloorByDepths(mode, balanced.depths);
    const uint32_t balancedConversionTerm =
        timeSeriesConversionTermUs(balanced.busiestSumUs);
    if (balancedFloor != kInvalidTiming &&
        balancedConversionTerm != kInvalidTiming) {
      balancedMinimum = std::max(balancedFloor, balancedConversionTerm);
    }
  }

  const String rule =
      "Minimum is the larger of the empirical floor for the selected " +
      splitText(boardDepth) + " ADC-board split (" + String(tableFloor) +
      " us) and the busiest-board conversion sum + 15us (" +
      String(conversionTerm) +
      " us; sampling faster than the ADCs convert repeats stale samples).";
  return timingTooShortFailure(
      "ADC interval", adcIntervalArg, minimum, rule,
      formatAdcTimingDetails(adcChannels, numAdcChannels),
      redistributionSuggestion(minimum, balancedMinimum, boardDepth,
                               balanced.depths));
}

OperationResult validateAwgWithAdcTiming(float dacIntervalArg,
                                         int numDacChannels,
                                         const int* adcChannels,
                                         int numAdcChannels) {
  const uint32_t minimum = minimumAwgWithAdcIntervalUs(
      numDacChannels, adcChannels, numAdcChannels);
  if (minimum == kInvalidTiming) {
    return OperationResult::Failure("Invalid ADC channel timing split");
  }
  if (dacIntervalArg >= static_cast<float>(minimum)) {
    return OperationResult::Success();
  }

  uint8_t boardDepth[NUM_ADC_BOARDS] = {};
  sortedAdcBoardDepths(adcChannels, numAdcChannels, boardDepth);
  const BalancedDistribution balanced =
      balancedAdcDistribution(adcChannels, numAdcChannels);
  const uint32_t balancedMinimum =
      balanced.valid
          ? awgWithAdcMinimumForSum(
                ceilConversionSumUs(balanced.busiestSumUs), numDacChannels,
                numAdcChannels)
          : kInvalidTiming;

  const String rule =
      "Minimum for the selected " + splitText(boardDepth) +
      " ADC-board split is busiest-board conversion sum + "
      "15us*numAdcChannels + 5us*numDacChannels + 25us.";
  return timingTooShortFailure(
      "DAC interval", dacIntervalArg, minimum, rule,
      formatAdcTimingDetails(adcChannels, numAdcChannels),
      redistributionSuggestion(minimum, balancedMinimum, boardDepth,
                               balanced.depths));
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

  uint8_t boardDepth[NUM_ADC_BOARDS] = {};
  sortedAdcBoardDepths(adcChannels, numAdcChannels, boardDepth);
  const uint32_t minimum = boxcarFloorByDepths(boardDepth);
  if (minimum == kInvalidTiming) {
    return OperationResult::Failure("Invalid ADC channel timing split");
  }
  if (adcConversionTimeArg >= static_cast<float>(minimum)) {
    return OperationResult::Success();
  }

  const BalancedDistribution balanced =
      balancedAdcDistribution(adcChannels, numAdcChannels);
  const uint32_t balancedMinimum =
      balanced.valid ? boxcarFloorByDepths(balanced.depths) : kInvalidTiming;

  const String rule =
      "Minimum is the hardware-verified boxcar floor for the selected " +
      splitText(boardDepth) +
      " ADC-board split; the floor is necessary but not sufficient (see the "
      "README boxcar table notes).";
  return timingTooShortFailure(
      "ADC conversion time", adcConversionTimeArg, minimum, rule, "",
      redistributionSuggestion(minimum, balancedMinimum, boardDepth,
                               balanced.depths));
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
