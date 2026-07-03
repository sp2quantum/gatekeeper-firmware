#include "Commands/BufferRamps/BufferRampCommon.h"

#include <algorithm>

#include "Peripherals/ADC/ADCController.h"
#include "Peripherals/DAC/DACController.h"
#include "Utils/TimingUtil.h"
#include "shared_memory.h"

namespace BufferRampCommon {

namespace {

constexpr uint16_t kInvalidTiming = 0;
constexpr uint16_t kMinDacLedSettlingUs = 20;
constexpr uint16_t kMinAdcConversionUs = 82;

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

uint16_t lookupTiming(const uint16_t table[5][5], const int* adcChannels,
                      int numAdcChannels) {
  uint8_t boardDepth[NUM_ADC_BOARDS] = {};
  sortedAdcBoardDepths(adcChannels, numAdcChannels, boardDepth);
  for (int i = 0; i < NUM_ADC_BOARDS; i++) {
    if (boardDepth[i] > NUM_CHANNELS_PER_ADC_BOARD ||
        boardDepth[i] > 4) {
      return kInvalidTiming;
    }
  }

  uint32_t timing = table[boardDepth[0]][NUM_ADC_BOARDS > 1 ? boardDepth[1] : 0];
  for (int i = 2; i < NUM_ADC_BOARDS && boardDepth[i] > 0; i++) {
    timing += table[boardDepth[i]][0];
  }
  if (timing > 65535) return 65535;
  return static_cast<uint16_t>(timing);
}

OperationResult validateMinimumTiming(const char* label, float actual,
                                      uint16_t minimum) {
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

uint16_t timeSeriesTableMinimumUs(const int* adcChannels, int numAdcChannels,
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

  switch (mode) {
    case TimeSeriesTimingMode::OneD:
      return lookupTiming(kOneD, adcChannels, numAdcChannels);
    case TimeSeriesTimingMode::TwoDNormal:
      return lookupTiming(kTwoDNormal, adcChannels, numAdcChannels);
    case TimeSeriesTimingMode::TwoDRetrace:
      return lookupTiming(kTwoDRetrace, adcChannels, numAdcChannels);
    case TimeSeriesTimingMode::TwoDSnake:
      return lookupTiming(kTwoDSnake, adcChannels, numAdcChannels);
  }
  return kInvalidTiming;
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

uint16_t minimumDacLedIntervalUs(const int* adcChannels, int numAdcChannels) {
  static constexpr uint16_t kTable[5][5] = {
      {0, 120, 200, 300, 400},
      {120, 120, 220, 320, 420},
      {200, 220, 240, 320, 420},
      {300, 320, 320, 340, 440},
      {400, 420, 420, 440, 460},
  };
  const uint16_t tableMinimum = lookupTiming(kTable, adcChannels, numAdcChannels);
  if (tableMinimum == kInvalidTiming) return tableMinimum;

  float maxSingleConversionUs = 0.0f;
  for (int i = 0; i < numAdcChannels; i++) {
    const float conversionUs =
        ADCController::getConversionTimeFloat(adcChannels[i]);
    if (conversionUs > maxSingleConversionUs) {
      maxSingleConversionUs = conversionUs;
    }
  }
  if (maxSingleConversionUs <= 90.0f) return tableMinimum;

  const float maxBoardConversionUs =
      maxAdcConversionTimePerBoard(adcChannels, numAdcChannels);
  const uint32_t conversionMinimum =
      static_cast<uint32_t>(maxBoardConversionUs * 1.2f + 0.999f);
  const uint32_t minimum =
      std::max<uint32_t>(tableMinimum, conversionMinimum);
  return minimum > 65535 ? 65535 : static_cast<uint16_t>(minimum);
}

uint16_t minimumAwgWithAdcIntervalUs(int numDacChannels,
                                    const int* adcChannels,
                                    int numAdcChannels) {
  const uint16_t base = minimumDacLedIntervalUs(adcChannels, numAdcChannels);
  if (base == kInvalidTiming) return base;
  uint16_t extra = 0;
  if (numDacChannels >= 8) {
    extra = 40 * static_cast<uint16_t>((numDacChannels + 7) / 8);
  } else if (numDacChannels >= 4 && base >= 200 && base < 300) {
    extra = 20;
  }
  return base + extra;
}

uint16_t minimumDacOnlyIntervalUs(int numDacChannels) {
  if (numDacChannels <= 0) return kInvalidTiming;
  if (numDacChannels <= 4) return 20;
  return 40;
}

uint16_t minimumTimeSeriesAdcIntervalUs(const int* adcChannels,
                                        int numAdcChannels,
                                        TimeSeriesTimingMode mode) {
  const uint16_t tableMinimum =
      timeSeriesTableMinimumUs(adcChannels, numAdcChannels, mode);
  return tableMinimum;
}

uint16_t minimumBoxcarConversionTimeUs(const int* adcChannels,
                                       int numAdcChannels) {
  uint8_t boardDepth[NUM_ADC_BOARDS] = {};
  sortedAdcBoardDepths(adcChannels, numAdcChannels, boardDepth);
  const uint8_t maxDepth = boardDepth[0];
  if (maxDepth == 0 || maxDepth > 4) return kInvalidTiming;
  static constexpr uint16_t kByMaxCardDepth[5] = {0, 300, 300, 500, 800};
  return kByMaxCardDepth[maxDepth];
}

OperationResult validateDacLedTiming(float dacIntervalArg,
                                     float dacSettlingTimeArg,
                                     const int* adcChannels,
                                     int numAdcChannels) {
  OperationResult settlingResult = validateMinimumTiming(
      "DAC settling time", dacSettlingTimeArg, kMinDacLedSettlingUs);
  if (!settlingResult.isSuccess()) return settlingResult;
  return validateMinimumTiming(
      "DAC interval", dacIntervalArg,
      minimumDacLedIntervalUs(adcChannels, numAdcChannels));
}

OperationResult validateTimeSeriesTiming(float adcIntervalArg,
                                         const int* adcChannels,
                                         int numAdcChannels,
                                         TimeSeriesTimingMode mode) {
  const uint16_t minimum =
      minimumTimeSeriesAdcIntervalUs(adcChannels, numAdcChannels, mode);
  if (minimum == kInvalidTiming) {
    return OperationResult::Failure("Invalid ADC channel timing split");
  }
  if (adcIntervalArg >= static_cast<float>(minimum)) {
    return OperationResult::Success();
  }

  const uint16_t tableMinimum =
      timeSeriesTableMinimumUs(adcChannels, numAdcChannels, mode);
  return OperationResult::Failure(
      "ADC interval too short for time-series ramp (" +
      String(adcIntervalArg, 3) + " us < computed minimum " +
      String(minimum) +
      " us). Minimum is the empirical table floor for the selected "
      "ADC-board split. " +
      formatAdcTimingDetails(adcChannels, numAdcChannels) +
      "; empirical table floor=" + String(tableMinimum) + " us");
}

OperationResult validateAwgWithAdcTiming(float dacIntervalArg,
                                         int numDacChannels,
                                         const int* adcChannels,
                                         int numAdcChannels) {
  return validateMinimumTiming(
      "DAC interval", dacIntervalArg,
      minimumAwgWithAdcIntervalUs(numDacChannels, adcChannels,
                                  numAdcChannels));
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
