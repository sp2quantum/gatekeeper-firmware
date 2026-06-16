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

struct AdcCardLoad {
  uint8_t card0;
  uint8_t card1;
};

AdcCardLoad adcCardLoadForChannels(const int* adcChannels,
                                   int numAdcChannels) {
  AdcCardLoad load = {};
  for (int i = 0; i < numAdcChannels; i++) {
    const int channel = adcChannels[i];
    if (channel < 0 || channel >= NUM_ADC_CHANNELS) continue;
    if (adcBoardForChannel(channel) == 0) {
      load.card0++;
    } else {
      load.card1++;
    }
  }
  return load;
}

uint16_t lookupTiming(const uint16_t table[5][5], const int* adcChannels,
                      int numAdcChannels) {
  const AdcCardLoad load =
      adcCardLoadForChannels(adcChannels, numAdcChannels);
  if (load.card0 > 4 || load.card1 > 4) return kInvalidTiming;
  return table[load.card0][load.card1];
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
  return lookupTiming(kTable, adcChannels, numAdcChannels);
}

uint16_t minimumAwgWithAdcIntervalUs(int numDacChannels,
                                    const int* adcChannels,
                                    int numAdcChannels) {
  const uint16_t base = minimumDacLedIntervalUs(adcChannels, numAdcChannels);
  if (base == kInvalidTiming) return base;
  uint16_t extra = 0;
  if (numDacChannels >= 8) {
    extra = 40;
  } else if (numDacChannels >= 4 && base >= 200 && base < 300) {
    extra = 20;
  }
  return base + extra;
}

uint16_t minimumTimeSeriesAdcIntervalUs(const int* adcChannels,
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
      {0, 80, 80, 80, 80},
      {80, 80, 80, 80, 280},
      {80, 80, 80, 120, 160},
      {80, 80, 80, 120, 160},
      {80, 280, 160, 160, 160},
  };
  static constexpr uint16_t kTwoDRetrace[5][5] = {
      {0, 80, 80, 80, 80},
      {80, 80, 80, 80, 280},
      {80, 80, 80, 80, 160},
      {80, 80, 80, 120, 160},
      {80, 280, 160, 160, 160},
  };
  static constexpr uint16_t kTwoDSnake[5][5] = {
      {0, 80, 80, 80, 80},
      {80, 80, 80, 80, 260},
      {80, 80, 80, 80, 160},
      {80, 120, 80, 120, 220},
      {80, 280, 160, 160, 270},
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

uint16_t minimumBoxcarConversionTimeUs(const int* adcChannels,
                                       int numAdcChannels) {
  const AdcCardLoad load =
      adcCardLoadForChannels(adcChannels, numAdcChannels);
  const uint8_t maxDepth = max(load.card0, load.card1);
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
  return validateMinimumTiming(
      "ADC interval", adcIntervalArg,
      minimumTimeSeriesAdcIntervalUs(adcChannels, numAdcChannels, mode));
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
