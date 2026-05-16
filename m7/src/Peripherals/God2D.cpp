#include "Peripherals/God2D.h"

#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Peripherals/God.h"
#include "Peripherals/PeripheralCommsController.h"
#include "Utils/FastGpio.h"

namespace {
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

bool isBooleanArg(float value) {
  return value == 0.0f || value == 1.0f;
}

constexpr int kTimeSeriesRowStartAdcDiscards = 2;
constexpr uint32_t kTimeSeriesAdcStartDelayUs = 10;

void setupTimeSeriesTimers(uint32_t dacIntervalUs, uint32_t adcIntervalUs,
                           uint8_t adcMask) {
  if (dacIntervalUs == adcIntervalUs &&
      kTimeSeriesAdcStartDelayUs < adcIntervalUs) {
    TimingUtil::setupTimersDacLed(dacIntervalUs, kTimeSeriesAdcStartDelayUs,
                                  adcMask);
    return;
  }
  TimingUtil::setupTimersTimeSeries(dacIntervalUs, adcIntervalUs, adcMask);
}

OperationResult validateDacChannels(const int* channels, int count) {
  for (int i = 0; i < count; i++) {
    if (!DACController::isChannelIndexValid(channels[i])) {
      return OperationResult::Failure("Invalid DAC channel index " +
                                      String(channels[i]));
    }
  }
  return OperationResult::Success();
}

OperationResult validateAdcChannels(const int* channels, int count) {
  for (int i = 0; i < count; i++) {
    if (!ADCController::isChannelIndexValid(channels[i])) {
      return OperationResult::Failure("Invalid ADC channel index " +
                                      String(channels[i]));
    }
  }
  return OperationResult::Success();
}

OperationResult finishRampTimingWatchdog(
    bool includeAdcConversionMissteps = true) {
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

void pauseTimeSeriesTimers() {
  TimingUtil::disableDacInterrupt();
  TimingUtil::disableAdcInterrupt();
  TimingUtil::dacFlag = false;
  TimingUtil::adcFlag = 0;
  TimingUtil::stopAndResetAdcTimer();
  FastGpio::digitalWrite(adc_sync, false);
}

void calculateDacLed2DVoltages(int pointIndex, int numStepsFast,
                               bool retrace, bool snake,
                               int numDacChannels, const float* startPoint,
                               const float* fastAxisVector,
                               const float* slowAxisStep,
                               double voltages[NUM_DAC_CHANNELS]) {
  const int scansPerSlowStep = (retrace && !snake) ? 2 : 1;
  const int scanIndex = pointIndex / numStepsFast;
  const int fastStep = pointIndex % numStepsFast;
  const int slowStep = scanIndex / scansPerSlowStep;
  const bool retraceScan = (retrace && !snake) &&
                           ((scanIndex % scansPerSlowStep) == 1);
  const bool snakeReverse = snake && ((slowStep % 2) != 0);
  const bool reverseFastAxis = retraceScan || snakeReverse;
  const double fastDenominator =
      numStepsFast > 1 ? static_cast<double>(numStepsFast - 1) : 1.0;
  double fastFraction = numStepsFast > 1
                            ? static_cast<double>(fastStep) / fastDenominator
                            : 0.0;
  if (reverseFastAxis) {
    fastFraction = 1.0 - fastFraction;
  }

  for (int i = 0; i < numDacChannels; i++) {
    voltages[i] = static_cast<double>(startPoint[i]) +
                  static_cast<double>(slowStep) *
                      static_cast<double>(slowAxisStep[i]) +
                  fastFraction * static_cast<double>(fastAxisVector[i]);
  }
}

struct DacLed2DStreamPoint {
  int pointIndex;
  bool sendAdc;
};

bool usesRowStartDummyConversions(bool retrace, bool snake,
                                  int numStepsFast,
                                  int numStepsSlow) {
  return !retrace && !snake && numStepsFast > 1 && numStepsSlow > 1;
}

int dacLed2DStreamPointCount(int totalPoints, bool retrace, bool snake,
                             int numStepsFast, int numStepsSlow) {
  if (!usesRowStartDummyConversions(retrace, snake, numStepsFast,
                                    numStepsSlow)) {
    return totalPoints;
  }
  return totalPoints + numStepsSlow - 1;
}

DacLed2DStreamPoint getDacLed2DStreamPoint(int streamIndex, bool retrace,
                                           bool snake, int numStepsFast,
                                           int numStepsSlow) {
  if (!usesRowStartDummyConversions(retrace, snake, numStepsFast,
                                    numStepsSlow) ||
      streamIndex < numStepsFast) {
    return {streamIndex, true};
  }

  const int laterRowStreamLength = numStepsFast + 1;
  const int remaining = streamIndex - numStepsFast;
  const int row = 1 + remaining / laterRowStreamLength;
  const int rowPosition = remaining % laterRowStreamLength;
  if (row >= numStepsSlow) {
    return {numStepsFast * numStepsSlow - 1, true};
  }
  if (rowPosition == 0) {
    return {row * numStepsFast, false};
  }
  return {row * numStepsFast + rowPosition - 1, true};
}

OperationResult runPreparedDacLedBufferRamp2D(
    int numDacChannels, int numAdcChannels, int numStepsFast,
    int numStepsSlow, bool retrace, bool snake, int numAdcAverages,
    uint32_t dac_interval_us, uint32_t dac_settling_time_us, int* dacChannels,
    float* startPoint, float* fastAxisVector, float* slowAxisVector,
    int* adcChannels, uint8_t adcMask) {
  const int scansPerSlowStep = (retrace && !snake) ? 2 : 1;
  const int totalPoints = numStepsFast * numStepsSlow * scansPerSlowStep;
  if (totalPoints < 1) {
    return OperationResult::Failure("Invalid number of 2D ramp points");
  }
  const int totalStreamPoints = dacLed2DStreamPointCount(
      totalPoints, retrace, snake, numStepsFast, numStepsSlow);

  float slowAxisStep[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < numDacChannels; i++) {
    slowAxisStep[i] =
        numStepsSlow > 1 ? slowAxisVector[i] / (numStepsSlow - 1) : 0.0f;
  }

  double currentVoltages[NUM_DAC_CHANNELS] = {};
  const DacLed2DStreamPoint firstStreamPoint = getDacLed2DStreamPoint(
      0, retrace, snake, numStepsFast, numStepsSlow);
  calculateDacLed2DVoltages(firstStreamPoint.pointIndex, numStepsFast,
                            retrace, snake, numDacChannels, startPoint,
                            fastAxisVector, slowAxisStep, currentVoltages);
  byte nextDacPackets[NUM_DAC_CHANNELS][3] = {};
  if (!encodeDacVoltagePackets(numDacChannels, dacChannels, currentVoltages,
                               nextDacPackets) ||
      !writeDacPackets(numDacChannels, dacChannels, nextDacPackets)) {
    return dacWriteFailure(dacChannels[0], currentVoltages[0]);
  }

  int dacStreamPointsPreloaded = 1;
  bool nextDacPacketsReady = false;
  double nextVoltages[NUM_DAC_CHANNELS] = {};
  auto prepareNextDacPackets = [&]() {
    if (dacStreamPointsPreloaded >= totalStreamPoints) {
      nextDacPacketsReady = false;
      return true;
    }
    const DacLed2DStreamPoint streamPoint = getDacLed2DStreamPoint(
        dacStreamPointsPreloaded, retrace, snake, numStepsFast,
        numStepsSlow);
    calculateDacLed2DVoltages(streamPoint.pointIndex, numStepsFast, retrace,
                              snake, numDacChannels, startPoint,
                              fastAxisVector, slowAxisStep, nextVoltages);
    nextDacPacketsReady = encodeDacVoltagePackets(
        numDacChannels, dacChannels, nextVoltages, nextDacPackets);
    return nextDacPacketsReady;
  };
  if (!prepareNextDacPackets()) {
    return dacWriteFailure(dacChannels[0], nextVoltages[0]);
  }

  FastGpio::digitalWrite(adc_sync, false);
  TimingUtil::setupTimersDacLed(dac_interval_us, dac_settling_time_us,
                                adcMask);
  TimingUtil::dacFlag = false;
  TimingUtil::adcFlag = 0;

  double packets[NUM_ADC_CHANNELS] = {};
  const double numAdcAveragesInv = 1.0 / static_cast<double>(numAdcAverages);
  int adcReads = 0;
  int adcStreamReads = 0;
  bool voltageOverflow = false;

  while (adcReads < totalPoints && !isWorkerStopRequested()) {
    __WFE();

    const bool adcPending = TimingUtil::consumeAdcFlag(adcMask);
    bool haveAdcPackets = false;
    if (adcPending) {
      if (adcStreamReads >= totalStreamPoints) {
        return OperationResult::Failure("2D ramp ADC stream overrun");
      }
      const DacLed2DStreamPoint streamPoint = getDacLed2DStreamPoint(
          adcStreamReads, retrace, snake, numStepsFast, numStepsSlow);
      adcStreamReads++;
      for (int i = 0; i < numAdcChannels; i++) {
        double total = 0.0;
        for (int j = 0; j < numAdcAverages; j++) {
          total += ADCController::getVoltageDataNoTransaction(adcChannels[i]);
        }
        packets[i] = total * numAdcAveragesInv;
      }
      FastGpio::digitalWrite(adc_sync, false);
      if (streamPoint.sendAdc) {
        adcReads++;
        haveAdcPackets = true;
      }
    }

    const bool dacPending = dacStreamPointsPreloaded < totalStreamPoints &&
                            TimingUtil::consumeDacFlag();
    if (dacPending) {
      if (!nextDacPacketsReady ||
          !writeDacPackets(numDacChannels, dacChannels, nextDacPackets)) {
        return dacWriteFailure(dacChannels[0], nextVoltages[0]);
      }
      dacStreamPointsPreloaded++;
      if (!prepareNextDacPackets()) {
        return dacWriteFailure(dacChannels[0], nextVoltages[0]);
      }
    }

    if (haveAdcPackets) {
      if (!sendVoltageFrame(packets, numAdcChannels)) {
        voltageOverflow = true;
        break;
      }
    }
  }

  if (isWorkerStopRequested()) {
    if (voltageOverflow) {
      return OperationResult::Failure("Voltage output buffer overflow");
    }
    return OperationResult::Failure("RAMPING_STOPPED");
  }
  if (voltageOverflow) {
    return OperationResult::Failure("Voltage output buffer overflow");
  }

  return OperationResult::Success();
}

OperationResult runBufferedTimeSeriesBufferRamp(
    int numDacChannels, int numAdcChannels, int numSteps,
    uint32_t dac_interval_us, uint32_t adc_interval_us, int* dacChannels,
    float* dacV0s, float* dacVfs, int* adcChannels, uint8_t adcMask,
    int initialAdcSamplesToDiscard, bool holdInitialDacUntilDiscarded,
    bool holdInitialDacThroughFirstVisibleSample) {
  const int savedDataSize = numSteps * dac_interval_us / adc_interval_us;
  if (savedDataSize < 1) {
    return OperationResult::Failure("Invalid time-series sample count");
  }

  std::vector<double> bufferedFrames(
      static_cast<size_t>(savedDataSize) *
      static_cast<size_t>(numAdcChannels));

  pauseTimeSeriesTimers();

  int steps = 0;
  int x = 0;
  int discardedAdcSamples = 0;
  const int discardCount =
      initialAdcSamplesToDiscard > 0 ? initialAdcSamplesToDiscard : 0;
  bool voltageOverflow = false;

  double voltageStepSize[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < numDacChannels; i++) {
    voltageStepSize[i] =
        numSteps > 1 ? (dacVfs[i] - dacV0s[i]) / (numSteps - 1) : 0.0;
  }

  double nextVoltageSet[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < numDacChannels; i++) {
    nextVoltageSet[i] = dacV0s[i];
  }
  byte nextDacPackets[NUM_DAC_CHANNELS][3] = {};
  bool nextDacPacketsReady = false;
  auto prepareNextDacPackets = [&]() {
    if (steps >= numSteps) {
      nextDacPacketsReady = false;
      return true;
    }
    nextDacPacketsReady = encodeDacVoltagePackets(
        numDacChannels, dacChannels, nextVoltageSet, nextDacPackets);
    return nextDacPacketsReady;
  };

  for (int i = 0; i < numDacChannels; i++) {
    if (!DACController::setVoltageNoTransactionNoLdac(dacChannels[i],
                                                      dacV0s[i])) {
      pauseTimeSeriesTimers();
      return dacWriteFailure(dacChannels[i], dacV0s[i]);
    }
    nextVoltageSet[i] += voltageStepSize[i];
  }
  DACController::toggleLdac();
  steps++;

  FastGpio::digitalWrite(adc_sync, false);
  for (int i = 0; i < numAdcChannels; i++) {
    ADCController::startContinuousConversion(adcChannels[i]);
    ADCController::setRDYFN(adcChannels[i]);
  }
  setupTimeSeriesTimers(dac_interval_us, adc_interval_us, adcMask);

  if (!prepareNextDacPackets()) {
    pauseTimeSeriesTimers();
    return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
  }

  FastGpio::digitalWrite(adc_sync, false);
  TimingUtil::dacFlag = false;
  TimingUtil::adcFlag = 0;
  bool dacWriteQueued = false;

  double packets[NUM_ADC_CHANNELS] = {};

  while ((discardedAdcSamples < discardCount || x < savedDataSize ||
          steps < numSteps) &&
         !isWorkerStopRequested()) {
    __WFE();
    const bool adcPending =
        (discardedAdcSamples < discardCount || x < savedDataSize) &&
        TimingUtil::consumeAdcFlag(adcMask);
    const bool holdingInitialDac =
        holdInitialDacUntilDiscarded &&
        (discardedAdcSamples < discardCount ||
         (holdInitialDacThroughFirstVisibleSample && x == 0));
    const bool holdingInitialDacBeforeAdc = holdingInitialDac;
    if ((holdingInitialDac || steps < numSteps) &&
        TimingUtil::consumeDacFlag()) {
      dacWriteQueued = true;
    }

    bool haveAdcPackets = false;
    if (adcPending) {
      for (int i = 0; i < numAdcChannels; i++) {
        packets[i] = ADCController::getVoltageDataNoTransaction(adcChannels[i]);
      }
      FastGpio::digitalWrite(adc_sync, false);
      if (discardedAdcSamples < discardCount) {
        discardedAdcSamples++;
      } else {
        haveAdcPackets = true;
      }
    }

    if (dacWriteQueued && TimingUtil::adcConversionInProgressMask == 0) {
      if (holdingInitialDacBeforeAdc) {
        // The initial DAC code is already loaded; avoid redundant SPI writes
        // while the hidden row-start ADC sample is being discarded.
        dacWriteQueued = false;
      } else if (!nextDacPacketsReady ||
                 !writeDacPackets(numDacChannels, dacChannels,
                                  nextDacPackets)) {
        pauseTimeSeriesTimers();
        return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
      } else {
        dacWriteQueued = false;
        for (int i = 0; i < numDacChannels; i++) {
          nextVoltageSet[i] += voltageStepSize[i];
        }
        steps++;
        if (!prepareNextDacPackets()) {
          pauseTimeSeriesTimers();
          return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
        }
      }
    }

    if (haveAdcPackets) {
      const size_t frameOffset =
          static_cast<size_t>(x) * static_cast<size_t>(numAdcChannels);
      for (int i = 0; i < numAdcChannels; i++) {
        bufferedFrames[frameOffset + static_cast<size_t>(i)] = packets[i];
      }
      x++;
    }
  }

  pauseTimeSeriesTimers();

  if (isWorkerStopRequested()) {
    if (voltageOverflow) {
      return OperationResult::Failure("Voltage output buffer overflow");
    }
    return OperationResult::Failure("RAMPING_STOPPED");
  }
  if (voltageOverflow) {
    return OperationResult::Failure("Voltage output buffer overflow");
  }

  for (int frame = 0; frame < savedDataSize && !isWorkerStopRequested();
       frame++) {
    const size_t frameOffset = static_cast<size_t>(frame) *
                               static_cast<size_t>(numAdcChannels);
    if (!sendVoltageFrame(&bufferedFrames[frameOffset], numAdcChannels)) {
      voltageOverflow = true;
      break;
    }
  }

  if (isWorkerStopRequested()) {
    if (voltageOverflow) {
      return OperationResult::Failure("Voltage output buffer overflow");
    }
    return OperationResult::Failure("RAMPING_STOPPED");
  }
  if (voltageOverflow) {
    return OperationResult::Failure("Voltage output buffer overflow");
  }

  return OperationResult::Success();
}

}

  void God2D::setup() { initializeRegistry(); }



  void God2D::initializeRegistry() {
    registerMemberFunctionVector(timeSeriesBufferRamp2D,
                                 "2D_TIME_SERIES_BUFFER_RAMP");
    registerMemberFunctionVector(dacLedBufferRamp2D, "2D_DAC_LED_BUFFER_RAMP");
  }



  // timeSeriesBufferRamp2D:
  // Arguments (in order):
  // numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
  // dacInterval_us, adcInterval_us, retrace (0.0f = false, 1.0f = true), snake (0.0f = false, 1.0f = true),
  // [dacChannelID] * numDacChannels,
  // [startPoint] * numDacChannels,
  // [fastAxisVector] * numDacChannels,
  // [slowAxisVector] * numDacChannels,
  // [adcChannelID] * numAdcChannels
  //
  // The fast/slow axis vectors define a 2D plane in the N-dimensional DAC phase space.
  // Position(s,f) = startPoint + s*slowAxisVector + f*fastAxisVector where s,f is in [0,1]
  // This allows probing arbitrary 2D planar subspaces anywhere in the full DAC phase space.
  OperationResult God2D::timeSeriesBufferRamp2D(
      const std::vector<float> &args) {
    // Minimum required arguments:
    // 8 initial params + numDacChannels + 3*numDacChannels vectors + numAdcChannels
    if (args.size() < 8) {
      return OperationResult::Failure(
          "Not enough arguments provided for 2D ramp");
    }

    size_t currentIndex = 0;

    // Parse initial parameters
    int numDacChannels = static_cast<int>(args[currentIndex++]);
    int numAdcChannels = static_cast<int>(args[currentIndex++]);
    int numStepsFast = static_cast<int>(args[currentIndex++]);
    int numStepsSlow = static_cast<int>(args[currentIndex++]);
    const float dacIntervalArg = args[currentIndex++];
    const float adcIntervalArg = args[currentIndex++];
    const float retraceArg = args[currentIndex++];
    const float snakeArg = args[currentIndex++];

    if (!isValidDacChannelCount(numDacChannels) ||
        !isValidAdcChannelCount(numAdcChannels)) {
      return OperationResult::Failure("Invalid number of channels");
    }
    if (!isUint32AtLeast(adcIntervalArg, 1) ||
        !isUint32AtLeast(dacIntervalArg, 1)) {
      return OperationResult::Failure("Invalid interval");
    }
    if (!isBooleanArg(retraceArg) || !isBooleanArg(snakeArg)) {
      return OperationResult::Failure("Invalid 2D scan boolean argument");
    }
    if (numStepsFast < 1 || numStepsSlow < 1) {
      return OperationResult::Failure("Invalid number of steps");
    }
    uint32_t dac_interval_us = static_cast<uint32_t>(dacIntervalArg);
    uint32_t adc_interval_us = static_cast<uint32_t>(adcIntervalArg);
    bool retrace = retraceArg != 0.0f;
    bool snake = snakeArg != 0.0f;

    const size_t expected =
        currentIndex + static_cast<size_t>(numDacChannels) +
        3u * static_cast<size_t>(numDacChannels) +
        static_cast<size_t>(numAdcChannels);
    if (args.size() != expected) {
      return OperationResult::Failure("Incorrect number of arguments for 2D ramp");
    }

    // Parse DAC channel IDs
    int dacChannels[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      dacChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    // Parse start point
    float startPoint[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      startPoint[i] = args[currentIndex++];
    }

    // Parse fast axis vector
    float fastAxisVector[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      fastAxisVector[i] = args[currentIndex++];
    }

    // Parse slow axis vector
    float slowAxisVector[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      slowAxisVector[i] = args[currentIndex++];
    }

    // Parse ADC Channels
    if (args.size() < currentIndex + numAdcChannels) {
      return OperationResult::Failure("Not enough arguments for ADC channels");
    }

    int adcChannels[NUM_ADC_CHANNELS] = {};
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    OperationResult dacValidation =
        validateDacChannels(dacChannels, numDacChannels);
    if (!dacValidation.isSuccess()) {
      return dacValidation;
    }
    OperationResult adcValidation =
        validateAdcChannels(adcChannels, numAdcChannels);
    if (!adcValidation.isSuccess()) {
      return adcValidation;
    }

    // Check voltage bounds for all four corners of the 2D scan rectangle (both calibrated bounds AND global limits)
    for (int i = 0; i < numDacChannels; i++) {
      int ch = dacChannels[i];
      float lowerBound = DACController::getLowerBound(ch);
      float upperBound = DACController::getUpperBound(ch);
      
      // Four corners: startPoint, startPoint+fast, startPoint+slow, startPoint+fast+slow
      float corner1 = startPoint[i];
      float corner2 = startPoint[i] + fastAxisVector[i];
      float corner3 = startPoint[i] + slowAxisVector[i];
      float corner4 = startPoint[i] + fastAxisVector[i] + slowAxisVector[i];
      
      float minVoltage = min(min(corner1, corner2), min(corner3, corner4));
      float maxVoltage = max(max(corner1, corner2), max(corner3, corner4));
      
      if (minVoltage < lowerBound || maxVoltage > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " 2D scan range [" + String(minVoltage, 6) + 
                                        ", " + String(maxVoltage, 6) + 
                                        "]V exceeds bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
    }

    // Calculate slow axis step sizes for each DAC channel
    float slowStepSize[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; i++) {
      slowStepSize[i] =
          numStepsSlow > 1 ? slowAxisVector[i] / (numStepsSlow - 1) : 0.0f;
    }

    // Track current position in phase space (starting at startPoint)
    float currentSlowPosition[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; i++) {
      currentSlowPosition[i] = startPoint[i];
    }

    clearWorkerStopRequest();
    PeripheralCommsController::dataLedOn();

    uint8_t adcMask = 0u;
    God::BoardUsage boardUsage{0, std::vector<uint8_t>()};
    OperationResult prepareResult = God::prepareTimeSeriesBufferRampHardware(
        numAdcChannels, dac_interval_us, adc_interval_us, adcChannels, adcMask,
        boardUsage);
    if (!prepareResult.isSuccess()) {
      PeripheralCommsController::dataLedOff();
      return prepareResult;
    }

    OperationResult rampResult = OperationResult::Success();

    // Iterate over slow steps
    for (int slowStep = 0; slowStep < numStepsSlow && !isWorkerStopRequested(); ++slowStep) {
      const bool reverseFastAxis = snake && ((slowStep % 2) != 0);

      // Calculate start and end voltages for fast axis ramp
      // Position = currentSlowPosition + t * fastAxisVector (where t goes from 0 to 1)
      float fastV0s[NUM_DAC_CHANNELS] = {};
      float fastVfs[NUM_DAC_CHANNELS] = {};
      
      for (int i = 0; i < numDacChannels; ++i) {
        if (reverseFastAxis) {
          fastV0s[i] = currentSlowPosition[i] + fastAxisVector[i];
          fastVfs[i] = currentSlowPosition[i];
        } else {
          fastV0s[i] = currentSlowPosition[i];
          fastVfs[i] = currentSlowPosition[i] + fastAxisVector[i];
        }
      }

      // Execute fast axis ramp
      const bool discardInitialConversion = true;
      OperationResult ramp1Result = runBufferedTimeSeriesBufferRamp(
          numDacChannels, numAdcChannels, numStepsFast, dac_interval_us,
          adc_interval_us, dacChannels, fastV0s, fastVfs, adcChannels,
          adcMask, discardInitialConversion ? kTimeSeriesRowStartAdcDiscards : 0,
          discardInitialConversion, false);

      // Execute retrace if requested (and not in snake mode)
      OperationResult ramp2Result = OperationResult::Success();
      if (retrace && !snake) {
        ramp2Result = runBufferedTimeSeriesBufferRamp(
            numDacChannels, numAdcChannels, numStepsFast, dac_interval_us,
            adc_interval_us, dacChannels, fastVfs, fastV0s, adcChannels,
            adcMask, kTimeSeriesRowStartAdcDiscards, true, false);
      }

      // Check for errors
      if (!ramp1Result.isSuccess() && !ramp2Result.isSuccess()) {
        rampResult = OperationResult::Failure(ramp1Result.getMessage() + "\n" +
                                              ramp2Result.getMessage());
        break;
      } else if (!ramp1Result.isSuccess()) {
        rampResult = OperationResult::Failure(ramp1Result.getMessage());
        break;
      } else if (!ramp2Result.isSuccess()) {
        rampResult = OperationResult::Failure(ramp2Result.getMessage());
        break;
      }

      // Advance along slow axis
      for (int i = 0; i < numDacChannels; ++i) {
        currentSlowPosition[i] += slowStepSize[i];
      }
    }

    God::cleanupTimeSeriesBufferRampHardware(numAdcChannels, adcChannels,
                                             boardUsage);

    PeripheralCommsController::dataLedOff();

    if (!rampResult.isSuccess()) {
      if (isWorkerStopRequested()) {
        clearWorkerStopRequest();
      }
      return rampResult;
    }

    if (isWorkerStopRequested()) {
      clearWorkerStopRequest();
      return OperationResult::Failure("2D RAMPING_STOPPED");
    }

    return finishRampTimingWatchdog(false);
  }




  // dacLedBufferRamp2D:
  // Arguments (in order):
  // numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
  // dacInterval_us, dacSettlingTime_us, retrace (0.0f = false, 1.0f = true), snake (0.0f = false, 1.0f = true),
  // numAdcAverages,
  // [dacChannelID] * numDacChannels,
  // [startPoint] * numDacChannels,
  // [fastAxisVector] * numDacChannels,
  // [slowAxisVector] * numDacChannels,
  // [adcChannelID] * numAdcChannels
  //
  // The fast/slow axis vectors define a 2D plane in the N-dimensional DAC phase space.
  // Position(s,f) = startPoint + s*slowAxisVector + f*fastAxisVector where s,f is in [0,1]
  // This allows probing arbitrary 2D planar subspaces anywhere in the full DAC phase space.
  OperationResult God2D::dacLedBufferRamp2D(const std::vector<float> &args) {
    // Minimum required arguments:
    // 9 initial params + numDacChannels + 3*numDacChannels vectors + numAdcChannels
    if (args.size() < 9) {
      return OperationResult::Failure(
          "Not enough arguments provided for 2D ramp");
    }

    size_t currentIndex = 0;

    // Parse initial parameters
    int numDacChannels = static_cast<int>(args[currentIndex++]);
    int numAdcChannels = static_cast<int>(args[currentIndex++]);
    int numStepsFast = static_cast<int>(args[currentIndex++]);
    int numStepsSlow = static_cast<int>(args[currentIndex++]);
    const float dacIntervalArg = args[currentIndex++];
    const float dacSettlingTimeArg = args[currentIndex++];
    const float retraceArg = args[currentIndex++];
    const float snakeArg = args[currentIndex++];
    int numAdcAverages = static_cast<int>(args[currentIndex++]);

    if (!isValidDacChannelCount(numDacChannels) ||
        !isValidAdcChannelCount(numAdcChannels)) {
      return OperationResult::Failure("Invalid number of channels");
    }
    if (!isUint32AtLeast(dacSettlingTimeArg, 1) ||
        !isUint32AtLeast(dacIntervalArg, 1) ||
        dacSettlingTimeArg >= dacIntervalArg) {
      return OperationResult::Failure("Invalid interval or settling time");
    }
    if (!isBooleanArg(retraceArg) || !isBooleanArg(snakeArg)) {
      return OperationResult::Failure("Invalid 2D scan boolean argument");
    }
    if (numAdcAverages < 1) {
      return OperationResult::Failure("Invalid number of ADC averages");
    }
    if (numStepsFast < 1 || numStepsSlow < 1) {
      return OperationResult::Failure("Invalid number of steps");
    }
    uint32_t dac_interval_us = static_cast<uint32_t>(dacIntervalArg);
    uint32_t dac_settling_time_us =
        static_cast<uint32_t>(dacSettlingTimeArg);
    bool retrace = retraceArg != 0.0f;
    bool snake = snakeArg != 0.0f;

    const size_t expected =
        currentIndex + static_cast<size_t>(numDacChannels) +
        3u * static_cast<size_t>(numDacChannels) +
        static_cast<size_t>(numAdcChannels);
    if (args.size() != expected) {
      return OperationResult::Failure("Incorrect number of arguments for 2D ramp");
    }

    // Parse DAC channel IDs
    int dacChannels[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      dacChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    // Parse start point
    float startPoint[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      startPoint[i] = args[currentIndex++];
    }

    // Parse fast axis vector
    float fastAxisVector[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      fastAxisVector[i] = args[currentIndex++];
    }

    // Parse slow axis vector
    float slowAxisVector[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      slowAxisVector[i] = args[currentIndex++];
    }

    // Parse ADC Channels
    if (args.size() < currentIndex + numAdcChannels) {
      return OperationResult::Failure("Not enough arguments for ADC channels");
    }

    int adcChannels[NUM_ADC_CHANNELS] = {};
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    OperationResult dacValidation =
        validateDacChannels(dacChannels, numDacChannels);
    if (!dacValidation.isSuccess()) {
      return dacValidation;
    }
    OperationResult adcValidation =
        validateAdcChannels(adcChannels, numAdcChannels);
    if (!adcValidation.isSuccess()) {
      return adcValidation;
    }

    // Check voltage bounds for all four corners of the 2D scan rectangle (both calibrated bounds AND global limits)
    for (int i = 0; i < numDacChannels; i++) {
      int ch = dacChannels[i];
      float lowerBound = max(DACController::getLowerBound(ch), DACLimits::lower_voltage_limit[ch]);
      float upperBound = min(DACController::getUpperBound(ch), DACLimits::upper_voltage_limit[ch]);
      
      // Four corners: startPoint, startPoint+fast, startPoint+slow, startPoint+fast+slow
      float corner1 = startPoint[i];
      float corner2 = startPoint[i] + fastAxisVector[i];
      float corner3 = startPoint[i] + slowAxisVector[i];
      float corner4 = startPoint[i] + fastAxisVector[i] + slowAxisVector[i];
      
      float minVoltage = min(min(corner1, corner2), min(corner3, corner4));
      float maxVoltage = max(max(corner1, corner2), max(corner3, corner4));
      
      if (minVoltage < lowerBound || maxVoltage > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " 2D scan range [" + String(minVoltage, 6) + 
                                        ", " + String(maxVoltage, 6) + 
                                        "]V exceeds bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
    }

    clearWorkerStopRequest();
    PeripheralCommsController::dataLedOn();

    uint8_t adcMask = 0u;
    God::BoardUsage boardUsage{0, std::vector<uint8_t>()};
    OperationResult prepareResult = God::prepareDacLedBufferRampHardware(
        numAdcChannels, numAdcAverages, dac_interval_us, dac_settling_time_us,
        adcChannels, adcMask, boardUsage, false);
    if (!prepareResult.isSuccess()) {
      PeripheralCommsController::dataLedOff();
      return prepareResult;
    }

    OperationResult rampResult = runPreparedDacLedBufferRamp2D(
        numDacChannels, numAdcChannels, numStepsFast, numStepsSlow, retrace,
        snake, numAdcAverages, dac_interval_us, dac_settling_time_us,
        dacChannels, startPoint, fastAxisVector, slowAxisVector, adcChannels,
        adcMask);

    God::cleanupDacLedBufferRampHardware(numAdcChannels, adcChannels,
                                         boardUsage);

    PeripheralCommsController::dataLedOff();

    if (!rampResult.isSuccess()) {
      if (isWorkerStopRequested()) {
        clearWorkerStopRequest();
      }
      return rampResult;
    }

    if (isWorkerStopRequested()) {
      clearWorkerStopRequest();
      return OperationResult::Failure("2D RAMPING_STOPPED");
    }

    return finishRampTimingWatchdog();
  }
