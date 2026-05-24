#include "Peripherals/BufferRamp2D.h"

#include "Config.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Peripherals/ADC/ADCController.h"
#include "Peripherals/BufferRamp.h"
#include "Peripherals/BufferRampCommon.h"
#include "Peripherals/DAC/DACController.h"
#include "Peripherals/PeripheralCommsController.h"
#include "Peripherals/RampCommand.h"
#include "Utils/FastGpio.h"
#include "Utils/TimingUtil.h"
#include "Utils/shared_memory.h"

namespace {
using BufferRampCommon::dacWriteFailure;
using BufferRampCommon::encodeDacVoltagePackets;
using BufferRampCommon::finishRampTimingWatchdog;
using BufferRampCommon::sendVoltageFrame;
using BufferRampCommon::validateRampChannels;
using BufferRampCommon::writeDacPackets;
using RampCommand::DacBoundsMode;
using RampCommand::validateDac2DScanBounds;

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

OperationResult validateScan2DRequest(
    int numDacChannels, int numAdcChannels, int numStepsFast,
    int numStepsSlow, float dacIntervalArg, float adcTimingArg,
    float retraceArg, float snakeArg, const int* dacChannels,
    const int* adcChannels, const float* startPoint,
    const float* fastAxisVector, const float* slowAxisVector,
    DacBoundsMode boundsMode) {
  if (!BufferRampCommon::isValidDacChannelCount(numDacChannels) ||
      !BufferRampCommon::isValidAdcChannelCount(numAdcChannels)) {
    return OperationResult::Failure("Invalid number of channels");
  }
  if (!BufferRampCommon::isUint32AtLeast(adcTimingArg, 1) ||
      !BufferRampCommon::isUint32AtLeast(dacIntervalArg, 1)) {
    return OperationResult::Failure("Invalid interval");
  }
  if (!RampCommand::isBooleanArg(retraceArg) ||
      !RampCommand::isBooleanArg(snakeArg)) {
    return OperationResult::Failure("Invalid 2D scan boolean argument");
  }
  if (numStepsFast < 1 || numStepsSlow < 1) {
    return OperationResult::Failure("Invalid number of steps");
  }

  OperationResult channelValidation = validateRampChannels(
      dacChannels, numDacChannels, adcChannels, numAdcChannels, false, false);
  if (!channelValidation.isSuccess()) {
    return channelValidation;
  }
  return validateDac2DScanBounds(numDacChannels, dacChannels, startPoint,
                                 fastAxisVector, slowAxisVector, boundsMode);
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
  bool dacTimerPending = false;
  bool voltageOverflow = false;

  while (adcReads < totalPoints && !isWorkerStopRequested()) {
    __WFE();

    if (dacStreamPointsPreloaded < totalStreamPoints &&
        TimingUtil::consumeDacFlag()) {
      dacTimerPending = true;
    }
    const bool adcConversionStarted =
        TimingUtil::consumeAdcConversionStartedFlag();
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

    if (dacTimerPending && adcConversionStarted) {
      if (!nextDacPacketsReady ||
          !writeDacPackets(numDacChannels, dacChannels, nextDacPackets)) {
        return dacWriteFailure(dacChannels[0], nextVoltages[0]);
      }
      dacTimerPending = false;
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

}

void BufferRamp2D::setup() { initializeRegistry(); }



void BufferRamp2D::initializeRegistry() {
  registerFunction(timeSeriesBufferRamp2D, "2D_TIME_SERIES_BUFFER_RAMP");
  registerFunction(dacLedBufferRamp2D, "2D_DAC_LED_BUFFER_RAMP");
}

OperationResult BufferRamp2D::timeSeriesBufferRamp2D(
    int numDacChannels, int numAdcChannels, int numStepsFast,
    int numStepsSlow, float dacIntervalArg, float adcIntervalArg,
    float retraceArg, float snakeArg,
    List<int, 0>& dacChannelsList,
    List<float, 0>& startPointList,
    List<float, 0>& fastAxisVectorList,
    List<float, 0>& slowAxisVectorList,
    List<int, 1>& adcChannelsList) {
  int* dacChannels = dacChannelsList.data();
  float* startPoint = startPointList.data();
  float* fastAxisVector = fastAxisVectorList.data();
  float* slowAxisVector = slowAxisVectorList.data();
  int* adcChannels = adcChannelsList.data();

  OperationResult requestValidation = validateScan2DRequest(
      numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
      dacIntervalArg, adcIntervalArg, retraceArg, snakeArg, dacChannels,
      adcChannels, startPoint, fastAxisVector, slowAxisVector,
      DacBoundsMode::Calibrated);
  if (!requestValidation.isSuccess()) {
    return requestValidation;
  }

  const uint32_t dacIntervalUs = static_cast<uint32_t>(dacIntervalArg);
  const uint32_t adcIntervalUs = static_cast<uint32_t>(adcIntervalArg);
  const bool retrace = retraceArg != 0.0f;
  const bool snake = snakeArg != 0.0f;

  float slowStepSize[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < numDacChannels; i++) {
    slowStepSize[i] =
        numStepsSlow > 1
            ? slowAxisVector[i] / (numStepsSlow - 1)
            : 0.0f;
  }

  float currentSlowPosition[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < numDacChannels; i++) {
    currentSlowPosition[i] = startPoint[i];
  }

  clearWorkerStopRequest();
  PeripheralCommsController::dataLedOn();

  uint8_t adcMask = 0u;
  BufferRamp::BoardUsage boardUsage{0, std::vector<uint8_t>()};
  OperationResult prepareResult =
      BufferRamp::prepareTimeSeriesBufferRampHardware(
          numAdcChannels, adcChannels, adcMask, boardUsage);
  if (!prepareResult.isSuccess()) {
    PeripheralCommsController::dataLedOff();
    return prepareResult;
  }

  OperationResult rampResult = OperationResult::Success();

  for (int slowStep = 0;
       slowStep < numStepsSlow && !isWorkerStopRequested();
       ++slowStep) {
    const bool reverseFastAxis = snake && ((slowStep % 2) != 0);

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

    OperationResult ramp1Result = BufferRamp::runPreparedTimeSeriesBufferRamp(
        numDacChannels, numAdcChannels, numStepsFast,
        dacIntervalUs, adcIntervalUs, dacChannels,
        fastV0s, fastVfs, adcChannels, adcMask,
        BufferRamp::TimeSeriesRampMode::Buffered2DRow);

    OperationResult ramp2Result = OperationResult::Success();
    if (retrace && !snake) {
      ramp2Result = BufferRamp::runPreparedTimeSeriesBufferRamp(
          numDacChannels, numAdcChannels, numStepsFast,
          dacIntervalUs, adcIntervalUs, dacChannels, fastVfs, fastV0s,
          adcChannels, adcMask,
          BufferRamp::TimeSeriesRampMode::Buffered2DRow);
    }

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

    for (int i = 0; i < numDacChannels; ++i) {
      currentSlowPosition[i] += slowStepSize[i];
    }
  }

  BufferRamp::cleanupTimeSeriesBufferRampHardware(
      numAdcChannels, adcChannels, boardUsage);

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
OperationResult BufferRamp2D::dacLedBufferRamp2D(
    int numDacChannels, int numAdcChannels, int numStepsFast,
    int numStepsSlow, float dacIntervalArg, float dacSettlingTimeArg,
    float retraceArg, float snakeArg, int numAdcAverages,
    List<int, 0>& dacChannelsList,
    List<float, 0>& startPointList,
    List<float, 0>& fastAxisVectorList,
    List<float, 0>& slowAxisVectorList,
    List<int, 1>& adcChannelsList) {
  if (numAdcAverages < 1) {
    return OperationResult::Failure("Invalid number of ADC averages");
  }

  int* dacChannels = dacChannelsList.data();
  float* startPoint = startPointList.data();
  float* fastAxisVector = fastAxisVectorList.data();
  float* slowAxisVector = slowAxisVectorList.data();
  int* adcChannels = adcChannelsList.data();

  OperationResult requestValidation = validateScan2DRequest(
      numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
      dacIntervalArg, dacSettlingTimeArg, retraceArg, snakeArg, dacChannels,
      adcChannels, startPoint, fastAxisVector, slowAxisVector,
      DacBoundsMode::CalibratedAndGlobal);
  if (!requestValidation.isSuccess()) {
    return requestValidation;
  }
  if (dacSettlingTimeArg >= dacIntervalArg) {
    return OperationResult::Failure("Invalid interval or settling time");
  }

  const uint32_t dacIntervalUs = static_cast<uint32_t>(dacIntervalArg);
  const uint32_t dacSettlingTimeUs =
      static_cast<uint32_t>(dacSettlingTimeArg);
  const bool retrace = retraceArg != 0.0f;
  const bool snake = snakeArg != 0.0f;

  clearWorkerStopRequest();
  PeripheralCommsController::dataLedOn();

  uint8_t adcMask = 0u;
  BufferRamp::BoardUsage boardUsage{0, std::vector<uint8_t>()};
  OperationResult prepareResult = BufferRamp::prepareDacLedBufferRampHardware(
      numAdcChannels, adcChannels, adcMask, boardUsage);
  if (!prepareResult.isSuccess()) {
    PeripheralCommsController::dataLedOff();
    return prepareResult;
  }

  OperationResult rampResult = runPreparedDacLedBufferRamp2D(
      numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
      retrace, snake, numAdcAverages, dacIntervalUs, dacSettlingTimeUs,
      dacChannels, startPoint, fastAxisVector, slowAxisVector, adcChannels,
      adcMask);

  BufferRamp::cleanupDacLedBufferRampHardware(
      numAdcChannels, adcChannels, boardUsage);

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
