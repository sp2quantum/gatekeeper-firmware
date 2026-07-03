#include "Config.h"
#include "FunctionRegistry/FunctionRegistryArgumentParser.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Commands/BufferRamps/BufferRampCommon.h"
#include "Commands/BufferRamps/RampCommand.h"
#include "Commands/BufferRamps/RampContext.h"
#include "Commands/BufferRamps/Ramp2DCommon.h"
#include "Peripherals/ADC/ADCController.h"
#include "Peripherals/DAC/DACController.h"
#include "Utils/FastGpio.h"
#include "Utils/TimingUtil.h"
#include "shared_memory.h"

using FunctionRegistryParsing::List;

namespace {

using BufferRampCommon::dacSetWriteFailure;
using BufferRampCommon::dacWriteFailure;
using BufferRampCommon::encodeDacVoltagePackets;
using BufferRampCommon::sendVoltageFrame;
using BufferRampCommon::writeDacPackets;
using Ramp2DCommon::calculateVoltages;

OperationResult runPrepared(
    int numDacChannels, int numAdcChannels, int numStepsFast,
    int numStepsSlow, bool retrace, bool snake, uint32_t dacIntervalUs,
    uint32_t adcIntervalUs, int* dacChannels, float* startPoint,
    float* fastAxisVector, float* slowAxisVector, int* adcChannels,
    AdcBoardMask adcMask) {
  const int scansPerSlowStep = (retrace && !snake) ? 2 : 1;
  const int totalScans = numStepsSlow * scansPerSlowStep;
  const int totalDacPoints = numStepsFast * totalScans;
  const uint64_t savedFramesPerScan64 =
      (static_cast<uint64_t>(numStepsFast) * dacIntervalUs) / adcIntervalUs;
  const uint64_t savedFrames64 =
      savedFramesPerScan64 * static_cast<uint64_t>(totalScans);
  if (totalDacPoints < 1 || savedFrames64 == 0 ||
      savedFrames64 > 2147483647ULL) {
    return OperationResult::Failure("Invalid 2D time-series sample count");
  }
  const int savedFrames = static_cast<int>(savedFrames64);

  float slowAxisStep[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < numDacChannels; i++) {
    slowAxisStep[i] =
        numStepsSlow > 1 ? slowAxisVector[i] / (numStepsSlow - 1) : 0.0f;
  }

  double currentVoltages[NUM_DAC_CHANNELS] = {};
  calculateVoltages(0, numStepsFast, retrace, snake, numDacChannels,
                    startPoint, fastAxisVector, slowAxisStep,
                    currentVoltages);
  for (int i = 0; i < numDacChannels; i++) {
    if (!DACController::setVoltageNoLdac(
            dacChannels[i], currentVoltages[i])) {
      return dacWriteFailure(dacChannels[i], currentVoltages[i]);
    }
  }
  DACController::toggleLdac();

  int nextDacPointIndex = 1;
  byte nextDacPackets[NUM_DAC_CHANNELS][3] = {};
  bool nextDacPacketsReady = false;
  double nextVoltages[NUM_DAC_CHANNELS] = {};
  auto prepareNextDacPackets = [&]() {
    if (nextDacPointIndex >= totalDacPoints) {
      nextDacPacketsReady = false;
      return true;
    }
    calculateVoltages(nextDacPointIndex, numStepsFast, retrace, snake,
                      numDacChannels, startPoint, fastAxisVector, slowAxisStep,
                      nextVoltages);
    nextDacPacketsReady = encodeDacVoltagePackets(
        numDacChannels, dacChannels, nextVoltages, nextDacPackets);
    return nextDacPacketsReady;
  };
  if (!prepareNextDacPackets()) {
    return dacSetWriteFailure(numDacChannels, dacChannels, nextVoltages);
  }

  FastGpio::digitalWrite(adc_sync, true);
  TimingUtil::setupTimersTimeSeriesSampled(dacIntervalUs, adcIntervalUs);
  TimingUtil::dacFlag = false;
  TimingUtil::dacFlagCount = 0;
  TimingUtil::adcFlag = 0;

  int framesCaptured = 0;
  bool voltageOverflow = false;

  while ((framesCaptured < savedFrames || nextDacPointIndex < totalDacPoints) &&
         !isWorkerStopRequested()) {
    bool didWork = false;
    const bool adcPending =
        framesCaptured < savedFrames && TimingUtil::consumeAdcSampleFlag();

    double packets[NUM_ADC_CHANNELS] = {};
    bool haveAdcPackets = false;
    if (adcPending) {
      for (int i = 0; i < numAdcChannels; i++) {
        packets[i] =
            ADCController::getVoltageData(adcChannels[i]);
      }
      haveAdcPackets = true;
      didWork = true;
    }

    while (nextDacPointIndex < totalDacPoints &&
           TimingUtil::consumeDacFlag()) {
      if (!nextDacPacketsReady ||
          !writeDacPackets(numDacChannels, dacChannels, nextDacPackets)) {
        TimingUtil::stopTimeSeriesTimers();
        return dacSetWriteFailure(numDacChannels, dacChannels, nextVoltages);
      }
      nextDacPointIndex++;
      if (!prepareNextDacPackets()) {
        TimingUtil::stopTimeSeriesTimers();
        return dacSetWriteFailure(numDacChannels, dacChannels, nextVoltages);
      }
      didWork = true;
    }

    if (haveAdcPackets) {
      if (!sendVoltageFrame(packets, numAdcChannels)) {
        voltageOverflow = true;
        break;
      }
      framesCaptured++;
    }

    if (!didWork) {
      __WFE();
    }
  }

  TimingUtil::stopTimeSeriesTimers();

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

OperationResult timeSeriesBufferRamp2DImpl(
    int numDacChannels, int numAdcChannels, int numStepsFast,
    int numStepsSlow, float dacIntervalArg, float adcIntervalArg,
    float retraceArg, float snakeArg, List<int, 0>& dacChannelsList,
    List<float, 0>& startPointList, List<float, 0>& fastAxisVectorList,
    List<float, 0>& slowAxisVectorList, List<int, 1>& adcChannelsList,
    bool enforceTiming) {
  int* dacChannels = dacChannelsList.data();
  float* startPoint = startPointList.data();
  float* fastAxisVector = fastAxisVectorList.data();
  float* slowAxisVector = slowAxisVectorList.data();
  int* adcChannels = adcChannelsList.data();

  OperationResult requestValidation = Ramp2DCommon::validateRequest(
      numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
      dacIntervalArg, adcIntervalArg, retraceArg, snakeArg, dacChannels,
      adcChannels, startPoint, fastAxisVector, slowAxisVector,
      RampCommand::DacBoundsMode::Calibrated);
  if (!requestValidation.isSuccess()) return requestValidation;

  const uint32_t dacIntervalUs = static_cast<uint32_t>(dacIntervalArg);
  const uint32_t adcIntervalUs = static_cast<uint32_t>(adcIntervalArg);
  const bool retrace = retraceArg != 0.0f;
  const bool snake = snakeArg != 0.0f;
  if (enforceTiming) {
    const BufferRampCommon::TimeSeriesTimingMode timingMode =
        snake ? BufferRampCommon::TimeSeriesTimingMode::TwoDSnake
              : (retrace ? BufferRampCommon::TimeSeriesTimingMode::TwoDRetrace
                         : BufferRampCommon::TimeSeriesTimingMode::TwoDNormal);
    OperationResult minimumTimingValidation =
        BufferRampCommon::validateTimeSeriesTiming(
            adcIntervalArg, adcChannels, numAdcChannels, timingMode);
    if (!minimumTimingValidation.isSuccess()) {
      return minimumTimingValidation;
    }
  }
  RampContext ctx;
  OperationResult setupResult =
      ctx.beginDacAndAdc(adcChannels, numAdcChannels);
  if (!setupResult.isSuccess()) return setupResult;

  OperationResult rampResult = runPrepared(
      numDacChannels, numAdcChannels, numStepsFast, numStepsSlow, retrace,
      snake, dacIntervalUs, adcIntervalUs, dacChannels, startPoint,
      fastAxisVector, slowAxisVector, adcChannels, ctx.adcMask());

  return ctx.finish(rampResult, true, false);
}

OperationResult timeSeriesBufferRamp2D(
    int numDacChannels, int numAdcChannels, int numStepsFast,
    int numStepsSlow, float dacIntervalArg, float adcIntervalArg,
    float retraceArg, float snakeArg, List<int, 0>& dacChannelsList,
    List<float, 0>& startPointList, List<float, 0>& fastAxisVectorList,
    List<float, 0>& slowAxisVectorList, List<int, 1>& adcChannelsList) {
  return timeSeriesBufferRamp2DImpl(
      numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
      dacIntervalArg, adcIntervalArg, retraceArg, snakeArg, dacChannelsList,
      startPointList, fastAxisVectorList, slowAxisVectorList, adcChannelsList,
      true);
}
COMMAND("2D_TIME_SERIES_BUFFER_RAMP", timeSeriesBufferRamp2D)

OperationResult timeSeriesBufferRamp2DSudo(
    int numDacChannels, int numAdcChannels, int numStepsFast,
    int numStepsSlow, float dacIntervalArg, float adcIntervalArg,
    float retraceArg, float snakeArg, List<int, 0>& dacChannelsList,
    List<float, 0>& startPointList, List<float, 0>& fastAxisVectorList,
    List<float, 0>& slowAxisVectorList, List<int, 1>& adcChannelsList) {
  return timeSeriesBufferRamp2DImpl(
      numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
      dacIntervalArg, adcIntervalArg, retraceArg, snakeArg, dacChannelsList,
      startPointList, fastAxisVectorList, slowAxisVectorList, adcChannelsList,
      false);
}
COMMAND("2D_TIME_SERIES_BUFFER_RAMP_SUDO", timeSeriesBufferRamp2DSudo)

}  // namespace
