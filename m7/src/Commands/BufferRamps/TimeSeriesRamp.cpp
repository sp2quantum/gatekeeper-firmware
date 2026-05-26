#include "Config.h"
#include "FunctionRegistry/FunctionRegistryArgumentParser.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Peripherals/ADC/ADCController.h"
#include "Commands/BufferRamps/BufferRampCommon.h"
#include "Peripherals/DAC/DACController.h"
#include "Commands/BufferRamps/RampCommand.h"
#include "Commands/BufferRamps/RampContext.h"
#include "Utils/FastGpio.h"
#include "Utils/TimingUtil.h"
#include "shared_memory.h"

using FunctionRegistryParsing::List;

namespace TimeSeriesRamp {

OperationResult runPrepared(int numDacChannels, int numAdcChannels,
                            int numSteps, uint32_t dac_interval_us,
                            uint32_t adc_interval_us, int* dacChannels,
                            float* dacV0s, float* dacVfs, int* adcChannels,
                            uint8_t adcMask);

namespace {

using BufferRampCommon::dacWriteFailure;
using BufferRampCommon::encodeDacVoltagePackets;
using BufferRampCommon::isValidAdcChannelCount;
using BufferRampCommon::isValidDacChannelCount;
using BufferRampCommon::sendVoltageFrame;
using BufferRampCommon::validateRampChannels;
using BufferRampCommon::writeDacPackets;

OperationResult timeSeriesBufferRamp(
    int numDacChannels, int numAdcChannels, int numSteps,
    float dacIntervalArg, float adcIntervalArg,
    List<int, 0>& dacChannelsList, List<float, 0>& dacV0sList,
    List<float, 0>& dacVfsList, List<int, 1>& adcChannelsList) {
  if (!isValidDacChannelCount(numDacChannels) ||
      !isValidAdcChannelCount(numAdcChannels)) {
    return OperationResult::Failure("Invalid number of channels");
  }
  if (!BufferRampCommon::isUint32AtLeast(adcIntervalArg, 1) ||
      !BufferRampCommon::isUint32AtLeast(dacIntervalArg, 1)) {
    return OperationResult::Failure("Invalid interval");
  }
  if (numSteps < 1) {
    return OperationResult::Failure("Invalid number of steps");
  }

  int* dacChannels = dacChannelsList.data();
  float* dacV0s = dacV0sList.data();
  float* dacVfs = dacVfsList.data();
  int* adcChannels = adcChannelsList.data();

  OperationResult channelValidation = validateRampChannels(
      dacChannels, numDacChannels, adcChannels, numAdcChannels);
  if (!channelValidation.isSuccess()) return channelValidation;
  OperationResult endpointValidation = RampCommand::validateDacEndpoints(
      numDacChannels, dacChannels, dacV0s, dacVfs);
  if (!endpointValidation.isSuccess()) return endpointValidation;

  RampContext ctx;
  OperationResult setupResult =
      ctx.beginDacAndAdc(adcChannels, numAdcChannels);
  if (!setupResult.isSuccess()) return setupResult;

  OperationResult rampResult = runPrepared(
      numDacChannels, numAdcChannels, numSteps,
      static_cast<uint32_t>(dacIntervalArg),
      static_cast<uint32_t>(adcIntervalArg), dacChannels, dacV0s, dacVfs,
      adcChannels, ctx.adcMask());

  return ctx.finish(rampResult, true, false);
}
COMMAND("TIME_SERIES_BUFFER_RAMP", timeSeriesBufferRamp)

}  // namespace

OperationResult runPrepared(int numDacChannels, int numAdcChannels,
                            int numSteps, uint32_t dac_interval_us,
                            uint32_t adc_interval_us, int* dacChannels,
                            float* dacV0s, float* dacVfs, int* adcChannels,
                            uint8_t adcMask) {
  int dacStepsLoaded = 0;
  int framesCaptured = 0;
  const uint64_t savedDataSize64 =
      (static_cast<uint64_t>(numSteps) * dac_interval_us) / adc_interval_us;
  if (savedDataSize64 == 0 || savedDataSize64 > 2147483647ULL) {
    return OperationResult::Failure("Invalid time-series sample count");
  }
  const int savedDataSize = static_cast<int>(savedDataSize64);
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
    if (dacStepsLoaded >= numSteps) {
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
      return dacWriteFailure(dacChannels[i], dacV0s[i]);
    }
    nextVoltageSet[i] += voltageStepSize[i];
  }
  DACController::toggleLdac();
  dacStepsLoaded++;
  if (!prepareNextDacPackets()) {
    return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
  }

  FastGpio::digitalWrite(adc_sync, false);
  for (int i = 0; i < numAdcChannels; i++) {
    ADCController::startContinuousConversion(adcChannels[i]);
    ADCController::setRDYFN(adcChannels[i]);
  }
  TimingUtil::setupTimersTimeSeriesRamp(dac_interval_us, adc_interval_us,
                                        adcMask);
  TimingUtil::dacFlag = false;
  TimingUtil::adcFlag = 0;
  bool dacTimerPending = false;

  while ((framesCaptured < savedDataSize || dacStepsLoaded < numSteps) &&
         !isWorkerStopRequested()) {
    __WFE();
    const bool adcPending = framesCaptured < savedDataSize &&
                            TimingUtil::consumeAdcFlag(adcMask);
    if (dacStepsLoaded < numSteps && TimingUtil::consumeDacFlag()) {
      dacTimerPending = true;
    }

    double packets[NUM_ADC_CHANNELS] = {};
    bool haveAdcPackets = false;
    if (adcPending) {
      for (int i = 0; i < numAdcChannels; i++) {
        packets[i] =
            ADCController::getVoltageDataNoTransaction(adcChannels[i]);
      }
      FastGpio::digitalWrite(adc_sync, false);
      haveAdcPackets = true;
    }
    if (dacTimerPending && TimingUtil::adcConversionInProgressMask == 0) {
      if (!nextDacPacketsReady ||
          !writeDacPackets(numDacChannels, dacChannels, nextDacPackets)) {
        TimingUtil::stopTimeSeriesTimers();
        return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
      } else {
        dacTimerPending = false;
        for (int i = 0; i < numDacChannels; i++) {
          nextVoltageSet[i] += voltageStepSize[i];
        }
        dacStepsLoaded++;
        if (!prepareNextDacPackets()) {
          TimingUtil::stopTimeSeriesTimers();
          return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
        }
      }
    }
    if (haveAdcPackets) {
      if (!sendVoltageFrame(packets, numAdcChannels)) {
        voltageOverflow = true;
        break;
      }
      framesCaptured++;
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

}  // namespace TimeSeriesRamp
