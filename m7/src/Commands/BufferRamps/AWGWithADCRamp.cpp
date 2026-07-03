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

namespace {

using BufferRampCommon::isValidAdcChannelCount;
using BufferRampCommon::isValidDacChannelCount;
using BufferRampCommon::sendVoltageFrame;
using BufferRampCommon::validateRampChannels;
using RampCommand::validateDacVoltageListBounds;

OperationResult awgWithAdcImpl(
    int numDacChannels, int numAdcChannels, int numSteps,
    float dacIntervalArg, int numCycles, List<int, 0>& dacChannelsList,
    List<int, 1>& adcChannelsList,
    List<float, 0, 2>& channelMajorVoltages, bool enforceTiming) {
  if (!BufferRampCommon::isUint32AtLeast(dacIntervalArg, 1) ||
      numCycles < 1) {
    return OperationResult::Failure(
        "Invalid channel counts or step/cycle count");
  }

  int* dacChannels = dacChannelsList.data();
  int* adcChannels = adcChannelsList.data();
  const uint32_t dac_interval_us = static_cast<uint32_t>(dacIntervalArg);

  if (dac_interval_us < 1) {
    return OperationResult::Failure("Invalid dac interval");
  }
  if (!isValidDacChannelCount(numDacChannels) ||
      !isValidAdcChannelCount(numAdcChannels) || numSteps < 1 ||
      numCycles < 1) {
    return OperationResult::Failure(
        "Invalid channel counts or step/cycle count");
  }
  OperationResult channelValidation = validateRampChannels(
      dacChannels, numDacChannels, adcChannels, numAdcChannels);
  if (!channelValidation.isSuccess()) return channelValidation;
  if (enforceTiming) {
    OperationResult minimumTimingValidation =
        BufferRampCommon::validateAwgWithAdcTiming(
            dacIntervalArg, numDacChannels, adcChannels, numAdcChannels);
    if (!minimumTimingValidation.isSuccess()) {
      return minimumTimingValidation;
    }
  }

  OperationResult waveformBounds = validateDacVoltageListBounds(
      numDacChannels, numSteps, dacChannels, channelMajorVoltages.data());
  if (!waveformBounds.isSuccess()) return waveformBounds;

  RampContext ctx;
  OperationResult beginResult = ctx.beginDacAndAdc(adcChannels, numAdcChannels);
  if (!beginResult.isSuccess()) return beginResult;

  double packets[NUM_ADC_CHANNELS] = {};
  bool voltageOverflow = false;

  for (int i = 0; i < numDacChannels; i++) {
    const float v0 = channelMajorVoltages[static_cast<size_t>(i) *
                                          static_cast<size_t>(numSteps)];
    DACController::setVoltageNoTransactionNoLdac(dacChannels[i], v0);
  }

  TimingUtil::setupTimerOnlyDac(dac_interval_us);
  TimingUtil::dacFlag = false;
  TimingUtil::dacFlagCount = 0;
  TimingUtil::adcFlag = 0;

  const int totalSteps = numSteps * numCycles;
  int stepsWritten = 0;
  int framesCaptured = 0;
  while ((stepsWritten < totalSteps || framesCaptured < totalSteps) &&
         !ctx.stopped()) {
    __WFE();
    if (stepsWritten < totalSteps && TimingUtil::consumeDacFlag()) {
      const int step = stepsWritten % numSteps;
      for (int i = 0; i < numDacChannels; i++) {
        const float v =
            channelMajorVoltages[static_cast<size_t>(i) *
                                     static_cast<size_t>(numSteps) +
                                 static_cast<size_t>(step)];
        DACController::setVoltageNoTransactionNoLdac(dacChannels[i], v);
      }
      FastGpio::digitalWrite(adc_sync, true);
      stepsWritten++;
    }

    if (framesCaptured < totalSteps &&
        TimingUtil::consumeAdcFlag(ctx.adcMask())) {
      FastGpio::digitalWrite(adc_sync, false);
      for (int i = 0; i < numAdcChannels; i++) {
        packets[i] = ADCController::getVoltageData(adcChannels[i]);
      }
      if (!sendVoltageFrame(packets, numAdcChannels)) {
        voltageOverflow = true;
        break;
      }
      framesCaptured++;
    }
  }

  return ctx.finish(
      voltageOverflow
          ? OperationResult::Failure("Voltage output buffer overflow")
          : OperationResult::Success());
}

OperationResult awgWithAdc(int numDacChannels, int numAdcChannels,
                           int numSteps, float dacIntervalArg, int numCycles,
                           List<int, 0>& dacChannelsList,
                           List<int, 1>& adcChannelsList,
                           List<float, 0, 2>& channelMajorVoltages) {
  return awgWithAdcImpl(numDacChannels, numAdcChannels, numSteps,
                        dacIntervalArg, numCycles, dacChannelsList,
                        adcChannelsList, channelMajorVoltages, true);
}
COMMAND("AWG_WITH_ADC", awgWithAdc)

OperationResult awgWithAdcSudo(int numDacChannels, int numAdcChannels,
                               int numSteps, float dacIntervalArg,
                               int numCycles,
                               List<int, 0>& dacChannelsList,
                               List<int, 1>& adcChannelsList,
                               List<float, 0, 2>& channelMajorVoltages) {
  return awgWithAdcImpl(numDacChannels, numAdcChannels, numSteps,
                        dacIntervalArg, numCycles, dacChannelsList,
                        adcChannelsList, channelMajorVoltages, false);
}
COMMAND("AWG_WITH_ADC_SUDO", awgWithAdcSudo)

}  // namespace
