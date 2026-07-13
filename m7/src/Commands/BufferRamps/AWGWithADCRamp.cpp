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

using BufferRampCommon::dacWriteFailure;
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
  if (!isValidDacChannelCount(numDacChannels) ||
      !isValidAdcChannelCount(numAdcChannels)) {
    return OperationResult::Failure("Invalid number of channels");
  }
  if (numSteps < 1 || numCycles < 1) {
    return OperationResult::Failure("Invalid step/cycle count");
  }
  if (!BufferRampCommon::isTimerPeriodUs(dacIntervalArg)) {
    return OperationResult::Failure("Invalid dac interval");
  }
  const uint64_t totalSteps64 =
      static_cast<uint64_t>(numSteps) * static_cast<uint64_t>(numCycles);
  if (totalSteps64 > 2147483647ULL) {
    return OperationResult::Failure("Invalid step/cycle count");
  }

  int* dacChannels = dacChannelsList.data();
  int* adcChannels = adcChannelsList.data();
  const uint32_t dac_interval_us = static_cast<uint32_t>(dacIntervalArg);

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
    if (!DACController::setVoltageNoLdac(dacChannels[i], v0)) {
      return ctx.finish(dacWriteFailure(dacChannels[i], v0));
    }
  }

  TimingUtil::setupTimerOnlyDac(dac_interval_us);
  TimingUtil::dacFlag = false;
  TimingUtil::dacFlagCount = 0;
  TimingUtil::adcFlag = 0;

  const int totalSteps = static_cast<int>(totalSteps64);
  // Step 0 is preloaded into the DAC input registers above; each timer tick
  // latches the previously written step, so the loop stays one step ahead:
  // tick k latches step k-1, starts its conversion, and queues step k.
  int stepsLatched = 0;
  int stepsWritten = 1;
  int framesCaptured = 0;
  while ((stepsLatched < totalSteps || framesCaptured < totalSteps) &&
         !ctx.stopped()) {
    __WFE();
    if (stepsLatched < totalSteps && TimingUtil::consumeDacFlag()) {
      if (stepsWritten < totalSteps) {
        const int step = stepsWritten % numSteps;
        for (int i = 0; i < numDacChannels; i++) {
          const float v =
              channelMajorVoltages[static_cast<size_t>(i) *
                                       static_cast<size_t>(numSteps) +
                                   static_cast<size_t>(step)];
          if (!DACController::setVoltageNoLdac(dacChannels[i], v)) {
            return ctx.finish(dacWriteFailure(dacChannels[i], v));
          }
        }
        stepsWritten++;
      }
      FastGpio::digitalWrite(adc_sync, true);
      stepsLatched++;
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
