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

#include "stm32h7xx.h"

using FunctionRegistryParsing::List;

namespace {

using BufferRampCommon::dacWriteFailure;
using BufferRampCommon::isValidAdcChannelCount;
using BufferRampCommon::isValidDacChannelCount;
using BufferRampCommon::sendVoltageFrame;
using BufferRampCommon::validateRampChannels;

OperationResult boxcarAverageRampImpl(
    int numDacChannels, int numAdcChannels, int numDacSteps,
    int numAdcMeasuresPerDacStep, int numAdcAverages,
    float adcConversionTimeArg, List<int, 0>& dacChannelsList,
    List<float, 0>& dacV0_1List,
    List<float, 0>& dacVf_1List, List<float, 0>& dacV0_2List,
    List<float, 0>& dacVf_2List, List<int, 1>& adcChannelsList,
    bool enforceTiming) {
  if (!isValidDacChannelCount(numDacChannels) ||
      !isValidAdcChannelCount(numAdcChannels)) {
    return OperationResult::Failure("Invalid number of channels");
  }
  if (numDacSteps < 1 || numAdcMeasuresPerDacStep < 1 ||
      numAdcAverages < 1 ||
      !BufferRampCommon::isUint32AtLeast(adcConversionTimeArg, 1)) {
    return OperationResult::Failure("Invalid boxcar timing/count argument");
  }

  int* dacChannels = dacChannelsList.data();
  float* dacV0_1 = dacV0_1List.data();
  float* dacVf_1 = dacVf_1List.data();
  float* dacV0_2 = dacV0_2List.data();
  float* dacVf_2 = dacVf_2List.data();
  int* adcChannels = adcChannelsList.data();

  OperationResult channelValidation = validateRampChannels(
      dacChannels, numDacChannels, adcChannels, numAdcChannels);
  if (!channelValidation.isSuccess()) return channelValidation;
  if (enforceTiming) {
    OperationResult minimumTimingValidation =
        BufferRampCommon::validateBoxcarTiming(
            adcConversionTimeArg, adcChannels, numAdcChannels);
    if (!minimumTimingValidation.isSuccess()) {
      return minimumTimingValidation;
    }
  }
  OperationResult endpointValidation =
      RampCommand::validateBoxcarDacEndpoints(numDacChannels, dacChannels,
                                              dacV0_1, dacVf_1, dacV0_2,
                                              dacVf_2);
  if (!endpointValidation.isSuccess()) return endpointValidation;

  const uint32_t adcConversionTime_us =
      static_cast<uint32_t>(adcConversionTimeArg);
  uint32_t actualConversionTime_us = ADCController::presetConversionTime(
      adcChannels[0], adcConversionTime_us, numAdcChannels > 1);
  for (int i = 1; i < numAdcChannels; ++i) {
    ADCController::presetConversionTime(adcChannels[i], adcConversionTime_us,
                                        numAdcChannels > 1);
  }

  uint8_t channelsPerBoard[NUM_ADC_BOARDS] = {};
  for (int i = 0; i < numAdcChannels; ++i) {
    channelsPerBoard[BufferRampCommon::adcBoardForChannel(adcChannels[i])]++;
  }
  uint8_t maxChannelsPerBoard = 0;
  for (uint8_t count : channelsPerBoard) {
    if (count > maxChannelsPerBoard) maxChannelsPerBoard = count;
  }

  constexpr uint64_t kAdcReadBudgetPerChannelUs = 15;
  constexpr uint64_t kAdcLoopBudgetUs = 15;
  const uint64_t adcFramePeriod64 =
      static_cast<uint64_t>(actualConversionTime_us) *
          static_cast<uint64_t>(maxChannelsPerBoard) +
      kAdcReadBudgetPerChannelUs * static_cast<uint64_t>(numAdcChannels) +
      kAdcLoopBudgetUs;
  if (adcFramePeriod64 > 0xFFFFFFFFULL) {
    return OperationResult::Failure("Boxcar ADC period is out of range");
  }
  const uint32_t adcFramePeriodUs =
      static_cast<uint32_t>(adcFramePeriod64);

  if (!TimingUtil::isTimerPeriodRepresentable(adcFramePeriodUs)) {
    return OperationResult::Failure("Boxcar timer period is out of range");
  }

  const uint64_t totalSteps64 = 2ULL *
                                static_cast<uint64_t>(numDacSteps) *
                                static_cast<uint64_t>(numAdcAverages);
  const uint64_t totalDataSize64 =
      totalSteps64 * static_cast<uint64_t>(numAdcMeasuresPerDacStep);
  if (totalDataSize64 > 2147483647ULL) {
    return OperationResult::Failure("Boxcar sample count is out of range");
  }

  RampContext ctx;
  OperationResult beginResult = ctx.beginDacAndAdc(adcChannels, numAdcChannels);
  if (!beginResult.isSuccess()) return beginResult;

  double voltageStepSizeLow[NUM_DAC_CHANNELS] = {};
  double voltageStepSizeHigh[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < numDacChannels; i++) {
    voltageStepSizeLow[i] =
        numDacSteps > 1
            ? (dacVf_1[i] - dacV0_1[i]) / static_cast<double>(numDacSteps - 1)
            : 0.0;
    voltageStepSizeHigh[i] =
        numDacSteps > 1
            ? (dacVf_2[i] - dacV0_2[i]) / static_cast<double>(numDacSteps - 1)
            : 0.0;
  }

  double previousVoltageSetLow[NUM_DAC_CHANNELS] = {};
  double previousVoltageSetHigh[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < numDacChannels; i++) {
    previousVoltageSetLow[i] = dacV0_1[i];
    previousVoltageSetHigh[i] = dacV0_2[i];
  }

  int latchedSteps = 0;
  const int totalSteps = static_cast<int>(totalSteps64);
  int x = 0;
  const int total_data_size = static_cast<int>(totalDataSize64);
  bool voltageOverflow = false;

  // Step 0 always outputs the low set.
  for (int i = 0; i < numDacChannels; i++) {
    if (!DACController::setVoltageNoLdac(dacChannels[i],
                                         previousVoltageSetLow[i])) {
      return ctx.finish(
          dacWriteFailure(dacChannels[i], previousVoltageSetLow[i]));
    }
  }
  DACController::toggleLdac();
  latchedSteps++;

  int failedDacChannel = -1;
  double failedDacVoltage = 0.0;
  auto queueStep = [&](int step) {
    for (int i = 0; i < numDacChannels; i++) {
      double currentVoltage;
      if (step % (2 * numAdcAverages) != 0) {
        currentVoltage = step % 2 == 0 ? previousVoltageSetLow[i]
                                       : previousVoltageSetHigh[i];
      } else {
        previousVoltageSetLow[i] += voltageStepSizeLow[i];
        previousVoltageSetHigh[i] += voltageStepSizeHigh[i];
        currentVoltage = step % 2 == 0 ? previousVoltageSetLow[i]
                                       : previousVoltageSetHigh[i];
      }
      if (!DACController::setVoltageNoLdac(dacChannels[i], currentVoltage)) {
        failedDacChannel = dacChannels[i];
        failedDacVoltage = currentVoltage;
        return false;
      }
    }
    return true;
  };

  int nextStepToQueue = 1;
  if (nextStepToQueue < totalSteps) {
    if (!queueStep(nextStepToQueue)) {
      return ctx.finish(
          dacWriteFailure(failedDacChannel, failedDacVoltage));
    }
    nextStepToQueue++;
  }

  TimingUtil::setupTimersOnlyADC(adcFramePeriodUs, ctx.adcMask());
  int samplesAtCurrentStep = 0;

  while (x < total_data_size && !ctx.stopped()) {
    if (TimingUtil::consumeAdcFlag(ctx.adcMask())) {
      FastGpio::digitalWrite(adc_sync, false);
      double packets[NUM_ADC_CHANNELS] = {};
      for (int i = 0; i < numAdcChannels; i++) {
        packets[i] =
            ADCController::getVoltageData(adcChannels[i]);
      }
      if (!sendVoltageFrame(packets, numAdcChannels)) {
        voltageOverflow = true;
        break;
      }
      x++;
      samplesAtCurrentStep++;

      if (samplesAtCurrentStep == numAdcMeasuresPerDacStep &&
          latchedSteps < totalSteps) {
        DACController::toggleLdac();
        latchedSteps++;
        samplesAtCurrentStep = 0;
        TIM8->CNT = 0;

        if (nextStepToQueue < totalSteps) {
          if (!queueStep(nextStepToQueue)) {
            return ctx.finish(
                dacWriteFailure(failedDacChannel, failedDacVoltage));
          }
          nextStepToQueue++;
        }
      }
    }
  }

  return ctx.finish(
      voltageOverflow
          ? OperationResult::Failure("Voltage output buffer overflow")
          : OperationResult::Success(),
      true, false);
}

OperationResult boxcarAverageRamp(
    int numDacChannels, int numAdcChannels, int numDacSteps,
    int numAdcMeasuresPerDacStep, int numAdcAverages,
    float adcConversionTimeArg, List<int, 0>& dacChannelsList,
    List<float, 0>& dacV0_1List,
    List<float, 0>& dacVf_1List, List<float, 0>& dacV0_2List,
    List<float, 0>& dacVf_2List, List<int, 1>& adcChannelsList) {
  return boxcarAverageRampImpl(
      numDacChannels, numAdcChannels, numDacSteps,
      numAdcMeasuresPerDacStep, numAdcAverages, adcConversionTimeArg,
      dacChannelsList, dacV0_1List, dacVf_1List, dacV0_2List, dacVf_2List,
      adcChannelsList, true);
}
COMMAND("BOXCAR_BUFFER_RAMP", boxcarAverageRamp)

OperationResult boxcarAverageRampSudo(
    int numDacChannels, int numAdcChannels, int numDacSteps,
    int numAdcMeasuresPerDacStep, int numAdcAverages,
    float adcConversionTimeArg, List<int, 0>& dacChannelsList,
    List<float, 0>& dacV0_1List,
    List<float, 0>& dacVf_1List, List<float, 0>& dacV0_2List,
    List<float, 0>& dacVf_2List, List<int, 1>& adcChannelsList) {
  return boxcarAverageRampImpl(
      numDacChannels, numAdcChannels, numDacSteps,
      numAdcMeasuresPerDacStep, numAdcAverages, adcConversionTimeArg,
      dacChannelsList, dacV0_1List, dacVf_1List, dacV0_2List, dacVf_2List,
      adcChannelsList, false);
}
COMMAND("BOXCAR_BUFFER_RAMP_SUDO", boxcarAverageRampSudo)

}  // namespace
