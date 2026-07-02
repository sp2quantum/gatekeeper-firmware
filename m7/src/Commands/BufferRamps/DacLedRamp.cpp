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

namespace DacLedRamp {

OperationResult validateAdcConversionTimes(int numAdcChannels,
                                           const int* adcChannels);

namespace {

using BufferRampCommon::dacWriteFailure;
using BufferRampCommon::encodeDacVoltagePackets;
using BufferRampCommon::isValidAdcChannelCount;
using BufferRampCommon::isValidDacChannelCount;
using BufferRampCommon::sendVoltageFrame;
using BufferRampCommon::validateRampChannels;
using BufferRampCommon::writeDacPackets;

OperationResult runPrepared(int numDacChannels, int numAdcChannels,
                            int numSteps, int numAdcAverages,
                            int* dacChannels, float* dacV0s, float* dacVfs,
                            int* adcChannels, AdcBoardMask adcMask) {
  double packets[NUM_ADC_CHANNELS] = {};
  double voltageStepSize[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < numDacChannels; i++) {
    voltageStepSize[i] =
        numSteps > 1 ? (dacVfs[i] - dacV0s[i]) / (numSteps - 1) : 0.0;
  }

  double numAdcAveragesInv = 1.0 / static_cast<double>(numAdcAverages);
  int dacStepsLoaded = 0;
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
  dacStepsLoaded++;
  if (!prepareNextDacPackets()) {
    return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
  }
  FastGpio::digitalWrite(adc_sync, false);
  int adcFramesRead = 0;
  bool dacTimerPending = false;
  bool voltageOverflow = false;

  while (adcFramesRead < numSteps && !isWorkerStopRequested()) {
    __WFE();

    if (dacStepsLoaded < numSteps && TimingUtil::consumeDacFlag()) {
      dacTimerPending = true;
    }
    const bool adcConversionStarted =
        TimingUtil::consumeAdcConversionStartedFlag();
    const bool adcPending = TimingUtil::consumeAdcFlag(adcMask);

    bool haveAdcPackets = false;
    if (adcPending) {
      adcFramesRead++;
      for (int i = 0; i < numAdcChannels; i++) {
        double total = 0.0;
        for (int j = 0; j < numAdcAverages; j++) {
          total += ADCController::getVoltageDataNoTransaction(adcChannels[i]);
        }
        packets[i] = total * numAdcAveragesInv;
      }
      FastGpio::digitalWrite(adc_sync, false);
      haveAdcPackets = true;
    }

    if (dacTimerPending && adcConversionStarted) {
      if (!nextDacPacketsReady ||
          !writeDacPackets(numDacChannels, dacChannels, nextDacPackets)) {
        return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
      }
      dacTimerPending = false;
      for (int i = 0; i < numDacChannels; i++) {
        nextVoltageSet[i] += voltageStepSize[i];
      }
      dacStepsLoaded++;
      if (!prepareNextDacPackets()) {
        return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
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

OperationResult dacLedBufferRamp(
    int numDacChannels, int numAdcChannels, int numSteps, int numAdcAverages,
    float dacIntervalArg, float dacSettlingTimeArg,
    List<int, 0>& dacChannelsList, List<float, 0>& dacV0sList,
    List<float, 0>& dacVfsList, List<int, 1>& adcChannelsList) {
  if (!isValidDacChannelCount(numDacChannels) ||
      !isValidAdcChannelCount(numAdcChannels)) {
    return OperationResult::Failure("Invalid number of channels");
  }
  if (!BufferRampCommon::isUint32AtLeast(dacSettlingTimeArg, 1) ||
      !BufferRampCommon::isUint32AtLeast(dacIntervalArg, 1) ||
      dacSettlingTimeArg >= dacIntervalArg) {
    return OperationResult::Failure("Invalid interval or settling time");
  }
  if (numAdcAverages < 1) {
    return OperationResult::Failure("Invalid number of ADC averages");
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
  OperationResult minimumTimingValidation =
      BufferRampCommon::validateDacLedTiming(
          dacIntervalArg, dacSettlingTimeArg, adcChannels, numAdcChannels);
  if (!minimumTimingValidation.isSuccess()) {
    return minimumTimingValidation;
  }

  ADCController::resetToPreviousConversionTimes();
  OperationResult timingValidation =
      DacLedRamp::validateAdcConversionTimes(numAdcChannels, adcChannels);
  if (!timingValidation.isSuccess()) return timingValidation;

  RampContext ctx;
  OperationResult setupResult =
      ctx.beginDacAndAdc(adcChannels, numAdcChannels);
  if (!setupResult.isSuccess()) return setupResult;

  const uint32_t dacIntervalUs = static_cast<uint32_t>(dacIntervalArg);
  const uint32_t dacSettlingTimeUs =
      static_cast<uint32_t>(dacSettlingTimeArg);

  TimingUtil::setupTimersDacLed(dacIntervalUs, dacSettlingTimeUs,
                                ctx.adcMask());
  TimingUtil::dacFlag = false;
  TimingUtil::dacFlagCount = 0;
  TimingUtil::adcFlag = 0;

  OperationResult rampResult =
      runPrepared(numDacChannels, numAdcChannels, numSteps, numAdcAverages,
                  dacChannels, dacV0s, dacVfs, adcChannels, ctx.adcMask());

  return ctx.finish(rampResult);
}
COMMAND("DAC_LED_BUFFER_RAMP", dacLedBufferRamp)

}  // namespace

OperationResult validateAdcConversionTimes(int numAdcChannels,
                                           const int* adcChannels) {
  bool selectedAdcChannels[NUM_ADC_CHANNELS] = {};
  uint8_t boardDepth[NUM_ADC_BOARDS] = {};

  for (int i = 0; i < numAdcChannels; i++) {
    const int channel = adcChannels[i];
    if (channel < 0 || channel >= NUM_ADC_CHANNELS ||
        selectedAdcChannels[channel]) {
      continue;
    }
    selectedAdcChannels[channel] = true;
    boardDepth[BufferRampCommon::adcBoardForChannel(channel)]++;
  }

  for (int channel = 0; channel < NUM_ADC_CHANNELS; channel++) {
    if (!selectedAdcChannels[channel]) continue;
    const uint8_t board = BufferRampCommon::adcBoardForChannel(channel);
    const bool multiChannelScan = boardDepth[board] > 1;
    const float conversionTimeUs =
        ADCController::getConversionTimeFloat(channel, multiChannelScan);
    if (conversionTimeUs < 0.0f) {
      return OperationResult::Failure("Invalid ADC conversion time");
    }
  }

  return OperationResult::Success();
}

}  // namespace DacLedRamp
