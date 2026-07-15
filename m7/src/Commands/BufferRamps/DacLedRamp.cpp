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

using BufferRampCommon::dacSetWriteFailure;
using BufferRampCommon::dacWriteFailure;
using BufferRampCommon::encodeDacVoltagePackets;
using BufferRampCommon::isValidAdcChannelCount;
using BufferRampCommon::isValidDacChannelCount;
using BufferRampCommon::sendVoltageFrame;
using BufferRampCommon::validateRampChannels;
using BufferRampCommon::writeDacPackets;

OperationResult runPrepared(int numDacChannels, int numAdcChannels,
                            int numSteps, int numAdcAverages,
                            uint32_t dac_interval_us,
                            uint32_t dac_settling_time_us,
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
    if (!DACController::setVoltageNoLdac(dacChannels[i], dacV0s[i])) {
      return dacWriteFailure(dacChannels[i], dacV0s[i]);
    }
    nextVoltageSet[i] += voltageStepSize[i];
  }
  dacStepsLoaded++;
  if (!prepareNextDacPackets()) {
    return dacSetWriteFailure(numDacChannels, dacChannels, nextVoltageSet);
  }

  // Start the timers only after the initial voltages are in the DAC input
  // registers, so the first LDAC pulse cannot latch stale/partial data.
  FastGpio::digitalWrite(adc_sync, false);
  TimingUtil::setupTimersDacLed(dac_interval_us, dac_settling_time_us,
                                adcMask);
  TimingUtil::dacFlag = false;
  TimingUtil::dacFlagCount = 0;
  TimingUtil::adcFlag = 0;

  int adcFramesRead = 0;
  int dacLatchesObserved = 0;
  int adcAveragesCollected = 0;
  bool dacTimerPending = false;
  bool adcFrameActive = false;
  bool voltageOverflow = false;

  while (adcFramesRead < numSteps && !isWorkerStopRequested()) {
    __WFE();

    if (TimingUtil::consumeDacFlag()) {
      if (dacTimerPending || adcFrameActive) {
        return OperationResult::Failure(
            "ADC averaging exceeded the DAC interval");
      }
      dacTimerPending = true;
      dacLatchesObserved++;
      if (dacLatchesObserved > numSteps) {
        return OperationResult::Failure("Unexpected extra DAC timer event");
      }
    }
    const bool adcConversionStarted =
        TimingUtil::consumeAdcConversionStartedFlag();
    const bool adcPending = TimingUtil::consumeAdcFlag(adcMask);

    if (adcConversionStarted) {
      if (!dacTimerPending || adcFrameActive) {
        return OperationResult::Failure(
            "ADC conversion started without a DAC latch");
      }
      dacTimerPending = false;
      adcFrameActive = true;
      adcAveragesCollected = 0;
      for (int i = 0; i < numAdcChannels; i++) {
        packets[i] = 0.0;
      }

      // The final DAC point is already in the input registers when its
      // conversion starts. Only load another point when one remains.
      if (dacStepsLoaded < numSteps) {
        if (!nextDacPacketsReady ||
            !writeDacPackets(numDacChannels, dacChannels, nextDacPackets)) {
          return dacSetWriteFailure(numDacChannels, dacChannels,
                                    nextVoltageSet);
        }
        for (int i = 0; i < numDacChannels; i++) {
          nextVoltageSet[i] += voltageStepSize[i];
        }
        dacStepsLoaded++;
        if (!prepareNextDacPackets()) {
          return dacSetWriteFailure(numDacChannels, dacChannels,
                                    nextVoltageSet);
        }
      }
    }

    bool haveAdcPackets = false;
    if (adcPending) {
      if (!adcFrameActive) {
        return OperationResult::Failure(
            "Unexpected ADC completion outside a DAC frame");
      }

      const bool finalAverage =
          adcAveragesCollected + 1 == numAdcAverages;
      if (finalAverage) {
        // Stop continuous conversion before reading the final result. This
        // prevents a later conversion from creating a stray DRDY event that
        // could be mistaken for a new DAC frame.
        FastGpio::digitalWrite(adc_sync, false);
      }
      for (int i = 0; i < numAdcChannels; i++) {
        packets[i] += ADCController::getVoltageData(adcChannels[i]);
      }
      adcAveragesCollected++;

      if (finalAverage) {
        for (int i = 0; i < numAdcChannels; i++) {
          packets[i] *= numAdcAveragesInv;
        }
        adcFramesRead++;
        adcFrameActive = false;
        haveAdcPackets = true;
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
  if (adcFrameActive || dacTimerPending ||
      adcAveragesCollected != numAdcAverages ||
      adcFramesRead != numSteps || dacLatchesObserved != numSteps ||
      dacStepsLoaded != numSteps) {
    return OperationResult::Failure("Incomplete DAC-led ramp");
  }

  return OperationResult::Success();
}

OperationResult dacLedBufferRampImpl(
    int numDacChannels, int numAdcChannels, int numSteps, int numAdcAverages,
    float dacIntervalArg, float dacSettlingTimeArg,
    List<int, 0>& dacChannelsList, List<float, 0>& dacV0sList,
    List<float, 0>& dacVfsList, List<int, 1>& adcChannelsList,
    bool enforceTiming) {
  if (!isValidDacChannelCount(numDacChannels) ||
      !isValidAdcChannelCount(numAdcChannels)) {
    return OperationResult::Failure("Invalid number of channels");
  }
  if (!BufferRampCommon::isUint32AtLeast(dacSettlingTimeArg, 1) ||
      !BufferRampCommon::isTimerPeriodUs(dacIntervalArg) ||
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
  if (enforceTiming) {
    OperationResult minimumTimingValidation =
        BufferRampCommon::validateDacLedTiming(
            dacIntervalArg, dacSettlingTimeArg, adcChannels, numAdcChannels,
            numAdcAverages);
    if (!minimumTimingValidation.isSuccess()) {
      return minimumTimingValidation;
    }
  }

  if (!ADCController::resetToPreviousConversionTimes())
    return OperationResult::Failure("ADC reset failed");
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

  OperationResult rampResult =
      runPrepared(numDacChannels, numAdcChannels, numSteps, numAdcAverages,
                  dacIntervalUs, dacSettlingTimeUs, dacChannels, dacV0s,
                  dacVfs, adcChannels, ctx.adcMask());

  return ctx.finish(rampResult);
}

OperationResult dacLedBufferRamp(
    int numDacChannels, int numAdcChannels, int numSteps, int numAdcAverages,
    float dacIntervalArg, float dacSettlingTimeArg,
    List<int, 0>& dacChannelsList, List<float, 0>& dacV0sList,
    List<float, 0>& dacVfsList, List<int, 1>& adcChannelsList) {
  return dacLedBufferRampImpl(numDacChannels, numAdcChannels, numSteps,
                              numAdcAverages, dacIntervalArg,
                              dacSettlingTimeArg, dacChannelsList, dacV0sList,
                              dacVfsList, adcChannelsList, true);
}
COMMAND("DAC_LED_BUFFER_RAMP", dacLedBufferRamp)

OperationResult dacLedBufferRampSudo(
    int numDacChannels, int numAdcChannels, int numSteps, int numAdcAverages,
    float dacIntervalArg, float dacSettlingTimeArg,
    List<int, 0>& dacChannelsList, List<float, 0>& dacV0sList,
    List<float, 0>& dacVfsList, List<int, 1>& adcChannelsList) {
  return dacLedBufferRampImpl(numDacChannels, numAdcChannels, numSteps,
                              numAdcAverages, dacIntervalArg,
                              dacSettlingTimeArg, dacChannelsList, dacV0sList,
                              dacVfsList, adcChannelsList, false);
}
COMMAND("DAC_LED_BUFFER_RAMP_SUDO", dacLedBufferRampSudo)

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
