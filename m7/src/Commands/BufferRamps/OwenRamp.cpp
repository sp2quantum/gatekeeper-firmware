#include "Config.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Peripherals/ADC/ADCController.h"
#include "Commands/BufferRamps/BufferRampCommon.h"
#include "Peripherals/DAC/DACController.h"
#include "Commands/BufferRamps/RampCommand.h"
#include "Commands/BufferRamps/RampContext.h"
#include "Utils/FastGpio.h"
#include "Utils/TimingUtil.h"
#include "shared_memory.h"

namespace OwenRamp {
namespace {

using BufferRampCommon::isValidAdcChannelCount;
using BufferRampCommon::isValidDacChannelCount;
using BufferRampCommon::maxAdcConversionTimePerBoard;
using BufferRampCommon::sendVoltageFrame;
using BufferRampCommon::validateRampChannels;
using RampCommand::validateDacVoltageListBounds;

OperationResult runOwenRamp(
    int numDacChannels, int numAdcChannels, int numLoops,
    int numDacStepsPerLoop, int numAdcAverages, uint32_t dac_interval_us,
    int* dacChannels, float** dacVoltageLists, int* adcChannels,
    int specialIndex, int numStepsPerSpecialRamp, float* specialDacV0s,
    float* specialDacVfs) {
  if (dac_interval_us < 1) {
    return OperationResult::Failure("Invalid interval or settling time");
  }
  if (numAdcAverages < 1) {
    return OperationResult::Failure("Invalid number of ADC averages");
  }
  if (numLoops < 1 || numDacStepsPerLoop < 1) {
    return OperationResult::Failure(
        "Invalid number of loops or steps per loop");
  }
  if (!isValidDacChannelCount(numDacChannels) ||
      !isValidAdcChannelCount(numAdcChannels)) {
    return OperationResult::Failure("Invalid number of channels");
  }

  OperationResult channelValidation = validateRampChannels(
      dacChannels, numDacChannels, adcChannels, numAdcChannels);
  if (!channelValidation.isSuccess()) return channelValidation;

  OperationResult waveformBounds = validateDacVoltageListBounds(
      numDacChannels, numDacStepsPerLoop, dacChannels, dacVoltageLists);
  if (!waveformBounds.isSuccess()) return waveformBounds;

  for (int i = 0; i < numDacChannels; i++) {
    int ch = dacChannels[i];
    float lowerBound = DACController::getLowerBound(ch);
    float upperBound = DACController::getUpperBound(ch);
    if (specialDacV0s[i] < lowerBound || specialDacV0s[i] > upperBound ||
        specialDacVfs[i] < lowerBound || specialDacVfs[i] > upperBound) {
      return OperationResult::Failure("DAC " + String(ch) +
                                      " special ramp voltage out of bounds");
    }
  }

  double packets[NUM_ADC_CHANNELS] = {};
  double numAdcAveragesInv = 1.0 / static_cast<double>(numAdcAverages);

  RampContext ctx;
  ctx.beginDacAndAdc(adcChannels, numAdcChannels);

  const float maxConvTime =
      maxAdcConversionTimePerBoard(adcChannels, numAdcChannels);
  uint32_t totalDacSweepTime = numDacStepsPerLoop * dac_interval_us;
  if (maxConvTime * numAdcAverages + 180 >= totalDacSweepTime) {
    return ctx.finish(
        OperationResult::Failure(
            "DAC sweep time is too short for specified ADC conversion time, "
            "please increase dac_interval_us or reduce numDacStepsPerLoop"),
        false);
  }

  float currentSpecialDacVoltages[NUM_DAC_CHANNELS] = {};
  float specialDacVoltageStep[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < numDacChannels; i++) {
    currentSpecialDacVoltages[i] = specialDacV0s[i];
    specialDacVoltageStep[i] =
        (specialDacVfs[i] - specialDacV0s[i]) / numLoops;
  }

  for (int i = 0; i < numDacChannels; i++) {
    DACController::setVoltageNoTransactionNoLdac(dacChannels[i],
                                                  dacVoltageLists[i][0]);
  }

  TimingUtil::setupTimerOnlyDac(dac_interval_us);
  TimingUtil::dacFlag = false;
  TimingUtil::dacFlagCount = 0;

  int currentLoop = 0;
  int currentDacStep = 1;
  bool voltageOverflow = false;
  bool done = false;
  int subIndex = 0;

  while (currentLoop < numLoops && !ctx.stopped()) {
    __WFE();

    if (currentDacStep < numLoops * numDacStepsPerLoop &&
        TimingUtil::consumeDacFlag()) {
      if (currentDacStep == specialIndex) {
        for (int i = 0; i < numDacChannels; i++) {
          DACController::setVoltageNoTransactionNoLdac(
              dacChannels[i], currentSpecialDacVoltages[i]);
        }
        subIndex++;
        if (subIndex >= numStepsPerSpecialRamp) {
          subIndex = 0;
          currentDacStep++;
          for (int i = 0; i < numDacChannels; i++) {
            currentSpecialDacVoltages[i] += specialDacVoltageStep[i];
          }
        }
      } else {
        for (int i = 0; i < numDacChannels; i++) {
          float voltage = dacVoltageLists[i][currentDacStep];
          DACController::setVoltageNoTransactionNoLdac(dacChannels[i],
                                                        voltage);
        }
        currentDacStep++;
      }

      if (currentDacStep >= numDacStepsPerLoop) {
        currentDacStep = 0;
        done = true;
      }
    }

    if (done) {
      done = false;
      for (int i = 0; i < numAdcChannels; i++) {
        double total = 0.0;
        for (int j = 0; j < numAdcAverages; j++) {
          total += ADCController::getVoltage(adcChannels[i]);
        }
        packets[i] = total * numAdcAveragesInv;
      }
      if (!sendVoltageFrame(packets, numAdcChannels)) {
        voltageOverflow = true;
      }
      FastGpio::digitalWrite(adc_sync, false);
      currentLoop++;
    }
  }

  return ctx.finish(
      voltageOverflow
          ? OperationResult::Failure("Voltage output buffer overflow")
          : OperationResult::Success());
}

}  // namespace

OperationResult owenRamp(
    int numDacChannels, int numAdcChannels, int numLoops,
    int numDacStepsPerLoop, int numAdcAverages, float dacIntervalArg,
    List<int, 0>& dacChannelsList, List<int, 1>& adcChannelsList,
    List<float, 0, 3>& dacVoltageStorage, int specialIndex,
    int specialWidth, int numStepsPerSpecialRamp,
    List<float, 0>& specialDacV0s, List<float, 0>& specialDacVfs) {
  if (!isValidDacChannelCount(numDacChannels) ||
      !isValidAdcChannelCount(numAdcChannels) || numLoops < 1 ||
      numDacStepsPerLoop < 1 || numAdcAverages < 1 ||
      !BufferRampCommon::isUint32AtLeast(dacIntervalArg, 1)) {
    return OperationResult::Failure(
        "Invalid channel or loop/step/average count");
  }
  if (specialIndex < 0 || specialIndex >= numDacStepsPerLoop ||
      specialWidth < 0 || numStepsPerSpecialRamp < 1) {
    return OperationResult::Failure("Invalid Owen special ramp parameters");
  }

  float* dacVoltageLists[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < numDacChannels; ++i) {
    dacVoltageLists[i] =
        &dacVoltageStorage[static_cast<size_t>(i) *
                           static_cast<size_t>(numDacStepsPerLoop)];
  }

  return runOwenRamp(
      numDacChannels, numAdcChannels, numLoops, numDacStepsPerLoop,
      numAdcAverages, static_cast<uint32_t>(dacIntervalArg),
      dacChannelsList.data(), dacVoltageLists, adcChannelsList.data(),
      specialIndex, numStepsPerSpecialRamp, specialDacV0s.data(),
      specialDacVfs.data());
}

}  // namespace OwenRamp
