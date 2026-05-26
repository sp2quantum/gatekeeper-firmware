#pragma once

#include <Arduino.h>

#include "Commands/BufferRamps/BufferRampCommon.h"
#include "Commands/BufferRamps/RampCommand.h"
#include "Utils/OperationResult.h"

namespace TimeSeriesRamp {
OperationResult runPrepared(int numDacChannels, int numAdcChannels,
                            int numSteps, uint32_t dac_interval_us,
                            uint32_t adc_interval_us, int* dacChannels,
                            float* dacV0s, float* dacVfs, int* adcChannels,
                            uint8_t adcMask);
}

namespace DacLedRamp {
OperationResult validateAdcConversionTimes(int numAdcChannels,
                                           const int* adcChannels);
}

namespace Ramp2DCommon {

inline OperationResult validateRequest(
    int numDacChannels, int numAdcChannels, int numStepsFast,
    int numStepsSlow, float dacIntervalArg, float adcTimingArg,
    float retraceArg, float snakeArg, const int* dacChannels,
    const int* adcChannels, const float* startPoint,
    const float* fastAxisVector, const float* slowAxisVector,
    RampCommand::DacBoundsMode boundsMode) {
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

  OperationResult channelValidation = BufferRampCommon::validateRampChannels(
      dacChannels, numDacChannels, adcChannels, numAdcChannels, false, false);
  if (!channelValidation.isSuccess()) return channelValidation;
  return RampCommand::validateDac2DScanBounds(
      numDacChannels, dacChannels, startPoint, fastAxisVector,
      slowAxisVector, boundsMode);
}

}  // namespace Ramp2DCommon
