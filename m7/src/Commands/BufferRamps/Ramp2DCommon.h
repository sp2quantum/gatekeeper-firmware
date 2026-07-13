#pragma once

#include <Arduino.h>

#include <limits>
#include "Commands/BufferRamps/BufferRampCommon.h"
#include "Commands/BufferRamps/RampCommand.h"
#include "Utils/OperationResult.h"

namespace TimeSeriesRamp {
OperationResult runPrepared(int numDacChannels, int numAdcChannels,
                            int numSteps, uint32_t dac_interval_us,
                            uint32_t adc_interval_us, int* dacChannels,
                            float* dacV0s, float* dacVfs, int* adcChannels,
                            AdcBoardMask adcMask);
}

namespace DacLedRamp {
OperationResult validateAdcConversionTimes(int numAdcChannels,
                                           const int* adcChannels);
}

namespace Ramp2DCommon {

// Computes the DAC voltages for one point of a 2D raster scan. The fast axis
// sweeps within a scan line; the slow axis advances once per line (or once
// per trace+retrace pair). Snake scans reverse the fast axis on odd lines.
inline void calculateVoltages(int pointIndex, int numStepsFast, bool retrace,
                              bool snake, int numDacChannels,
                              const float* startPoint,
                              const float* fastAxisVector,
                              const float* slowAxisStep,
                              double voltages[NUM_DAC_CHANNELS]) {
  const int scansPerSlowStep = (retrace && !snake) ? 2 : 1;
  const int scanIndex = pointIndex / numStepsFast;
  const int fastStep = pointIndex % numStepsFast;
  const int slowStep = scanIndex / scansPerSlowStep;
  const bool retraceScan =
      (retrace && !snake) && ((scanIndex % scansPerSlowStep) == 1);
  const bool snakeReverse = snake && ((slowStep % 2) != 0);
  const bool reverseFastAxis = retraceScan || snakeReverse;
  const double fastDenominator =
      numStepsFast > 1 ? static_cast<double>(numStepsFast - 1) : 1.0;
  double fastFraction = numStepsFast > 1
                            ? static_cast<double>(fastStep) / fastDenominator
                            : 0.0;
  if (reverseFastAxis) fastFraction = 1.0 - fastFraction;

  for (int i = 0; i < numDacChannels; i++) {
    voltages[i] = static_cast<double>(startPoint[i]) +
                  static_cast<double>(slowStep) *
                      static_cast<double>(slowAxisStep[i]) +
                  fastFraction * static_cast<double>(fastAxisVector[i]);
  }
}

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
  if (!BufferRampCommon::isTimerPeriodUs(adcTimingArg) ||
      !BufferRampCommon::isTimerPeriodUs(dacIntervalArg)) {
    return OperationResult::Failure("Invalid interval");
  }
  if (!RampCommand::isBooleanArg(retraceArg) ||
      !RampCommand::isBooleanArg(snakeArg)) {
    return OperationResult::Failure("Invalid 2D scan boolean argument");
  }
  if (numStepsFast < 1 || numStepsSlow < 1) {
    return OperationResult::Failure("Invalid number of steps");
  }
  const uint64_t scansPerSlowStep =
      (retraceArg != 0.0f && snakeArg == 0.0f) ? 2u : 1u;
  const uint64_t totalPoints = static_cast<uint64_t>(numStepsFast) *
                               static_cast<uint64_t>(numStepsSlow) *
                               scansPerSlowStep;
  if (totalPoints >
      static_cast<uint64_t>(std::numeric_limits<int>::max())) {
    return OperationResult::Failure("2D ramp point count is too large");
  }

  OperationResult channelValidation = BufferRampCommon::validateRampChannels(
      dacChannels, numDacChannels, adcChannels, numAdcChannels, false, false);
  if (!channelValidation.isSuccess()) return channelValidation;
  return RampCommand::validateDac2DScanBounds(
      numDacChannels, dacChannels, startPoint, fastAxisVector,
      slowAxisVector, boundsMode);
}

}  // namespace Ramp2DCommon
