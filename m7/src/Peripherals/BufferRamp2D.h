#pragma once

#include <Arduino.h>

#include "FunctionRegistry/FunctionRegistryArgumentParser.h"
#include "Peripherals/OperationResult.h"
#include "Peripherals/RampCommand.h"

#include <vector>

using FunctionRegistryParsing::List;

class BufferRamp2D {
 public:
  static void setup();

  static void initializeRegistry();

  // timeSeriesBufferRamp2D:
  // Arguments (in order):
  // numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
  // dacInterval_us, adcInterval_us, retrace (0.0f = false, 1.0f = true), snake (0.0f = false, 1.0f = true),
  // [dacChannelID] * numDacChannels,
  // [startPoint] * numDacChannels,
  // [fastAxisVector] * numDacChannels,
  // [slowAxisVector] * numDacChannels,
  // [adcChannelID] * numAdcChannels
  //
  // The fast/slow axis vectors define a 2D plane in the N-dimensional DAC phase space.
  // Position(s,f) = startPoint + s*slowAxisVector + f*fastAxisVector where s,f is in [0,1]
  // This allows probing arbitrary 2D planar subspaces anywhere in the full DAC phase space.
  static OperationResult timeSeriesBufferRamp2D(
      int numDacChannels, int numAdcChannels, int numStepsFast,
      int numStepsSlow, float dacIntervalArg, float adcIntervalArg,
      float retraceArg, float snakeArg,
      List<int, 0>& dacChannels,
      List<float, 0>& startPoint,
      List<float, 0>& fastAxisVector,
      List<float, 0>& slowAxisVector,
      List<int, 1>& adcChannels);

  // dacLedBufferRamp2D:
  // Arguments (in order):
  // numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
  // dacInterval_us, dacSettlingTime_us, retrace (0.0f = false, 1.0f = true), snake (0.0f = false, 1.0f = true),
  // numAdcAverages,
  // [dacChannelID] * numDacChannels,
  // [startPoint] * numDacChannels,
  // [fastAxisVector] * numDacChannels,
  // [slowAxisVector] * numDacChannels,
  // [adcChannelID] * numAdcChannels
  //
  // The fast/slow axis vectors define a 2D plane in the N-dimensional DAC phase space.
  // Position(s,f) = startPoint + s*slowAxisVector + f*fastAxisVector where s,f is in [0,1]
  // This allows probing arbitrary 2D planar subspaces anywhere in the full DAC phase space.
  static OperationResult dacLedBufferRamp2D(
      int numDacChannels, int numAdcChannels, int numStepsFast,
      int numStepsSlow, float dacIntervalArg, float dacSettlingTimeArg,
      float retraceArg, float snakeArg, int numAdcAverages,
      List<int, 0>& dacChannels,
      List<float, 0>& startPoint,
      List<float, 0>& fastAxisVector,
      List<float, 0>& slowAxisVector,
      List<int, 1>& adcChannels);
};
