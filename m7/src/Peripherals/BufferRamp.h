#pragma once

#include <Arduino.h>

#include "FunctionRegistry/FunctionRegistryArgumentParser.h"
#include "Peripherals/OperationResult.h"
#include "Peripherals/RampCommand.h"

#include <vector>

using FunctionRegistryParsing::List;

class BufferRamp {
 public:
  static void setup();

  static void initializeRegistry();

  static OperationResult initialize();

  static OperationResult hardResetCalibrationToDefaults();

  struct BoardUsage {
    uint8_t numBoards;
    std::vector<uint8_t> idx;
  };

  enum class TimeSeriesRampMode {
    Streaming,
    Buffered2DRow,
  };

  static BoardUsage getUsedBoards(const int *adcChannels, int numAdcChannels);

  static OperationResult timeSeriesAdcRead(
      int numAdcChannels, List<int, 0>& adcChannels,
      float conversionTimeArg, float totalDurationArg);

  // args:
  // numDacChannels, numAdcChannels, numSteps, dacInterval_us, adcInterval_us,
  // dacChannels..., dacV0s..., dacVfs..., adcChannels...
  static OperationResult timeSeriesBufferRampBase(
      int numDacChannels, int numAdcChannels, int numSteps,
      float dacIntervalArg, float adcIntervalArg,
      List<int, 0>& dacChannels,
      List<float, 0>& dacV0s,
      List<float, 0>& dacVfs,
      List<int, 1>& adcChannels);

  static OperationResult prepareTimeSeriesBufferRampHardware(
      int numAdcChannels, int* adcChannels, uint8_t& adcMask,
      BoardUsage& boardUsage);

  static OperationResult runPreparedTimeSeriesBufferRamp(
      int numDacChannels, int numAdcChannels, int numSteps,
      uint32_t dac_interval_us, uint32_t adc_interval_us, int* dacChannels,
      float* dacV0s, float* dacVfs, int* adcChannels, uint8_t adcMask,
      TimeSeriesRampMode mode = TimeSeriesRampMode::Streaming);

  static void cleanupTimeSeriesBufferRampHardware(
      int numAdcChannels, int* adcChannels, const BoardUsage& boardUsage);

  // args:
  // numDacChannels, numAdcChannels, numSteps, numAdcAverages, dacInterval_us,
  // dacSettlingTime_us, dacChannels..., dacV0s..., dacVfs..., adcChannels...
  static OperationResult dacLedBufferRampBase(
      int numDacChannels, int numAdcChannels, int numSteps,
      int numAdcAverages, float dacIntervalArg, float dacSettlingTimeArg,
      List<int, 0>& dacChannels,
      List<float, 0>& dacV0s,
      List<float, 0>& dacVfs,
      List<int, 1>& adcChannels);

  static OperationResult prepareDacLedBufferRampHardware(
      int numAdcChannels, int* adcChannels, uint8_t& adcMask,
      BoardUsage& boardUsage);

  static OperationResult runPreparedDacLedBufferRamp(
      int numDacChannels, int numAdcChannels, int numSteps, int numAdcAverages,
      int* dacChannels, float* dacV0s, float* dacVfs, int* adcChannels,
      uint8_t adcMask);

  static void cleanupDacLedBufferRampHardware(
      int numAdcChannels, int* adcChannels, const BoardUsage& boardUsage);

  static OperationResult OwenRampWrapper(
      int numDacChannels, int numAdcChannels, int numLoops,
      int numDacStepsPerLoop, int numAdcAverages, float dacIntervalArg,
      List<int, 0>& dacChannels,
      List<int, 1>& adcChannels,
      List<float, 0, 3>& dacVoltageStorage,
      int specialIndex, int specialWidth, int numStepsPerSpecialRamp,
      List<float, 0>& specialDacV0s,
      List<float, 0>& specialDacVfs);

  static OperationResult OwenRampBase(
      int numDacChannels, int numAdcChannels, int numLoops,
      int numDacStepsPerLoop, int numAdcAverages,
      uint32_t dac_interval_us, int* dacChannels,
      float** dacVoltageLists, int* adcChannels, int specialIndex,
      int numStepsPerSpecialRamp, float* specialDacV0s,
      float* specialDacVfs);

  static OperationResult AWGBufferRampWrapper(
      int numDacChannels, int numSteps, float dacIntervalArg,
      List<int, 0>& dacChannels,
      List<float, 0, 1>& channelMajorVoltages);

  static OperationResult AWGDacOnlyRampBase(
      int numDacChannels,
      int numSteps,
      uint32_t dac_interval_us,
      int* dacChannels,
      const float* channelMajorVoltages);

  // AWG_WITH_ADC: AWG waveform with ADC reading at each step
  // Format: AWG_WITH_ADC,dacN,adcN,numSteps,dac_interval_us,numCycles,dacChannels...,adcChannels...,voltages...
  // Voltages are channel-major: all points for DAC0, then all for DAC1, etc.
  static OperationResult AWGWithADCWrapper(
      int numDacChannels, int numAdcChannels, int numSteps,
      float dacIntervalArg, int numCycles,
      List<int, 0>& dacChannels,
      List<int, 1>& adcChannels,
      List<float, 0, 2>& channelMajorVoltages);

  static OperationResult AWGWithADCBase(
      int numDacChannels, int numAdcChannels, int numSteps,
      uint32_t dac_interval_us, int numCycles,
      int* dacChannels, int* adcChannels, const float* channelMajorVoltages);

  static OperationResult AWGBufferRampBase(
      int numDacChannels, int numAdcChannels, int numLoops,
      int numDacStepsPerLoop, int numAdcAverages,
      uint32_t dac_interval_us, int* dacChannels,
      float** dacVoltageLists, int* adcChannels);

  static OperationResult dacChannelCalibration();

  // args: numDacChannels, numAdcChannels, numDacSteps,
  // numAdcMeasuresPerDacStep, numAdcAverages, numAdcConversionSkips,
  // adcConversionTime_us, dacChannels..., dacV0_1s..., dacVf_1s...,
  // dacV0_2s..., dacVf_2s..., adcChannels...
  static OperationResult boxcarAverageRamp(
      int numDacChannels, int numAdcChannels, int numDacSteps,
      int numAdcMeasuresPerDacStep, int numAdcAverages,
      int numAdcConversionSkips, float adcConversionTimeArg,
      List<int, 0>& dacChannels,
      List<float, 0>& dacV0_1s,
      List<float, 0>& dacVf_1s,
      List<float, 0>& dacV0_2s,
      List<float, 0>& dacVf_2s,
      List<int, 1>& adcChannels);
};
