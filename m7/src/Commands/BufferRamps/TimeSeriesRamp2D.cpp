#include "Config.h"
#include "FunctionRegistry/FunctionRegistryArgumentParser.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Commands/BufferRamps/RampCommand.h"
#include "Commands/BufferRamps/RampContext.h"
#include "Commands/BufferRamps/Ramp2DCommon.h"

using FunctionRegistryParsing::List;

namespace {

OperationResult timeSeriesBufferRamp2D(
    int numDacChannels, int numAdcChannels, int numStepsFast,
    int numStepsSlow, float dacIntervalArg, float adcIntervalArg,
    float retraceArg, float snakeArg, List<int, 0>& dacChannelsList,
    List<float, 0>& startPointList, List<float, 0>& fastAxisVectorList,
    List<float, 0>& slowAxisVectorList, List<int, 1>& adcChannelsList) {
  int* dacChannels = dacChannelsList.data();
  float* startPoint = startPointList.data();
  float* fastAxisVector = fastAxisVectorList.data();
  float* slowAxisVector = slowAxisVectorList.data();
  int* adcChannels = adcChannelsList.data();

  OperationResult requestValidation = Ramp2DCommon::validateRequest(
      numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
      dacIntervalArg, adcIntervalArg, retraceArg, snakeArg, dacChannels,
      adcChannels, startPoint, fastAxisVector, slowAxisVector,
      RampCommand::DacBoundsMode::Calibrated);
  if (!requestValidation.isSuccess()) return requestValidation;

  const uint32_t dacIntervalUs = static_cast<uint32_t>(dacIntervalArg);
  const uint32_t adcIntervalUs = static_cast<uint32_t>(adcIntervalArg);
  const bool retrace = retraceArg != 0.0f;
  const bool snake = snakeArg != 0.0f;

  float slowStepSize[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < numDacChannels; i++) {
    slowStepSize[i] = numStepsSlow > 1
                          ? slowAxisVector[i] / (numStepsSlow - 1)
                          : 0.0f;
  }

  float currentSlowPosition[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < numDacChannels; i++) {
    currentSlowPosition[i] = startPoint[i];
  }

  RampContext ctx;
  OperationResult setupResult =
      ctx.beginDacAndAdc(adcChannels, numAdcChannels);
  if (!setupResult.isSuccess()) return setupResult;

  OperationResult rampResult = OperationResult::Success();

  for (int slowStep = 0; slowStep < numStepsSlow && !ctx.stopped();
       ++slowStep) {
    const bool reverseFastAxis = snake && ((slowStep % 2) != 0);

    float fastV0s[NUM_DAC_CHANNELS] = {};
    float fastVfs[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      if (reverseFastAxis) {
        fastV0s[i] = currentSlowPosition[i] + fastAxisVector[i];
        fastVfs[i] = currentSlowPosition[i];
      } else {
        fastV0s[i] = currentSlowPosition[i];
        fastVfs[i] = currentSlowPosition[i] + fastAxisVector[i];
      }
    }

    OperationResult ramp1Result = TimeSeriesRamp::runPrepared(
        numDacChannels, numAdcChannels, numStepsFast, dacIntervalUs,
        adcIntervalUs, dacChannels, fastV0s, fastVfs, adcChannels,
        ctx.adcMask());

    OperationResult ramp2Result = OperationResult::Success();
    if (retrace && !snake) {
      ramp2Result = TimeSeriesRamp::runPrepared(
          numDacChannels, numAdcChannels, numStepsFast, dacIntervalUs,
          adcIntervalUs, dacChannels, fastVfs, fastV0s, adcChannels,
          ctx.adcMask());
    }

    if (!ramp1Result.isSuccess() && !ramp2Result.isSuccess()) {
      rampResult = OperationResult::Failure(ramp1Result.getMessage() + "\n" +
                                            ramp2Result.getMessage());
      break;
    } else if (!ramp1Result.isSuccess()) {
      rampResult = OperationResult::Failure(ramp1Result.getMessage());
      break;
    } else if (!ramp2Result.isSuccess()) {
      rampResult = OperationResult::Failure(ramp2Result.getMessage());
      break;
    }

    for (int i = 0; i < numDacChannels; ++i) {
      currentSlowPosition[i] += slowStepSize[i];
    }
  }

  return ctx.finish(rampResult, true, false);
}
COMMAND("2D_TIME_SERIES_BUFFER_RAMP", timeSeriesBufferRamp2D)

}  // namespace
