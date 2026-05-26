#include "Config.h"
#include "FunctionRegistry/FunctionRegistryArgumentParser.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Peripherals/ADC/ADCController.h"
#include "Commands/BufferRamps/BufferRampCommon.h"
#include "Peripherals/DAC/DACController.h"
#include "Commands/BufferRamps/RampCommand.h"
#include "Commands/BufferRamps/RampContext.h"
#include "Commands/BufferRamps/Ramp2DCommon.h"
#include "Utils/FastGpio.h"
#include "Utils/TimingUtil.h"
#include "shared_memory.h"

using FunctionRegistryParsing::List;

namespace {

using BufferRampCommon::dacWriteFailure;
using BufferRampCommon::encodeDacVoltagePackets;
using BufferRampCommon::sendVoltageFrame;
using BufferRampCommon::writeDacPackets;

void calculateVoltages(int pointIndex, int numStepsFast, bool retrace,
                       bool snake, int numDacChannels,
                       const float* startPoint, const float* fastAxisVector,
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

OperationResult runPrepared(
    int numDacChannels, int numAdcChannels, int numStepsFast,
    int numStepsSlow, bool retrace, bool snake, int numAdcAverages,
    uint32_t dac_interval_us, uint32_t dac_settling_time_us,
    int* dacChannels, float* startPoint, float* fastAxisVector,
    float* slowAxisVector, int* adcChannels, uint8_t adcMask) {
  const int scansPerSlowStep = (retrace && !snake) ? 2 : 1;
  const int totalPoints = numStepsFast * numStepsSlow * scansPerSlowStep;
  if (totalPoints < 1) {
    return OperationResult::Failure("Invalid number of 2D ramp points");
  }

  float slowAxisStep[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < numDacChannels; i++) {
    slowAxisStep[i] =
        numStepsSlow > 1 ? slowAxisVector[i] / (numStepsSlow - 1) : 0.0f;
  }

  double currentVoltages[NUM_DAC_CHANNELS] = {};
  calculateVoltages(0, numStepsFast, retrace, snake, numDacChannels,
                    startPoint, fastAxisVector, slowAxisStep, currentVoltages);
  byte nextDacPackets[NUM_DAC_CHANNELS][3] = {};
  if (!encodeDacVoltagePackets(numDacChannels, dacChannels, currentVoltages,
                               nextDacPackets) ||
      !writeDacPackets(numDacChannels, dacChannels, nextDacPackets)) {
    return dacWriteFailure(dacChannels[0], currentVoltages[0]);
  }

  int nextDacPointIndex = 1;
  bool nextDacPacketsReady = false;
  double nextVoltages[NUM_DAC_CHANNELS] = {};
  auto prepareNextDacPackets = [&]() {
    if (nextDacPointIndex >= totalPoints) {
      nextDacPacketsReady = false;
      return true;
    }
    calculateVoltages(nextDacPointIndex, numStepsFast, retrace, snake,
                      numDacChannels, startPoint, fastAxisVector, slowAxisStep,
                      nextVoltages);
    nextDacPacketsReady = encodeDacVoltagePackets(
        numDacChannels, dacChannels, nextVoltages, nextDacPackets);
    return nextDacPacketsReady;
  };
  if (!prepareNextDacPackets()) {
    return dacWriteFailure(dacChannels[0], nextVoltages[0]);
  }

  FastGpio::digitalWrite(adc_sync, false);
  TimingUtil::setupTimersDacLed(dac_interval_us, dac_settling_time_us,
                                adcMask);
  TimingUtil::dacFlag = false;
  TimingUtil::adcFlag = 0;

  double packets[NUM_ADC_CHANNELS] = {};
  const double numAdcAveragesInv = 1.0 / static_cast<double>(numAdcAverages);
  int adcReads = 0;
  bool dacTimerPending = false;
  bool voltageOverflow = false;

  while (adcReads < totalPoints && !isWorkerStopRequested()) {
    __WFE();

    if (nextDacPointIndex < totalPoints && TimingUtil::consumeDacFlag()) {
      dacTimerPending = true;
    }
    const bool adcConversionStarted =
        TimingUtil::consumeAdcConversionStartedFlag();
    const bool adcPending = TimingUtil::consumeAdcFlag(adcMask);
    if (adcPending) {
      for (int i = 0; i < numAdcChannels; i++) {
        double total = 0.0;
        for (int j = 0; j < numAdcAverages; j++) {
          total += ADCController::getVoltageDataNoTransaction(adcChannels[i]);
        }
        packets[i] = total * numAdcAveragesInv;
      }
      FastGpio::digitalWrite(adc_sync, false);
      adcReads++;
    }

    if (dacTimerPending && adcConversionStarted) {
      if (!nextDacPacketsReady ||
          !writeDacPackets(numDacChannels, dacChannels, nextDacPackets)) {
        return dacWriteFailure(dacChannels[0], nextVoltages[0]);
      }
      dacTimerPending = false;
      nextDacPointIndex++;
      if (!prepareNextDacPackets()) {
        return dacWriteFailure(dacChannels[0], nextVoltages[0]);
      }
    }

    if (adcPending) {
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

OperationResult dacLedBufferRamp2D(
    int numDacChannels, int numAdcChannels, int numStepsFast,
    int numStepsSlow, float dacIntervalArg, float dacSettlingTimeArg,
    float retraceArg, float snakeArg, int numAdcAverages,
    List<int, 0>& dacChannelsList, List<float, 0>& startPointList,
    List<float, 0>& fastAxisVectorList, List<float, 0>& slowAxisVectorList,
    List<int, 1>& adcChannelsList) {
  if (numAdcAverages < 1) {
    return OperationResult::Failure("Invalid number of ADC averages");
  }

  int* dacChannels = dacChannelsList.data();
  float* startPoint = startPointList.data();
  float* fastAxisVector = fastAxisVectorList.data();
  float* slowAxisVector = slowAxisVectorList.data();
  int* adcChannels = adcChannelsList.data();

  OperationResult requestValidation = Ramp2DCommon::validateRequest(
      numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
      dacIntervalArg, dacSettlingTimeArg, retraceArg, snakeArg, dacChannels,
      adcChannels, startPoint, fastAxisVector, slowAxisVector,
      RampCommand::DacBoundsMode::CalibratedAndGlobal);
  if (!requestValidation.isSuccess()) return requestValidation;
  if (dacSettlingTimeArg >= dacIntervalArg) {
    return OperationResult::Failure("Invalid interval or settling time");
  }

  ADCController::resetToPreviousConversionTimes();
  OperationResult timingValidation =
      DacLedRamp::validateAdcConversionTimes(numAdcChannels, adcChannels);
  if (!timingValidation.isSuccess()) return timingValidation;

  const uint32_t dacIntervalUs = static_cast<uint32_t>(dacIntervalArg);
  const uint32_t dacSettlingTimeUs =
      static_cast<uint32_t>(dacSettlingTimeArg);
  const bool retrace = retraceArg != 0.0f;
  const bool snake = snakeArg != 0.0f;

  RampContext ctx;
  OperationResult setupResult =
      ctx.beginDacAndAdc(adcChannels, numAdcChannels);
  if (!setupResult.isSuccess()) return setupResult;

  OperationResult rampResult = runPrepared(
      numDacChannels, numAdcChannels, numStepsFast, numStepsSlow, retrace,
      snake, numAdcAverages, dacIntervalUs, dacSettlingTimeUs, dacChannels,
      startPoint, fastAxisVector, slowAxisVector, adcChannels,
      ctx.adcMask());

  return ctx.finish(rampResult);
}
COMMAND("2D_DAC_LED_BUFFER_RAMP", dacLedBufferRamp2D)

}  // namespace
