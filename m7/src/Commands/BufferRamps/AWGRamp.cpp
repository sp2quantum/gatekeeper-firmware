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

namespace AWGRamp {

OperationResult runDacOnly(int numDacChannels, int numSteps,
                           uint32_t dac_interval_us, int* dacChannels,
                           const float* channelMajorVoltages);

namespace {

using BufferRampCommon::dacWriteFailure;
using BufferRampCommon::isValidAdcChannelCount;
using BufferRampCommon::isValidDacChannelCount;
using BufferRampCommon::maxAdcConversionTimePerBoard;
using BufferRampCommon::sendVoltageFrame;
using BufferRampCommon::validateDacChannels;
using BufferRampCommon::validateRampChannels;
using RampCommand::validateDacVoltageListBounds;

OperationResult awgBufferRamp(int numDacChannels, int numSteps,
                              float dacIntervalArg,
                              List<int, 0>& dacChannels,
                              List<float, 0, 1>& channelMajorVoltages) {
  if (!BufferRampCommon::isUint32AtLeast(dacIntervalArg, 1)) {
    return OperationResult::Failure("Invalid number of channels or steps");
  }
  return AWGRamp::runDacOnly(numDacChannels, numSteps,
                             static_cast<uint32_t>(dacIntervalArg),
                             dacChannels.data(), channelMajorVoltages.data());
}
COMMAND("AWG_BUFFER_RAMP", awgBufferRamp)

}  // namespace

OperationResult runDacOnly(int numDacChannels, int numSteps,
                           uint32_t dac_interval_us, int* dacChannels,
                           const float* channelMajorVoltages) {
  if (dac_interval_us < 1) {
    return OperationResult::Failure("Invalid dac interval");
  }
  if (!isValidDacChannelCount(numDacChannels) || numSteps < 1) {
    return OperationResult::Failure("Invalid number of channels or steps");
  }
  OperationResult dacValidation =
      validateDacChannels(dacChannels, numDacChannels);
  if (!dacValidation.isSuccess()) return dacValidation;

  OperationResult waveformBounds = validateDacVoltageListBounds(
      numDacChannels, numSteps, dacChannels, channelMajorVoltages);
  if (!waveformBounds.isSuccess()) return waveformBounds;

  std::vector<uint8_t> dacPackets(
      static_cast<size_t>(numDacChannels) * static_cast<size_t>(numSteps) *
      3u);
  for (int i = 0; i < numDacChannels; i++) {
    int ch = dacChannels[i];
    const float* vlist =
        &channelMajorVoltages[static_cast<size_t>(i) *
                              static_cast<size_t>(numSteps)];
    for (int j = 0; j < numSteps; j++) {
      uint8_t* packet =
          &dacPackets[(static_cast<size_t>(i) * static_cast<size_t>(numSteps) +
                       static_cast<size_t>(j)) *
                      3u];
      if (!DACController::encodeVoltagePacket(ch, vlist[j], packet)) {
        return dacWriteFailure(ch, vlist[j]);
      }
    }
  }

  RampContext ctx;
  ctx.beginDacOnly();

  for (int i = 0; i < numDacChannels; i++) {
    const uint8_t* packet =
        &dacPackets[static_cast<size_t>(i) * static_cast<size_t>(numSteps) *
                    3u];
    DACController::writeVoltagePacketNoLdac(dacChannels[i], packet);
  }

  TimingUtil::setupTimerOnlyDac(dac_interval_us);
  TimingUtil::dacFlag = false;
  TimingUtil::dacFlagCount = 0;

  int step = 1;
  while (!ctx.stopped()) {
    __WFE();
    if (TimingUtil::consumeDacFlag()) {
      for (int i = 0; i < numDacChannels; i++) {
        const uint8_t* packet =
            &dacPackets[(static_cast<size_t>(i) *
                             static_cast<size_t>(numSteps) +
                         static_cast<size_t>(step)) *
                        3u];
        DACController::writeVoltagePacketNoLdac(dacChannels[i], packet);
      }
      step++;
      if (step >= numSteps) step = 0;
    }
  }

  return ctx.finish(OperationResult::Success(), false);
}

OperationResult runLoop(int numDacChannels, int numAdcChannels, int numLoops,
                        int numDacStepsPerLoop, int numAdcAverages,
                        uint32_t dac_interval_us, int* dacChannels,
                        float** dacVoltageLists, int* adcChannels) {
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

  OperationResult waveformBounds = validateDacVoltageListBounds(
      numDacChannels, numDacStepsPerLoop, dacChannels, dacVoltageLists);
  if (!waveformBounds.isSuccess()) return waveformBounds;

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

  while (currentLoop < numLoops && !ctx.stopped()) {
    __WFE();

    if (currentDacStep < numLoops * numDacStepsPerLoop &&
        TimingUtil::consumeDacFlag()) {
      for (int i = 0; i < numDacChannels; i++) {
        float voltage = dacVoltageLists[i][currentDacStep];
        DACController::setVoltageNoTransactionNoLdac(dacChannels[i], voltage);
      }
      currentDacStep++;
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

}  // namespace AWGRamp
