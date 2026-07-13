#include <vector>

#include "Config.h"
#include "FunctionRegistry/FunctionRegistryArgumentParser.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Commands/BufferRamps/BufferRampCommon.h"
#include "Peripherals/DAC/DACController.h"
#include "Commands/BufferRamps/RampCommand.h"
#include "Commands/BufferRamps/RampContext.h"
#include "Utils/TimingUtil.h"

using FunctionRegistryParsing::List;

namespace AWGRamp {

OperationResult runDacOnly(int numDacChannels, int numSteps,
                           uint32_t dac_interval_us, int* dacChannels,
                           const float* channelMajorVoltages);

namespace {

using BufferRampCommon::dacWriteFailure;
using BufferRampCommon::isValidDacChannelCount;
using BufferRampCommon::validateDacChannels;
using RampCommand::validateDacVoltageListBounds;

OperationResult awgBufferRampImpl(
    int numDacChannels, int numSteps, float dacIntervalArg,
    List<int, 0>& dacChannels,
    List<float, 0, 1>& channelMajorVoltages, bool enforceTiming) {
  if (!BufferRampCommon::isTimerPeriodUs(dacIntervalArg)) {
    return OperationResult::Failure("Invalid dac interval");
  }
  if (enforceTiming) {
    OperationResult timingValidation =
        BufferRampCommon::validateDacOnlyTiming(dacIntervalArg,
                                                numDacChannels);
    if (!timingValidation.isSuccess()) return timingValidation;
  }
  return AWGRamp::runDacOnly(numDacChannels, numSteps,
                             static_cast<uint32_t>(dacIntervalArg),
                             dacChannels.data(), channelMajorVoltages.data());
}

OperationResult awgBufferRamp(int numDacChannels, int numSteps,
                              float dacIntervalArg,
                              List<int, 0>& dacChannels,
                              List<float, 0, 1>& channelMajorVoltages) {
  return awgBufferRampImpl(numDacChannels, numSteps, dacIntervalArg,
                           dacChannels, channelMajorVoltages, true);
}
COMMAND("AWG_BUFFER_RAMP", awgBufferRamp)

OperationResult awgBufferRampSudo(int numDacChannels, int numSteps,
                                  float dacIntervalArg,
                                  List<int, 0>& dacChannels,
                                  List<float, 0, 1>& channelMajorVoltages) {
  return awgBufferRampImpl(numDacChannels, numSteps, dacIntervalArg,
                           dacChannels, channelMajorVoltages, false);
}
COMMAND("AWG_BUFFER_RAMP_SUDO", awgBufferRampSudo)

}  // namespace

OperationResult runDacOnly(int numDacChannels, int numSteps,
                           uint32_t dac_interval_us, int* dacChannels,
                           const float* channelMajorVoltages) {
  if (!TimingUtil::isTimerPeriodRepresentable(dac_interval_us)) {
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
    if (!DACController::writeVoltagePacketNoLdac(dacChannels[i], packet)) {
      return dacWriteFailure(
          dacChannels[i],
          channelMajorVoltages[static_cast<size_t>(i) * numSteps]);
    }
  }

  TimingUtil::setupTimerOnlyDac(dac_interval_us);
  TimingUtil::dacFlag = false;
  TimingUtil::dacFlagCount = 0;

  // Step 0 is preloaded above and each timer tick latches the previous
  // write, so the loop stays one step ahead of the output.
  int step = numSteps > 1 ? 1 : 0;
  while (!ctx.stopped()) {
    __WFE();
    if (TimingUtil::consumeDacFlag()) {
      for (int i = 0; i < numDacChannels; i++) {
        const uint8_t* packet =
            &dacPackets[(static_cast<size_t>(i) *
                             static_cast<size_t>(numSteps) +
                         static_cast<size_t>(step)) *
                        3u];
        if (!DACController::writeVoltagePacketNoLdac(dacChannels[i], packet)) {
          return ctx.finish(
              dacWriteFailure(
                  dacChannels[i],
                  channelMajorVoltages[static_cast<size_t>(i) * numSteps +
                                       static_cast<size_t>(step)]),
              false);
        }
      }
      step++;
      if (step >= numSteps) step = 0;
    }
  }

  return ctx.finish(OperationResult::Success(), false);
}

}  // namespace AWGRamp
