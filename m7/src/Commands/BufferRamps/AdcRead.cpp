#include "Config.h"
#include "FunctionRegistry/FunctionRegistryArgumentParser.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Peripherals/ADC/ADCController.h"
#include "Commands/BufferRamps/BufferRampCommon.h"
#include "Commands/BufferRamps/RampContext.h"
#include "Utils/FastGpio.h"
#include "Utils/TimingUtil.h"
#include "shared_memory.h"

using FunctionRegistryParsing::List;

namespace {

using BufferRampCommon::isValidAdcChannelCount;
using BufferRampCommon::maxSelectedAdcChannelsPerBoard;
using BufferRampCommon::sendVoltageFrame;

OperationResult timeSeriesAdcRead(int numAdcChannels,
                                  List<int, 0>& adcChannelsList,
                                  float conversionTimeArg,
                                  float totalDurationArg) {
  if (!isValidAdcChannelCount(numAdcChannels)) {
    return OperationResult::Failure("Invalid number of ADC channels");
  }
  OperationResult adcValidation = BufferRampCommon::validateAdcChannels(
      adcChannelsList.data(), numAdcChannels);
  if (!adcValidation.isSuccess()) return adcValidation;
  if (!BufferRampCommon::isUint32AtLeast(conversionTimeArg, 1) ||
      !BufferRampCommon::isUint32AtLeast(totalDurationArg, 82)) {
    return OperationResult::Failure("Invalid total duration");
  }

  int* adcChannels = adcChannelsList.data();
  const uint32_t conversionTimeUs = static_cast<uint32_t>(conversionTimeArg);
  const uint32_t totalDurationUs = static_cast<uint32_t>(totalDurationArg);

  for (int i = 0; i < numAdcChannels; i++) {
    ADCController::setConversionTime(adcChannels[i], conversionTimeUs);
  }
  float realConversionTime =
      ADCController::getConversionTimeFloat(adcChannels[0]);
  const int maxIndependentAdcs =
      maxSelectedAdcChannelsPerBoard(adcChannels, numAdcChannels);
  const double samplePeriodUsFloat =
      maxIndependentAdcs * realConversionTime * 1.5f;
  const int samplePeriodUs = static_cast<int>(samplePeriodUsFloat);
  const int savedDataSize = totalDurationUs / samplePeriodUs;

  if (!sendVoltageFrame(&samplePeriodUsFloat, 1)) {
    clearWorkerStopRequest();
    return OperationResult::Failure("Voltage output buffer overflow");
  }

  RampContext ctx;
  ctx.beginAdcOnly(adcChannels, numAdcChannels);

  TimingUtil::setupTimersOnlyADC(samplePeriodUs);

  int samplesCaptured = 0;
  bool voltageOverflow = false;

  while (samplesCaptured < savedDataSize && !ctx.stopped()) {
    __WFE();
    if (TimingUtil::consumeAdcFlag(ctx.adcMask())) {
      double packets[NUM_ADC_CHANNELS] = {};
      for (int i = 0; i < numAdcChannels; i++) {
        packets[i] =
            ADCController::getVoltageDataNoTransaction(adcChannels[i]);
      }
      FastGpio::digitalWrite(adc_sync, false);
      if (!sendVoltageFrame(packets, numAdcChannels)) {
        voltageOverflow = true;
        break;
      }
      samplesCaptured++;
    }
  }

  return ctx.finish(
      voltageOverflow
          ? OperationResult::Failure("Voltage output buffer overflow")
          : OperationResult::Success(),
      false);
}
COMMAND("TIME_SERIES_ADC_READ", timeSeriesAdcRead)

}  // namespace
