#include "Config.h"

#include <array>
#include <utility>

#include "FunctionRegistry/FunctionRegistryArgumentParser.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Peripherals/ADC/ADCController.h"
#include "Commands/BufferRamps/BufferRampCommon.h"
#include "PeripheralCommsController.h"
#include "Utils/FastGpio.h"
#include "Utils/TimingUtil.h"
#include "shared_memory.h"

using FunctionRegistryParsing::List;

namespace {

using BufferRampCommon::isValidAdcChannelCount;
using BufferRampCommon::sendVoltageFrame;

volatile AdcBoardMask adcReadExpectedMask = 0;

template <int boardIndex>
void adcReadDataReadyISR() {
  const AdcBoardMask bit = TimingUtil::adcBoardBit(boardIndex);
  TimingUtil::adcConversionInProgressMask &= ~bit;
  TimingUtil::adcFlag |= bit;
  if (adcReadExpectedMask != 0 &&
      (TimingUtil::adcFlag & adcReadExpectedMask) == adcReadExpectedMask) {
    FastGpio::digitalWrite(adc_sync, false);
  }
  __SEV();
}

using AdcIsr = void (*)();

template <size_t... Indices>
std::array<AdcIsr, sizeof...(Indices)> makeAdcReadIsrFunctions(
    std::index_sequence<Indices...>) {
  return {{adcReadDataReadyISR<Indices>...}};
}

const auto kAdcReadIsrFunctions =
    makeAdcReadIsrFunctions(std::make_index_sequence<NUM_ADC_BOARDS>{});

void clearAdcReadFlags() {
  __disable_irq();
  TimingUtil::adcFlag = 0;
  TimingUtil::adcConversionInProgressMask = 0;
  TimingUtil::adcConversionWatchMask = 0;
  TimingUtil::adcConversionStartedFlag = false;
  adcReadExpectedMask = 0;
  __enable_irq();
}

bool consumeExpectedAdcFlag(AdcBoardMask expectedMask) {
  if (expectedMask == 0 ||
      (TimingUtil::adcFlag & expectedMask) != expectedMask) {
    return false;
  }

  __disable_irq();
  const bool pending =
      (TimingUtil::adcFlag & expectedMask) == expectedMask;
  if (pending) {
    TimingUtil::adcFlag &= ~expectedMask;
    adcReadExpectedMask = 0;
  }
  __enable_irq();
  return pending;
}

void attachDataReadyInterrupts(AdcBoardMask boardMask) {
  for (int board = 0; board < NUM_ADC_BOARDS; board++) {
    if ((boardMask & TimingUtil::adcBoardBit(board)) == 0) continue;
    const int pin = ADCController::getDataReadyPin(board);
    if (pin == NC) continue;
    attachInterrupt(digitalPinToInterrupt(pin), kAdcReadIsrFunctions[board],
                    FALLING);
  }
}

void detachDataReadyInterrupts(AdcBoardMask boardMask) {
  for (int board = 0; board < NUM_ADC_BOARDS; board++) {
    if ((boardMask & TimingUtil::adcBoardBit(board)) == 0) continue;
    const int pin = ADCController::getDataReadyPin(board);
    if (pin == NC) continue;
    detachInterrupt(digitalPinToInterrupt(pin));
  }
}

OperationResult timeSeriesAdcReadImpl(int numAdcChannels,
                                      List<int, 0>& adcChannelsList,
                                      float conversionTimeArg,
                                      float totalDurationArg,
                                      bool enforceTiming) {
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
  if (enforceTiming && conversionTimeArg < 82.0f) {
    return OperationResult::Failure(
        "ADC conversion time too short (" + String(conversionTimeArg, 3) +
        " us < minimum 82 us)");
  }

  int* adcChannels = adcChannelsList.data();
  const uint32_t conversionTimeUs = static_cast<uint32_t>(conversionTimeArg);
  const uint32_t totalDurationUs = static_cast<uint32_t>(totalDurationArg);

  uint8_t boardDepth[NUM_ADC_BOARDS] = {};
  AdcBoardMask boardMask = 0;
  uint8_t maxDepth = 0;
  int channelAtSlot[NUM_ADC_BOARDS][NUM_CHANNELS_PER_ADC_BOARD] = {};
  int outputIndexAtSlot[NUM_ADC_BOARDS][NUM_CHANNELS_PER_ADC_BOARD] = {};

  for (int outputIndex = 0; outputIndex < numAdcChannels; outputIndex++) {
    const int channel = adcChannels[outputIndex];
    const uint8_t board = BufferRampCommon::adcBoardForChannel(channel);
    const uint8_t slot = boardDepth[board]++;
    channelAtSlot[board][slot] = channel;
    outputIndexAtSlot[board][slot] = outputIndex;
    boardMask |= TimingUtil::adcBoardBit(board);
    if (boardDepth[board] > maxDepth) {
      maxDepth = boardDepth[board];
    }
  }

  float conversionTimeByOutput[NUM_ADC_CHANNELS] = {};
  for (int outputIndex = 0; outputIndex < numAdcChannels; outputIndex++) {
    const int channel = adcChannels[outputIndex];
    const uint8_t board = BufferRampCommon::adcBoardForChannel(channel);
    const bool multiplexed = boardDepth[board] > 1;
    const float actualConversionTime =
        ADCController::presetConversionTime(
            channel, static_cast<int>(conversionTimeUs), multiplexed);
    if (actualConversionTime < 0.0f) {
      return OperationResult::Failure(
          "The filter word you selected is not valid.");
    }
    conversionTimeByOutput[outputIndex] = actualConversionTime;
  }

  double samplePeriodUsFloat = 0.0;
  for (int slot = 0; slot < maxDepth; slot++) {
    float slotConversionUs = 0.0f;
    for (int board = 0; board < NUM_ADC_BOARDS; board++) {
      if (boardDepth[board] <= slot) continue;
      const int outputIndex = outputIndexAtSlot[board][slot];
      const float conversionUs = conversionTimeByOutput[outputIndex];
      if (conversionUs > slotConversionUs) {
        slotConversionUs = conversionUs;
      }
    }
    samplePeriodUsFloat += slotConversionUs;
  }

  const int savedDataSize =
      static_cast<int>(static_cast<double>(totalDurationUs) /
                       samplePeriodUsFloat);

  if (!sendVoltageFrame(&samplePeriodUsFloat, 1)) {
    clearWorkerStopRequest();
    return OperationResult::Failure("Voltage output buffer overflow");
  }
  if (savedDataSize <= 0) {
    ADCController::resetToPreviousConversionTimes();
    return OperationResult::Success();
  }

  auto activeMaskForSlot = [&](int slot) {
    AdcBoardMask mask = 0;
    for (int board = 0; board < NUM_ADC_BOARDS; board++) {
      if (boardDepth[board] > slot) {
        mask |= TimingUtil::adcBoardBit(board);
      }
    }
    return mask;
  };

  auto startSlot = [&](int slot) {
    const AdcBoardMask expectedMask = activeMaskForSlot(slot);
    FastGpio::digitalWrite(adc_sync, false);
    for (int board = 0; board < NUM_ADC_BOARDS; board++) {
      if (boardDepth[board] <= slot) continue;
      ADCController::selectContinuousConversionChannel(
          channelAtSlot[board][slot]);
    }

    __disable_irq();
    TimingUtil::adcFlag = 0;
    TimingUtil::adcConversionInProgressMask = expectedMask;
    TimingUtil::adcConversionWatchMask = expectedMask;
    TimingUtil::adcConversionStartedFlag = true;
    adcReadExpectedMask = expectedMask;
    __enable_irq();

    FastGpio::digitalWrite(adc_sync, true);
    return expectedMask;
  };

  auto readSlot = [&](int slot, double* packets) {
    for (int board = 0; board < NUM_ADC_BOARDS; board++) {
      if (boardDepth[board] <= slot) continue;
      const int channel = channelAtSlot[board][slot];
      const int outputIndex = outputIndexAtSlot[board][slot];
      packets[outputIndex] =
          ADCController::getVoltageDataNoTransaction(channel);
    }
  };

  auto cleanup = [&]() {
    FastGpio::digitalWrite(adc_sync, false);
    detachDataReadyInterrupts(boardMask);
    clearAdcReadFlags();
    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
      ADCController::unsetRDYFN(adcChannels[i]);
    }
    ADCController::resetToPreviousConversionTimes();
    PeripheralCommsController::dataLedOff();
  };

  clearWorkerStopRequest();
  PeripheralCommsController::dataLedOn();
  TimingUtil::resetTimers();
  ADCController::resetToPreviousConversionTimes();
  FastGpio::digitalWrite(adc_sync, false);
  clearAdcReadFlags();
  attachDataReadyInterrupts(boardMask);
  for (int i = 0; i < numAdcChannels; i++) {
    ADCController::startContinuousConversion(adcChannels[i]);
    ADCController::unsetRDYFN(adcChannels[i]);
  }

  int samplesCaptured = 0;
  int currentSlot = 0;
  AdcBoardMask expectedMask = startSlot(currentSlot);
  double packets[NUM_ADC_CHANNELS] = {};
  bool voltageOverflow = false;

  while (samplesCaptured < savedDataSize && !isWorkerStopRequested()) {
    if (!consumeExpectedAdcFlag(expectedMask)) {
      __WFE();
      continue;
    }

    const int completedSlot = currentSlot;
    const bool completedFrame = completedSlot + 1 >= maxDepth;
    const bool needNextConversion =
        !completedFrame || (samplesCaptured + 1 < savedDataSize);
    const int nextSlot = completedFrame ? 0 : completedSlot + 1;

    AdcBoardMask nextExpectedMask = 0;
    if (needNextConversion) {
      nextExpectedMask = startSlot(nextSlot);
    } else {
      FastGpio::digitalWrite(adc_sync, false);
    }

    readSlot(completedSlot, packets);

    if (completedFrame) {
      if (!sendVoltageFrame(packets, numAdcChannels)) {
        voltageOverflow = true;
        break;
      }
      samplesCaptured++;
    }

    currentSlot = nextSlot;
    expectedMask = nextExpectedMask;
  }

  cleanup();

  if (voltageOverflow) {
    if (isWorkerStopRequested()) clearWorkerStopRequest();
    return OperationResult::Failure("Voltage output buffer overflow");
  }
  if (isWorkerStopRequested()) {
    clearWorkerStopRequest();
    return OperationResult::Failure("RAMPING_STOPPED");
  }
  return OperationResult::Success();
}

OperationResult timeSeriesAdcRead(int numAdcChannels,
                                  List<int, 0>& adcChannelsList,
                                  float conversionTimeArg,
                                  float totalDurationArg) {
  return timeSeriesAdcReadImpl(numAdcChannels, adcChannelsList,
                               conversionTimeArg, totalDurationArg, true);
}
COMMAND("TIME_SERIES_ADC_READ", timeSeriesAdcRead)

OperationResult timeSeriesAdcReadSudo(int numAdcChannels,
                                      List<int, 0>& adcChannelsList,
                                      float conversionTimeArg,
                                      float totalDurationArg) {
  return timeSeriesAdcReadImpl(numAdcChannels, adcChannelsList,
                               conversionTimeArg, totalDurationArg, false);
}
COMMAND("TIME_SERIES_ADC_READ_SUDO", timeSeriesAdcReadSudo)

}  // namespace
