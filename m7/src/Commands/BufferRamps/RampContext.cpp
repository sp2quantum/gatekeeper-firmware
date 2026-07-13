#include "Commands/BufferRamps/RampContext.h"

#include <array>
#include <utility>

#include "Commands/BufferRamps/BufferRampCommon.h"
#include "Peripherals/ADC/ADCController.h"
#include "Utils/FastGpio.h"
#include "Utils/TimingUtil.h"
#include "shared_memory.h"

namespace {

using AdcIsr = void (*)();

template <size_t... Indices>
std::array<AdcIsr, sizeof...(Indices)> makeAdcSyncIsrFunctions(
    std::index_sequence<Indices...>) {
  return {{TimingUtil::adcSyncISR<Indices>...}};
}

const auto kAdcSyncIsrFunctions =
    makeAdcSyncIsrFunctions(std::make_index_sequence<NUM_ADC_BOARDS>{});

void attachAdcSyncInterrupts(const BoardUsage& boardUsage) {
  for (int i = 0; i < boardUsage.numBoards; i++) {
    const int pin = ADCController::getDataReadyPin(boardUsage.idx[i]);
    if (pin == NC) continue;
    AdcIsr isr = kAdcSyncIsrFunctions[i];
    attachInterrupt(digitalPinToInterrupt(pin), isr, FALLING);
  }
}

void detachAdcSyncInterrupts(const BoardUsage& boardUsage) {
  for (int i = 0; i < boardUsage.numBoards; i++) {
    const int pin = ADCController::getDataReadyPin(boardUsage.idx[i]);
    if (pin == NC) continue;
    detachInterrupt(digitalPinToInterrupt(pin));
  }
}

AdcBoardMask adcMaskForBoardUsage(const BoardUsage& boardUsage) {
  AdcBoardMask mask = 0;
  for (int i = 0; i < boardUsage.numBoards; i++) {
    mask |= TimingUtil::adcBoardBit(i);
  }
  return mask;
}

BoardUsage getUsedAdcBoards(const int* adcChannels, int numAdcChannels) {
  BoardUsage boardUsage;
  bool selected[NUM_ADC_BOARDS] = {};
  for (int i = 0; i < numAdcChannels; ++i) {
    const int ch = adcChannels[i];
    if (ch < 0 || ch >= NUM_ADC_CHANNELS) continue;
    const uint8_t board = BufferRampCommon::adcBoardForChannel(ch);
    if (board < NUM_ADC_BOARDS) selected[board] = true;
  }

  for (uint8_t board = 0; board < NUM_ADC_BOARDS; board++) {
    if (selected[board]) boardUsage.idx[boardUsage.numBoards++] = board;
  }
  return boardUsage;
}

}  // namespace

RampContext::RampContext() = default;

RampContext::~RampContext() { cleanup(); }

OperationResult RampContext::setupAdcHardware(int* adcChannels,
                                              int numAdcChannels) {
  adcChannels_ = adcChannels;
  numAdcChannels_ = numAdcChannels;
  hasAdc_ = true;

  if (!ADCController::resetToPreviousConversionTimes())
    return OperationResult::Failure("ADC reset failed");
  FastGpio::digitalWrite(adc_sync, false);

  boardUsage_ = getUsedAdcBoards(adcChannels, numAdcChannels);
  attachAdcSyncInterrupts(boardUsage_);
  adcMask_ = adcMaskForBoardUsage(boardUsage_);

  for (int i = 0; i < numAdcChannels; i++) {
    if (!ADCController::startContinuousConversion(adcChannels[i])) {
      return OperationResult::Failure("ADC continuous-conversion setup failed");
    }
    OperationResult readyResult = ADCController::setRDYFN(adcChannels[i]);
    if (!readyResult.isSuccess()) return readyResult;
  }

  TimingUtil::dacFlag = false;
  TimingUtil::dacFlagCount = 0;
  TimingUtil::adcFlag = 0;
  return OperationResult::Success();
}

OperationResult RampContext::beginDacAndAdc(int* adcChannels,
                                            int numAdcChannels) {
  clearWorkerStopRequest();
  begun_ = true;
  OperationResult setupResult = setupAdcHardware(adcChannels, numAdcChannels);
  if (!setupResult.isSuccess()) cleanup();
  return setupResult;
}

void RampContext::beginDacOnly() {
  clearWorkerStopRequest();
  begun_ = true;
}

bool RampContext::stopped() const { return isWorkerStopRequested(); }

void RampContext::cleanup() {
  if (!begun_ || finished_) return;
  finished_ = true;

  TimingUtil::disableDacInterrupt();
  TimingUtil::disableAdcInterrupt();
  TimingUtil::dacFlag = false;
  TimingUtil::dacFlagCount = 0;
  TimingUtil::adcFlag = 0;

  if (hasAdc_) {
    for (int i = 0; i < numAdcChannels_; i++) {
      if (!ADCController::idleMode(adcChannels_[i]).isSuccess())
        cleanupFailed_ = true;
      if (!ADCController::unsetRDYFN(adcChannels_[i]).isSuccess())
        cleanupFailed_ = true;
    }
    if (!ADCController::resetToPreviousConversionTimes()) cleanupFailed_ = true;
    detachAdcSyncInterrupts(boardUsage_);
  }

}

OperationResult RampContext::finish(OperationResult rampResult,
                                    bool checkTiming,
                                    bool checkAdcMissteps) {
  cleanup();

  if (!rampResult.isSuccess()) {
    if (isWorkerStopRequested()) clearWorkerStopRequest();
    return rampResult;
  }

  if (isWorkerStopRequested()) {
    clearWorkerStopRequest();
    return OperationResult::Failure("RAMPING_STOPPED");
  }
  if (cleanupFailed_) {
    return OperationResult::Failure("ADC cleanup failed");
  }

  if (checkTiming) {
    return BufferRampCommon::finishRampTimingWatchdog(checkAdcMissteps);
  }

  return OperationResult::Success();
}
