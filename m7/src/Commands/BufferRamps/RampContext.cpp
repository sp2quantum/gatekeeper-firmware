#include "Commands/BufferRamps/RampContext.h"

#include "Commands/BufferRamps/BufferRampCommon.h"
#include "Peripherals/ADC/ADCController.h"
#include "Utils/FastGpio.h"
#include "PeripheralCommsController.h"
#include "Utils/TimingUtil.h"
#include "shared_memory.h"

namespace {

using AdcIsr = void (*)();

AdcIsr kAdcSyncIsrFunctions[NUM_ADC_BOARDS] = {
    TimingUtil::adcSyncISR<0>,
    TimingUtil::adcSyncISR<1>,
};

void attachAdcSyncInterrupts(const BoardUsage& boardUsage) {
  for (int i = 0; i < boardUsage.numBoards; i++) {
    const int pin = ADCController::getDataReadyPin(boardUsage.idx[i]);
    if (pin == NC) continue;
    attachInterrupt(digitalPinToInterrupt(pin), kAdcSyncIsrFunctions[i],
                    FALLING);
  }
}

void detachAdcSyncInterrupts(const BoardUsage& boardUsage) {
  for (int i = 0; i < boardUsage.numBoards; i++) {
    const int pin = ADCController::getDataReadyPin(boardUsage.idx[i]);
    if (pin == NC) continue;
    detachInterrupt(digitalPinToInterrupt(pin));
  }
}

uint8_t adcMaskForBoardUsage(const BoardUsage& boardUsage) {
  uint8_t mask = 0u;
  for (int i = 0; i < boardUsage.numBoards; i++) {
    mask |= 1 << i;
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

void RampContext::setupAdcHardware(int* adcChannels, int numAdcChannels) {
  adcChannels_ = adcChannels;
  numAdcChannels_ = numAdcChannels;
  hasAdc_ = true;

  ADCController::resetToPreviousConversionTimes();
  FastGpio::digitalWrite(adc_sync, false);

  boardUsage_ = getUsedAdcBoards(adcChannels, numAdcChannels);
  attachAdcSyncInterrupts(boardUsage_);
  adcMask_ = adcMaskForBoardUsage(boardUsage_);

  for (int i = 0; i < numAdcChannels; i++) {
    ADCController::startContinuousConversion(adcChannels[i]);
    ADCController::setRDYFN(adcChannels[i]);
  }

  TimingUtil::dacFlag = false;
  TimingUtil::adcFlag = 0;
}

OperationResult RampContext::beginDacAndAdc(int* adcChannels,
                                            int numAdcChannels) {
  clearWorkerStopRequest();
  PeripheralCommsController::dataLedOn();
  setupAdcHardware(adcChannels, numAdcChannels);
  begun_ = true;
  return OperationResult::Success();
}

OperationResult RampContext::beginAdcOnly(int* adcChannels,
                                          int numAdcChannels) {
  clearWorkerStopRequest();
  PeripheralCommsController::dataLedOn();
  setupAdcHardware(adcChannels, numAdcChannels);
  begun_ = true;
  return OperationResult::Success();
}

void RampContext::beginDacOnly() {
  clearWorkerStopRequest();
  PeripheralCommsController::dataLedOn();
  begun_ = true;
}

bool RampContext::stopped() const { return isWorkerStopRequested(); }

void RampContext::cleanup() {
  if (!begun_ || finished_) return;
  finished_ = true;

  TimingUtil::disableDacInterrupt();
  TimingUtil::disableAdcInterrupt();
  TimingUtil::dacFlag = false;
  TimingUtil::adcFlag = 0;

  if (hasAdc_) {
    for (int i = 0; i < numAdcChannels_; i++) {
      ADCController::idleMode(adcChannels_[i]);
      ADCController::unsetRDYFN(adcChannels_[i]);
    }
    ADCController::resetToPreviousConversionTimes();
    detachAdcSyncInterrupts(boardUsage_);
  }

  PeripheralCommsController::dataLedOff();
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

  if (checkTiming) {
    return BufferRampCommon::finishRampTimingWatchdog(checkAdcMissteps);
  }

  return OperationResult::Success();
}
