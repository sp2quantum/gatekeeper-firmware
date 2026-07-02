#include "Utils/TimingUtil.h"

#include "Config.h"
#include "PeripheralCommsController.h"
#include "Utils/FastGpio.h"

volatile AdcBoardMask TimingUtil::adcFlag = 0;
volatile bool TimingUtil::dacFlag = false;
volatile uint32_t TimingUtil::dacFlagCount = 0;
volatile uint32_t TimingUtil::dacSpiMisstepEvents = 0;
volatile uint32_t TimingUtil::adcSpiMisstepEvents = 0;
volatile uint32_t TimingUtil::adcConversionMisstepEvents = 0;
volatile AdcBoardMask TimingUtil::adcConversionInProgressMask = 0;
volatile AdcBoardMask TimingUtil::adcConversionWatchMask = 0;
volatile bool TimingUtil::adcConversionStartedFlag = false;

namespace {
constexpr uint32_t kTimeSeriesAdcStartDelayUs = 10;

void enableTimerClock(uint32_t clock_enable_bits) {
  RCC->APB2ENR |= clock_enable_bits;
  const uint32_t apb2enr = RCC->APB2ENR;
  (void)apb2enr;
  __DMB();
}

bool spiStillClocking(SPI_TypeDef* spi, volatile bool& inProgress) {
  if (!inProgress) {
    return false;
  }
  const uint32_t status = spi->SR;
  if ((status & SPI_SR_EOT) != 0 || (status & SPI_SR_TXC) != 0) {
    inProgress = false;
    return false;
  }
  return true;
}

void handleAdcTimerStartEvent() {
  if (spiStillClocking(SPI5,
                       PeripheralCommsController::adcSpiTransferInProgress)) {
    TimingUtil::adcSpiMisstepEvents++;
  }
  if (TimingUtil::adcConversionInProgressMask != 0) {
    TimingUtil::adcConversionMisstepEvents++;
  }
  TimingUtil::adcConversionInProgressMask =
      TimingUtil::adcConversionWatchMask;
  FastGpio::digitalWrite(adc_sync, true);
  if (TimingUtil::adcConversionWatchMask != 0) {
    TimingUtil::adcConversionStartedFlag = true;
    __SEV();
  }
}

struct TimerPeriod {
  uint16_t prescaler;
  uint16_t autoReload;
};

TimerPeriod timerPeriodForMicros(uint64_t periodUs, uint64_t timerClock) {
  const uint64_t totalTicks = (periodUs * timerClock) / 1000000;
  if (totalTicks <= 65536) {
    return {0, static_cast<uint16_t>(totalTicks - 1)};
  }

  const uint32_t prescaler = (totalTicks + 65536 - 1) / 65536;
  return {static_cast<uint16_t>(prescaler - 1),
          static_cast<uint16_t>((totalTicks / prescaler) - 1)};
}
}

void TimingUtil::resetTimers() {
  __disable_irq();

  __HAL_RCC_TIM1_FORCE_RESET();
  __HAL_RCC_TIM8_FORCE_RESET();
  __HAL_RCC_TIM1_RELEASE_RESET();
  __HAL_RCC_TIM8_RELEASE_RESET();

  __HAL_RCC_TIM1_CLK_DISABLE();
  __HAL_RCC_TIM8_CLK_DISABLE();

  NVIC_DisableIRQ(TIM1_UP_IRQn);
  NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
  NVIC_DisableIRQ(TIM8_CC_IRQn);
  NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
  NVIC_ClearPendingIRQ(TIM8_UP_TIM13_IRQn);
  NVIC_ClearPendingIRQ(TIM8_CC_IRQn);

  adcFlag = 0;
  dacFlag = false;
  dacFlagCount = 0;
  adcConversionInProgressMask = 0;
  adcConversionWatchMask = 0;
  adcConversionStartedFlag = false;

  __enable_irq();
  delayMicroseconds(5);
}

void TimingUtil::resetTimingWatchdog(AdcBoardMask adc_watch_mask) {
  __disable_irq();
  dacSpiMisstepEvents = 0;
  adcSpiMisstepEvents = 0;
  adcConversionMisstepEvents = 0;
  adcConversionInProgressMask = 0;
  adcConversionWatchMask = adc_watch_mask;
  adcConversionStartedFlag = false;
  __enable_irq();
}

void TimingUtil::stopAndResetAdcTimer() {
  TIM8->CR1 &= ~TIM_CR1_CEN;
  TIM8->CNT = 0;
}

void TimingUtil::startAdcTimer() {
  TIM8->CR1 |= TIM_CR1_CEN;
}

void TimingUtil::stopTimeSeriesTimers() {
  disableDacInterrupt();
  disableAdcInterrupt();
  dacFlag = false;
  dacFlagCount = 0;
  adcFlag = 0;
  adcConversionInProgressMask = 0;
  adcConversionStartedFlag = false;
  stopAndResetAdcTimer();
  FastGpio::digitalWrite(adc_sync, false);
}

void TimingUtil::setupTimerOnlyDac(uint32_t period_us) {
  resetTimers();
  resetTimingWatchdog();

  enableTimerClock(RCC_APB2ENR_TIM1EN);

  const uint64_t timerClock = 2 * HAL_RCC_GetPCLK2Freq();
  const TimerPeriod dacPeriod = timerPeriodForMicros(period_us, timerClock);

  TIM1->CR1 &= ~TIM_CR1_CEN;
  TIM1->PSC = dacPeriod.prescaler;
  TIM1->ARR = dacPeriod.autoReload;
  TIM1->CR1 = TIM_CR1_ARPE;
  TIM1->DIER |= TIM_DIER_UIE;

  NVIC_SetPriority(TIM1_UP_IRQn, 0);
  NVIC_EnableIRQ(TIM1_UP_IRQn);

  TIM1->CR1 |= TIM_CR1_CEN;
}

void TimingUtil::setupTimersOnlyADC(uint32_t adc_period_us) {
  resetTimers();
  resetTimingWatchdog();

  enableTimerClock(RCC_APB2ENR_TIM8EN);

  const uint64_t timerClock = 2 * HAL_RCC_GetPCLK2Freq();
  const TimerPeriod adcPeriod = timerPeriodForMicros(adc_period_us, timerClock);

  TIM8->CR1 &= ~TIM_CR1_CEN;
  TIM8->PSC = adcPeriod.prescaler;
  TIM8->ARR = adcPeriod.autoReload;
  TIM8->CR1 = TIM_CR1_ARPE;
  TIM8->DIER |= TIM_DIER_UIE;

  TIM8->EGR |= 0x01;
  TIM8->SR &= ~TIM_SR_UIF;

  NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 3);
  NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

  TIM8->CR1 |= TIM_CR1_CEN;
}

void TimingUtil::setupTimersTimeSeries(uint32_t dac_period_us,
                                       uint32_t adc_period_us,
                                       AdcBoardMask adc_watch_mask) {
  resetTimers();
  resetTimingWatchdog(adc_watch_mask);

  enableTimerClock(RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN);

  const uint64_t timerClock = 2 * HAL_RCC_GetPCLK2Freq();
  const TimerPeriod dacPeriod =
      timerPeriodForMicros(dac_period_us, timerClock);
  const TimerPeriod adcPeriod =
      timerPeriodForMicros(adc_period_us, timerClock);

  TIM1->CR1 &= ~TIM_CR1_CEN;
  TIM1->PSC = dacPeriod.prescaler;
  TIM1->ARR = dacPeriod.autoReload;
  TIM1->CR1 = TIM_CR1_ARPE;
  TIM1->DIER |= TIM_DIER_UIE;

  TIM1->EGR |= 0x01;
  TIM1->SR &= ~TIM_SR_UIF;

  TIM8->CR1 &= ~TIM_CR1_CEN;
  TIM8->PSC = adcPeriod.prescaler;
  TIM8->ARR = adcPeriod.autoReload;
  TIM8->CR1 = TIM_CR1_ARPE;
  TIM8->DIER |= TIM_DIER_UIE;

  TIM8->EGR |= 0x01;
  TIM8->SR &= ~TIM_SR_UIF;

  NVIC_SetPriority(TIM1_UP_IRQn, 0);
  NVIC_EnableIRQ(TIM1_UP_IRQn);
  NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 3);
  NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

  TIM1->CR1 |= TIM_CR1_CEN;
  TIM8->CR1 |= TIM_CR1_CEN;
}

void TimingUtil::setupTimersTimeSeriesRamp(uint32_t dac_period_us,
                                           uint32_t adc_period_us,
                                           AdcBoardMask adc_watch_mask) {
  if (dac_period_us == adc_period_us &&
      kTimeSeriesAdcStartDelayUs < adc_period_us) {
    setupTimersDacLed(dac_period_us, kTimeSeriesAdcStartDelayUs,
                      adc_watch_mask);
    return;
  }
  setupTimersTimeSeries(dac_period_us, adc_period_us, adc_watch_mask);
}

void TimingUtil::setupTimersDacLed(uint64_t period_us,
                                   uint64_t phase_shift_us,
                                   AdcBoardMask adc_watch_mask) {
  resetTimers();
  resetTimingWatchdog(adc_watch_mask);

  enableTimerClock(RCC_APB2ENR_TIM1EN | RCC_APB2ENR_TIM8EN);

  const uint64_t timerClock = 2 * HAL_RCC_GetPCLK2Freq();
  const TimerPeriod period = timerPeriodForMicros(period_us, timerClock);

  TIM1->PSC = period.prescaler;
  TIM1->ARR = period.autoReload;
  TIM1->CR1 = TIM_CR1_ARPE;
  TIM1->CNT = 0;

  TIM1->CR2 &= ~TIM_CR2_MMS;
  TIM1->CR2 |= TIM_CR2_MMS_1;
  TIM1->DIER |= TIM_DIER_UIE;

  TIM8->PSC = period.prescaler;
  TIM8->ARR = period.autoReload;
  TIM8->CR1 = TIM_CR1_ARPE;
  TIM8->CNT = 0;

  TIM8->SMCR &= ~TIM_SMCR_TS;
  TIM8->SMCR &= ~TIM_SMCR_SMS;
  TIM8->SMCR |= TIM_SMCR_SMS_3;

  if (phase_shift_us > 0 && phase_shift_us < period_us) {
    uint32_t timerPhaseShift = (phase_shift_us * (TIM8->ARR + 1)) / period_us;
    TIM8->CCR1 = timerPhaseShift;
    TIM8->CCMR1 &= ~TIM_CCMR1_OC1M;
    TIM8->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
    TIM8->CCER |= TIM_CCER_CC1E;
    TIM8->DIER |= TIM_DIER_CC1IE;
  } else {
    TIM8->DIER |= TIM_DIER_UIE;
  }

  TIM1->CR1 &= ~TIM_CR1_CEN;
  TIM1->EGR |= 0x01;
  TIM1->SR &= ~TIM_SR_UIF;
  TIM1->EGR |= 0x02;
  TIM1->SR &= ~TIM_SR_CC1IF;

  TIM8->CR1 &= ~TIM_CR1_CEN;
  TIM8->EGR |= 0x01;
  TIM8->SR &= ~TIM_SR_UIF;
  TIM8->EGR |= 0x02;
  TIM8->SR &= ~TIM_SR_CC1IF;

  NVIC_SetPriority(TIM1_UP_IRQn, 0);
  NVIC_EnableIRQ(TIM1_UP_IRQn);

  NVIC_SetPriority(TIM8_CC_IRQn, 3);
  NVIC_EnableIRQ(TIM8_CC_IRQn);

  TIM1->CR1 |= TIM_CR1_CEN;
}

void TimingUtil::disableDacInterrupt() {
  TIM1->DIER &= ~TIM_DIER_UIE;
  NVIC_DisableIRQ(TIM1_UP_IRQn);
}

void TimingUtil::disableAdcInterrupt() {
  TIM8->DIER &= ~TIM_DIER_UIE;
  TIM8->DIER &= ~TIM_DIER_CC1IE;
  NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
  NVIC_DisableIRQ(TIM8_CC_IRQn);
}

bool TimingUtil::consumeDacFlag() {
  if (dacFlagCount == 0) {
    return false;
  }
  __disable_irq();
  const bool pending = dacFlagCount > 0;
  if (pending) {
    dacFlagCount--;
    dacFlag = dacFlagCount > 0;
  }
  __enable_irq();
  return pending;
}

bool TimingUtil::consumeAdcFlag(AdcBoardMask expectedMask) {
  if (adcFlag != expectedMask) {
    return false;
  }
  __disable_irq();
  const bool pending = adcFlag == expectedMask;
  if (pending) {
    adcFlag = 0;
  }
  __enable_irq();
  return pending;
}

bool TimingUtil::consumeAnyAdcFlag() {
  if (!adcFlag) {
    return false;
  }
  __disable_irq();
  const bool pending = adcFlag != 0;
  if (pending) {
    adcFlag = 0;
  }
  __enable_irq();
  return pending;
}

bool TimingUtil::consumeAdcConversionStartedFlag() {
  if (!adcConversionStartedFlag) {
    return false;
  }
  __disable_irq();
  const bool pending = adcConversionStartedFlag;
  adcConversionStartedFlag = false;
  __enable_irq();
  return pending;
}

extern "C" void TIM1_UP_IRQHandler(void) {
  if (TIM1->SR & TIM_SR_UIF) {
    TIM1->SR &= ~TIM_SR_UIF;
    if (spiStillClocking(SPI1,
                         PeripheralCommsController::dacSpiTransferInProgress)) {
      TimingUtil::dacSpiMisstepEvents++;
    }
    if (TimingUtil::adcConversionInProgressMask != 0) {
      TimingUtil::adcConversionMisstepEvents++;
    }
    FastGpio::pulseLowHigh(ldac);
    TimingUtil::dacFlagCount++;
    TimingUtil::dacFlag = true;
    __SEV();
  }
}

extern "C" void TIM8_UP_TIM13_IRQHandler(void) {
  if (TIM8->SR & TIM_SR_UIF) {
    TIM8->SR &= ~TIM_SR_UIF;
    handleAdcTimerStartEvent();
  }
}

extern "C" void TIM8_CC_IRQHandler(void) {
  if (TIM8->SR & TIM_SR_CC1IF) {
    TIM8->SR &= ~TIM_SR_CC1IF;
    handleAdcTimerStartEvent();
  }
}
