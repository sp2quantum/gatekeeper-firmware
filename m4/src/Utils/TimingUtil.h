#pragma once

#include "stm32h7xx.h"

struct TimingUtil {
  inline static volatile bool adcFlag = false;
  inline static volatile bool dacFlag = false;

  inline static void resetTimers() {
    // Disable all interrupts
    __disable_irq();

    // Reset TIM1 and TIM8
    __HAL_RCC_TIM1_FORCE_RESET();
    __HAL_RCC_TIM8_FORCE_RESET();
    __HAL_RCC_TIM1_RELEASE_RESET();
    __HAL_RCC_TIM8_RELEASE_RESET();

    // Disable TIM1 and TIM8 clocks
    __HAL_RCC_TIM1_CLK_DISABLE();
    __HAL_RCC_TIM8_CLK_DISABLE();

    // Disable and clear all relevant interrupts
    NVIC_DisableIRQ(TIM1_UP_IRQn);
    NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
    NVIC_DisableIRQ(TIM8_CC_IRQn);
    NVIC_ClearPendingIRQ(TIM1_UP_IRQn);
    NVIC_ClearPendingIRQ(TIM8_UP_TIM13_IRQn);
    NVIC_ClearPendingIRQ(TIM8_CC_IRQn);

    // Reset flags
    adcFlag = false;
    dacFlag = false;

    // Re-enable interrupts
    __enable_irq();
    delayMicroseconds(5);
  }

  inline static void setupTimerOnlyDac(uint32_t period_us) {
    resetTimers();

    // Enable TIM1 clock
    __HAL_RCC_TIM1_CLK_ENABLE();

    uint32_t timerClock = HAL_RCC_GetPCLK2Freq();

    // Configure TIM1
    TIM1->PSC = (2 * timerClock / 1000000) - 1;  // For 1us resolution
    TIM1->ARR = period_us - 1;
    TIM1->CR1 = TIM_CR1_ARPE;
    TIM1->DIER |= TIM_DIER_UIE;

    // Enable interrupts
    NVIC_SetPriority(TIM1_UP_IRQn, 2);
    NVIC_EnableIRQ(TIM1_UP_IRQn);

    // Start timer
    TIM1->CR1 |= TIM_CR1_CEN;
  }

  inline static void setupTimersTimeSeries(uint32_t dac_period_us,
                                           uint32_t adc_period_us) {
    resetTimers();

    // Enable TIM1 and TIM8 clocks
   __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    uint32_t timerClock = HAL_RCC_GetPCLK2Freq();

    // Configure TIM1
    TIM1->PSC = (2 * timerClock / 1000000) - 1;  // For 1us resolution
    TIM1->ARR = dac_period_us - 1;
    TIM1->CR1 = TIM_CR1_ARPE;
    TIM1->DIER |= TIM_DIER_UIE;

    // Configure TIM8
    TIM8->PSC = (2 * timerClock / 1000000) - 1;  // For 1us resolution
    TIM8->ARR = adc_period_us - 1;
    TIM8->CR1 = TIM_CR1_ARPE;
    TIM8->DIER |= TIM_DIER_UIE;

    // Enable interrupts
    NVIC_SetPriority(TIM1_UP_IRQn, 2);
    NVIC_EnableIRQ(TIM1_UP_IRQn);
    NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 3);
    NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);

    // Start timers
    TIM1->CR1 |= TIM_CR1_CEN;
    TIM8->CR1 |= TIM_CR1_CEN;
  }


  inline static void setupTimersDacLed(uint32_t period_us, uint32_t phase_shift_us) {
    // Reset timers and clear flags
    resetTimers();
    // Enable TIM1 and TIM8 clocks
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    uint32_t timerClock = HAL_RCC_GetPCLK2Freq();
    //
    // Configure TIM1 as Master (DAC trigger)
    //
    TIM1->PSC = (2 * timerClock / 1000000) - 1;  // For 1µs resolution
    TIM1->ARR = period_us - 1;
    TIM1->CR1 = TIM_CR1_ARPE;
    // Set Master Mode: Update event (TRGO) on each overflow
    TIM1->CR2 = (TIM1->CR2 & ~TIM_CR2_MMS) | TIM_CR2_MMS_1;
    // Enable update interrupt (if needed for other purposes)
    TIM1->DIER |= TIM_DIER_UIE;
    //
    // Configure TIM8 as Slave (ADC trigger) in RESET mode
    //
    TIM8->PSC = (2 * timerClock / 1000000) - 1;  // For 1µs resolution
    TIM8->ARR = period_us - 1;
    TIM8->CR1 = TIM_CR1_ARPE;
    // Clear the trigger selection (TS) bits.
    // For ITR0 (which is TIM1) on many STM32 MCUs TS should be 0.
    TIM8->SMCR &= ~(TIM_SMCR_TS);
    // Set the slave mode selection (SMS) bits to 0b100 (Reset Mode)
    // (This makes TIM8 reset its counter when the trigger occurs.)
    TIM8->SMCR &= ~(TIM_SMCR_SMS);
    TIM8->SMCR |= TIM_SMCR_SMS_2;  // Assuming TIM_SMCR_SMS_2 corresponds to 0b100
    // Optional: If you require a phase shift for the ADC trigger, configure channel 1.
    if (phase_shift_us > 0 && phase_shift_us < period_us) {
      // Set compare register to trigger ADC a few microseconds after reset.
      TIM8->CCR1 = phase_shift_us + 2;
      // Configure Output Compare for Channel 1 to PWM mode 1.
      TIM8->CCMR1 &= ~(TIM_CCMR1_OC1M);  // Clear OC1M bits
      TIM8->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;
      // Enable the Capture/Compare 1 output.
      TIM8->CCER |= TIM_CCER_CC1E;
      // Enable Capture/Compare 1 interrupt.
      TIM8->DIER |= TIM_DIER_CC1IE;
    }
    // Explicitly clear TIM8 counter to guarantee it starts at 0.
    TIM8->CNT = 0;
    //
    // Configure NVIC priorities and enable interrupts
    //
    NVIC_SetPriority(TIM1_UP_IRQn, 2);
    NVIC_EnableIRQ(TIM1_UP_IRQn);
    NVIC_SetPriority(TIM8_CC_IRQn, 3);
    NVIC_EnableIRQ(TIM8_CC_IRQn);
    //
    // Start the timers: start the slave (TIM8) first, then the master (TIM1)
    //
    TIM8->CR1 |= TIM_CR1_CEN;
    TIM1->CR1 |= TIM_CR1_CEN;
  }

  inline static void disableDacInterrupt() {
    TIM1->DIER &= ~TIM_DIER_UIE;
    NVIC_DisableIRQ(TIM1_UP_IRQn);
  }

  inline static void disableAdcInterrupt() {
    TIM8->DIER &= ~TIM_DIER_UIE;
    NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
    NVIC_DisableIRQ(TIM8_CC_IRQn);
  }
};

extern "C" void TIM1_UP_IRQHandler(void) {
  if (TIM1->SR & TIM_SR_UIF) {
    TIM1->SR &= ~TIM_SR_UIF;
    TimingUtil::dacFlag = true;
  }
}

extern "C" void TIM8_UP_TIM13_IRQHandler(void) {
  if (TIM8->SR & TIM_SR_UIF) {
    TIM8->SR &= ~TIM_SR_UIF;
    TimingUtil::adcFlag = true;
  }
}

extern "C" void TIM8_CC_IRQHandler(void) {
  if (TIM8->SR & TIM_SR_CC1IF) {
    TIM8->SR &= ~TIM_SR_CC1IF;
    TimingUtil::adcFlag = true;
  }
}