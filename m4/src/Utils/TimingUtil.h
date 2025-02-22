#pragma once

#include "stm32h7xx.h"
#include "Config.h"

struct TimingUtil {
  inline static volatile uint8_t adcFlag = 8;
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
    adcFlag = 0;
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

    TIM1->EGR |= 0x01;
    TIM1->SR &= ~TIM_SR_UIF;
    TIM1->EGR |= 0x02;
    TIM1->CCR1 &= ~TIM_SR_CC1IF;

    // Configure TIM8
    TIM8->PSC = (2 * timerClock / 1000000) - 1;  // For 1us resolution
    TIM8->ARR = adc_period_us - 1;
    TIM8->CR1 = TIM_CR1_ARPE;
    TIM8->DIER |= TIM_DIER_UIE;

    TIM8->EGR |= 0x01;
    TIM8->SR &= ~TIM_SR_UIF;
    TIM8->EGR |= 0x02;
    TIM8->CCR1 &= ~TIM_SR_CC1IF;

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
    // Reset timers and clear prior configuration
    resetTimers();
    // Enable clocks for TIM1 (master) and TIM8 (slave)
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();

    uint32_t timerClock = HAL_RCC_GetPCLK2Freq();
    // --- Configure TIM1 as Master (for DAC triggering) ---
    TIM1->PSC = (2 * timerClock / 1000000) - 1;  // 1 µs resolution
    TIM1->ARR = period_us - 1;
    TIM1->CR1 = TIM_CR1_ARPE;
    TIM1->CNT = 0;  // Start at 0

    // Clear previous MMS bits then set master mode to “Update event” (TRGO)
    TIM1->CR2 &= ~TIM_CR2_MMS;
    TIM1->CR2 |= TIM_CR2_MMS_1;
    TIM1->DIER |= TIM_DIER_UIE;  // (Enable update interrupt if needed)

    // --- Configure TIM8 as Slave (for ADC triggering) ---
    TIM8->PSC = (2 * timerClock / 1000000) - 1;  // 1 µs resolution
    TIM8->ARR = period_us;
    TIM8->CR1 = TIM_CR1_ARPE;
    TIM8->CNT = 0;  // Start at 0

    // --- Set up TIM8 slave mode to use TIM1’s trigger ---
    // Clear TS bits to select ITR0 (which is 0 on STM32H7 devices)
    TIM8->SMCR &= ~TIM_SMCR_TS;
    // Clear the SMS bits...
    TIM8->SMCR &= ~TIM_SMCR_SMS;
    // ...and set SMS bits to 0b100 (Reset Mode)
    TIM8->SMCR |= TIM_SMCR_SMS_2;  // (Assuming SMS_2 represents the bit for value 4)
    // --- Configure phase shift if requested ---
    if (phase_shift_us > 0 && phase_shift_us < period_us) {
      // Use Channel 1 compare event for phase-shifted ADC trigger
      TIM8->CCR1 = phase_shift_us - 1;  // Adjust as needed
      // Set Channel 1 to PWM mode 1: clear then set OC1M bits
      TIM8->CCMR1 &= ~TIM_CCMR1_OC1M;
      TIM8->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; 
      TIM8->CCER |= TIM_CCER_CC1E;      // Enable CC1 output
      // Enable only the compare interrupt (disable UIE if not needed)
      TIM8->DIER &= ~TIM_DIER_UIE;
      TIM8->DIER |= TIM_DIER_CC1IE;
    } else {
      // No phase shift: use the update interrupt for ADC triggering.
      TIM8->DIER |= TIM_DIER_UIE;
    }
    // --- Finalize and start the timers ---
    // --- NVIC Setup ---
    //Clear UIF flags and reset UG bits
    TIM1->EGR |= 0x01;
    TIM1->SR &= ~TIM_SR_UIF;
    TIM1->EGR |= 0x02;
    TIM1->CCR1 &= ~TIM_SR_CC1IF;

    NVIC_SetPriority(TIM1_UP_IRQn, 2);
    NVIC_EnableIRQ(TIM1_UP_IRQn);

    TIM8->EGR |= 0x01;
    TIM8->SR &= ~TIM_SR_UIF;
    TIM8->EGR |= 0x02;
    TIM8->CCR1 &= ~TIM_SR_CC1IF;

    NVIC_SetPriority(TIM8_CC_IRQn, 3);
    NVIC_EnableIRQ(TIM8_CC_IRQn);

    // Then start the master.
    TIM1->CR1 |= TIM_CR1_CEN;
    // Start the slave timer first so it’s waiting for TIM1’s trigger.
    TIM1->EGR |= 0x01;
    TIM1->SR &= ~TIM_SR_UIF;
    TIM8->CR1 |= TIM_CR1_CEN;
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

  template<int boardIndex>
  inline static void adcSyncISR() {
    adcFlag |= 1 << boardIndex;
    digitalWrite(adc_sync, LOW);
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
    // TimingUtil::adcFlag = true;
    digitalWrite(adc_sync, HIGH);
  }
}

extern "C" void TIM8_CC_IRQHandler(void) {
  if (TIM8->SR & TIM_SR_CC1IF) {
    TIM8->SR &= ~TIM_SR_CC1IF;
    // TimingUtil::adcFlag = true;
    digitalWrite(adc_sync, HIGH);
  }
}