#include <Arduino.h>

#include "Config.h"
#include "Calibration/Calibration.h"
#include "Calibration/CalibrationFlash.h"
#include "FunctionRegistry/FunctionRegistry.h"
#include "shared_memory.h"
#include "UserIOHandler.h"

#if defined(ARDUINO_GIGA) || defined(CORE_STM32H7)
#include "stm32h7xx.h"
#else
#error "This code is intended for STM32H7 based boards like Arduino Giga."
#endif

constexpr char kFlashWriteFailure[] =
    "Failed to write calibration data to flash!";

static void configureSharedMemoryMpu() {
  HAL_MPU_Disable();

  MPU_Region_InitTypeDef MPU_InitStruct;
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.BaseAddress = SHARED_MEMORY_ADDRESS;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32KB;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER15;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.SubRegionDisable = 0x00;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
  __DSB();
  __ISB();
}

static void enableRccPeripheralClock(volatile uint32_t& enableRegister,
                                     uint32_t enableMask) {
  enableRegister |= enableMask;
  const uint32_t enabled = enableRegister & enableMask;
  (void)enabled;
}

static void prepareM4UsbClock() {
  __HAL_RCC_PWR_CLK_ENABLE();
  enableRccPeripheralClock(RCC->APB4ENR, RCC_APB4ENR_SYSCFGEN);
  __HAL_RCC_HSI48_ENABLE();
  enableRccPeripheralClock(RCC->APB1HENR, RCC_APB1HENR_CRSEN);

  uint32_t start = HAL_GetTick();
  while (__HAL_RCC_GET_FLAG(RCC_FLAG_HSI48RDY) == RESET &&
         (HAL_GetTick() - start) < 10) {
  }

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {};
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  (void)HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  RCC_CRSInitTypeDef CRSInitStruct = {};
  CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB2;
  CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  CRSInitStruct.ReloadValue = RCC_CRS_RELOADVALUE_DEFAULT;
  CRSInitStruct.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
  CRSInitStruct.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
  HAL_RCCEx_CRSConfig(&CRSInitStruct);

  HAL_PWREx_EnableUSBVoltageDetector();
}

void enableM4() {
  FLASH_OBProgramInitTypeDef OBInit;

  OBInit.Banks = FLASH_BANK_1;
  HAL_FLASHEx_OBGetConfig(&OBInit);
  if (OBInit.USERConfig & FLASH_OPTSR_BCM4) {
    OBInit.OptionType = OPTIONBYTE_USER;
    OBInit.USERType = OB_USER_BCM4;
    OBInit.USERConfig = 0;
    if (HAL_FLASH_OB_Unlock() != HAL_OK) return;
    if (HAL_FLASH_Unlock() != HAL_OK) return;
    if (HAL_FLASHEx_OBProgram(&OBInit) != HAL_OK) return;
    if (HAL_FLASH_OB_Launch() != HAL_OK) return;
    if (HAL_FLASH_OB_Lock() != HAL_OK) return;
    if (HAL_FLASH_Lock() != HAL_OK) return;
    NVIC_SystemReset();
    return;
  }

  prepareM4UsbClock();

  LL_SYSCFG_SetCM4BootAddress0(CM4_BINARY_START >> 16);
  LL_RCC_ForceCM4Boot();
}

static CalibrationData loadCalibrationData() {
  CalibrationData calibration_data = {};
  const bool loadedFromFlash = readCalibrationFromFlash(calibration_data);
  CalibrationRegistry::prepare(calibration_data, loadedFromFlash);
  return calibration_data;
}

void setup() {
  configureSharedMemoryMpu();

  if (!initSharedMemory()) {
    while (1) {
      delay(1000);
    }
  }

  enableM4();

  CalibrationData calibration_data = loadCalibrationData();
  publishCalibrationData(calibration_data);

  SetupRegistry::runAll();
}

void loop() {
  UserIOHandler::handleUserIO();

  if (isCalibrationDataUpdated()) {
    CalibrationData calibration_data;
    readCalibrationData(calibration_data);
    if (!writeCalibrationToFlash(calibration_data)) {
      sendTextToGateway(kFlashWriteFailure, sizeof(kFlashWriteFailure));
    }
    clearCalibrationDataUpdated();
  }
}
