#pragma once

#include <cstddef>
#include <cstdint>

#include "CalibrationData.h"

// Fixed flash memory address for storing calibration (15 MB offset in 16 MB QSPI
// flash). This avoids the WiFi firmware and OTA regions.
static constexpr uint32_t CALIBRATION_FLASH_ADDR = 15 * 1024 * 1024;

uint32_t calculateCRC32(const uint8_t* data, size_t length);
bool writeCalibrationToFlash(const CalibrationData& data);
bool readCalibrationFromFlash(CalibrationData& data);

