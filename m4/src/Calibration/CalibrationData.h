#pragma once

#include <stddef.h>
#include <stdint.h>

constexpr uint32_t kCalibrationDataMagic = 0x474B4341UL;  // "GKCA"
constexpr uint16_t kCalibrationDataVersion = 1;
constexpr size_t kCalibrationPayloadCapacity = 512;

struct CalibrationData {
  uint32_t magic;
  uint16_t version;
  uint16_t payloadSize;
  alignas(8) uint8_t payload[kCalibrationPayloadCapacity];
};

