#pragma once

#include <stddef.h>
#include <stdint.h>

#include "CalibrationData.h"
#include "Utils/OperationResult.h"

namespace CalibrationRegistry {

using DefaultsCallback = void (*)(void* section);
using ValidateCallback = bool (*)(const void* section);

uint32_t sectionId(const char* name);
void registerSection(const char* name, size_t size,
                     DefaultsCallback setDefaults,
                     ValidateCallback validate);

void prepare(CalibrationData& data, bool loadedFromFlash);
void resetToDefaults(CalibrationData& data);

void* getSection(CalibrationData& data, uint32_t id, size_t expectedSize);
const void* getSection(const CalibrationData& data, uint32_t id,
                       size_t expectedSize);

OperationResult hardResetToDefaults();

}  // namespace CalibrationRegistry

#define _CAL_CAT2(a, b) a##b
#define _CAL_CAT(a, b) _CAL_CAT2(a, b)

#define CALIBRATION_SECTION(name, type, defaults, validate) \
  namespace { auto _CAL_CAT(_cal_section_, __COUNTER__) = \
      (CalibrationRegistry::registerSection(name, sizeof(type), defaults, \
                                            validate), 0); }
