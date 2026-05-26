#include "Calibration.h"

#include <Arduino.h>

#include <string.h>
#include <vector>

#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "shared_memory.h"

namespace {

struct SectionHeader {
  uint32_t id;
  uint16_t size;
  uint16_t reserved;
};

struct SectionEntry {
  uint32_t id;
  const char* name;
  uint16_t size;
  CalibrationRegistry::DefaultsCallback setDefaults;
  CalibrationRegistry::ValidateCallback validate;
};

std::vector<SectionEntry>& sections() {
  static std::vector<SectionEntry> entries;
  return entries;
}

uint16_t align4(uint16_t value) { return (value + 3u) & ~3u; }

bool isValidHeader(const CalibrationData& data) {
  return data.magic == kCalibrationDataMagic &&
         data.version == kCalibrationDataVersion &&
         data.payloadSize <= kCalibrationPayloadCapacity;
}

const SectionHeader* findSectionHeader(const CalibrationData& data,
                                       uint32_t id) {
  if (!isValidHeader(data)) return nullptr;

  uint16_t offset = 0;
  while (offset + sizeof(SectionHeader) <= data.payloadSize) {
    const auto* header =
        reinterpret_cast<const SectionHeader*>(&data.payload[offset]);
    const uint16_t recordSize =
        align4(static_cast<uint16_t>(sizeof(SectionHeader) + header->size));
    if (header->size == 0 || offset + recordSize > data.payloadSize) {
      return nullptr;
    }
    if (header->id == id) return header;
    offset += recordSize;
  }
  return nullptr;
}

void* appendSection(CalibrationData& data, const SectionEntry& entry) {
  const uint16_t recordSize =
      align4(static_cast<uint16_t>(sizeof(SectionHeader) + entry.size));
  if (data.payloadSize + recordSize > kCalibrationPayloadCapacity) {
    return nullptr;
  }

  auto* header = reinterpret_cast<SectionHeader*>(&data.payload[data.payloadSize]);
  header->id = entry.id;
  header->size = entry.size;
  header->reserved = 0;

  void* section = &data.payload[data.payloadSize + sizeof(SectionHeader)];
  memset(section, 0, entry.size);
  data.payloadSize += recordSize;
  return section;
}

OperationResult hardResetCalibrationToDefaults() {
  CalibrationData data = {};
  CalibrationRegistry::resetToDefaults(data);
  updateCalibrationData(data);
  return OperationResult::Success("Calibration data reset to defaults");
}
COMMAND("HARD_RESET_CALIBRATION", hardResetCalibrationToDefaults)

}  // namespace

namespace CalibrationRegistry {

uint32_t sectionId(const char* name) {
  uint32_t hash = 2166136261UL;
  while (*name) {
    hash ^= static_cast<uint8_t>(*name++);
    hash *= 16777619UL;
  }
  return hash;
}

void registerSection(const char* name, size_t size,
                     DefaultsCallback setDefaults,
                     ValidateCallback validate) {
  sections().push_back({sectionId(name), name, static_cast<uint16_t>(size),
                        setDefaults, validate});
}

void resetToDefaults(CalibrationData& data) {
  memset(&data, 0, sizeof(data));
  data.magic = kCalibrationDataMagic;
  data.version = kCalibrationDataVersion;
  data.payloadSize = 0;

  for (const auto& entry : sections()) {
    void* section = appendSection(data, entry);
    if (section && entry.setDefaults) entry.setDefaults(section);
  }
}

void prepare(CalibrationData& data, bool loadedFromFlash) {
  CalibrationData old = data;
  resetToDefaults(data);

  if (!loadedFromFlash || !isValidHeader(old)) return;

  for (const auto& entry : sections()) {
    const void* oldSection = getSection(old, entry.id, entry.size);
    void* newSection = getSection(data, entry.id, entry.size);
    if (!oldSection || !newSection) continue;

    memcpy(newSection, oldSection, entry.size);
    if (entry.validate && !entry.validate(newSection) && entry.setDefaults) {
      entry.setDefaults(newSection);
    }
  }
}

void* getSection(CalibrationData& data, uint32_t id, size_t expectedSize) {
  const SectionHeader* header = findSectionHeader(data, id);
  if (!header || header->size != expectedSize) return nullptr;
  return reinterpret_cast<uint8_t*>(const_cast<SectionHeader*>(header)) +
         sizeof(SectionHeader);
}

const void* getSection(const CalibrationData& data, uint32_t id,
                       size_t expectedSize) {
  const SectionHeader* header = findSectionHeader(data, id);
  if (!header || header->size != expectedSize) return nullptr;
  return reinterpret_cast<const uint8_t*>(header) + sizeof(SectionHeader);
}

OperationResult hardResetToDefaults() { return hardResetCalibrationToDefaults(); }

}  // namespace CalibrationRegistry
