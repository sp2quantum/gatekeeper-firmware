#include "CalibrationFlash.h"

#include <Arduino.h>

#ifdef abs
#define GATEKEEPER_RESTORE_ARDUINO_ABS
#undef abs
#endif
#include "QSPIFBlockDevice.h"
#ifdef GATEKEEPER_RESTORE_ARDUINO_ABS
#define abs(x) ((x)>0?(x):-(x))
#undef GATEKEEPER_RESTORE_ARDUINO_ABS
#endif

#include <cstring>

uint32_t calculateCRC32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320 : crc >> 1;
    }
  }
  return crc ^ 0xFFFFFFFF;
}

bool writeCalibrationToFlash(const CalibrationData& data) {
  QSPIFBlockDevice qspi(QSPI_SO0, QSPI_SO1, QSPI_SO2, QSPI_SO3, QSPI_SCK,
                        QSPI_CS, QSPIF_POLARITY_MODE_1, 40000000);

  int qspiStatus = qspi.init();
  if (qspiStatus != 0) return false;

  uint8_t dataBuf[sizeof(CalibrationData)];
  memcpy(dataBuf, &data, sizeof(CalibrationData));
  const uint32_t crc = calculateCRC32(dataBuf, sizeof(CalibrationData));

  const size_t totalSize = sizeof(CalibrationData) + sizeof(crc);
  uint8_t writeBuf[totalSize];
  memcpy(writeBuf, dataBuf, sizeof(CalibrationData));
  memcpy(writeBuf + sizeof(CalibrationData), &crc, sizeof(crc));

  const size_t eraseSize = qspi.get_erase_size(CALIBRATION_FLASH_ADDR);
  const uint32_t alignedAddr =
      CALIBRATION_FLASH_ADDR - (CALIBRATION_FLASH_ADDR % eraseSize);
  qspiStatus = qspi.erase(alignedAddr, eraseSize);
  if (qspiStatus != 0) {
    qspi.deinit();
    return false;
  }

  qspiStatus = qspi.program(writeBuf, CALIBRATION_FLASH_ADDR, totalSize);
  qspi.deinit();
  return qspiStatus == 0;
}

bool readCalibrationFromFlash(CalibrationData& data) {
  QSPIFBlockDevice qspi(QSPI_SO0, QSPI_SO1, QSPI_SO2, QSPI_SO3, QSPI_SCK,
                        QSPI_CS, QSPIF_POLARITY_MODE_1, 40000000);

  int qspiStatus = qspi.init();
  if (qspiStatus != 0) return false;

  const size_t totalSize = sizeof(CalibrationData) + sizeof(uint32_t);
  uint8_t readBuf[totalSize];

  qspiStatus = qspi.read(readBuf, CALIBRATION_FLASH_ADDR, totalSize);
  if (qspiStatus != 0) {
    qspi.deinit();
    return false;
  }

  uint8_t rawData[sizeof(CalibrationData)];
  memcpy(rawData, readBuf, sizeof(CalibrationData));

  uint32_t storedCrc;
  memcpy(&storedCrc, readBuf + sizeof(CalibrationData), sizeof(storedCrc));

  const uint32_t calcCrc = calculateCRC32(rawData, sizeof(CalibrationData));
  if (calcCrc != storedCrc) {
    qspi.deinit();
    return false;
  }

  memcpy(&data, rawData, sizeof(CalibrationData));
  qspi.deinit();
  return true;
}

