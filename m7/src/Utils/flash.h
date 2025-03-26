#pragma once

#include <Arduino.h>
#include "QSPIFBlockDevice.h"
#include <cstdint>
#include <cstring>
#include "CalibrationData.h"

// Fixed flash memory address for storing calibration (15 MB offset in 16 MB QSPI flash).
// This address is chosen to avoid the region used by WiFi firmware (first 1MB) and OTA (next 13MB).
static constexpr uint32_t CALIBRATION_FLASH_ADDR = 15 * 1024 * 1024;  // 15 MB offset

// Helper function: Calculate CRC32 (polynomial 0xEDB88320) over a data buffer
inline uint32_t calculateCRC32(const uint8_t *data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;  // start with all bits high
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;  // LSB is 1: shift and XOR with polynomial
            } else {
                crc >>= 1;                     // LSB is 0: just shift
            }
        }
    }
    crc ^= 0xFFFFFFFF;  // invert bits
    return crc;
}

// Write calibration data to QSPI flash. Returns true on success, false on failure.
inline bool writeCalibrationToFlash(const CalibrationData &data) {
    // Instantiate the QSPI flash block device (use Arduino GIGA's QSPI pins and settings)
    QSPIFBlockDevice qspi(QSPI_SO0, QSPI_SO1, QSPI_SO2, QSPI_SO3, QSPI_SCK, QSPI_CS,
                          QSPIF_POLARITY_MODE_1, 40000000);  // Mode 1, 40 MHz
    
    // Initialize the QSPI flash device
    int err = qspi.init();
    if (err != 0) {
        Serial.println("FAILURE: QSPI flash init failed");
        return false;
    }
    
    // Prepare a buffer for the 128-byte calibration data
    uint8_t dataBuf[sizeof(CalibrationData)];
    memcpy(dataBuf, &data, sizeof(CalibrationData));  // Copy struct bytes into buffer
    
    // Compute CRC32 of the 128-byte calibration data
    uint32_t crc = calculateCRC32(dataBuf, sizeof(CalibrationData));
    
    // Combine data and CRC into one buffer for writing
    const size_t totalSize = sizeof(CalibrationData) + sizeof(crc);  // 128 + 4 = 132 bytes
    uint8_t writeBuf[totalSize];
    memcpy(writeBuf, dataBuf, sizeof(CalibrationData));
    memcpy(writeBuf + sizeof(CalibrationData), &crc, sizeof(crc));
    
    // Erase the flash sector that will hold the data (flash must be erased before writing)
    size_t eraseSize = qspi.get_erase_size(CALIBRATION_FLASH_ADDR);
    uint32_t alignedAddr = CALIBRATION_FLASH_ADDR - (CALIBRATION_FLASH_ADDR % eraseSize);
    err = qspi.erase(alignedAddr, eraseSize);
    if (err != 0) {
        Serial.println("FAILURE: QSPI flash erase failed");
        qspi.deinit();
        return false;
    }
    
    // Program (write) the data + CRC buffer into flash at the fixed address
    err = qspi.program(writeBuf, CALIBRATION_FLASH_ADDR, totalSize);
    if (err != 0) {
        Serial.println("FAILURE: QSPI flash program failed");
        qspi.deinit();
        return false;
    }
    
    // Deinitialize the QSPI device to ensure data is physically written and free resources
    qspi.deinit();
    return true;
}

// Read calibration data from QSPI flash. Returns true if valid data was read (CRC ok).
inline bool readCalibrationFromFlash(CalibrationData &data) {
    QSPIFBlockDevice qspi(QSPI_SO0, QSPI_SO1, QSPI_SO2, QSPI_SO3, QSPI_SCK, QSPI_CS,
                          QSPIF_POLARITY_MODE_1, 40000000);
    int err = qspi.init();
    if (err != 0) {
        Serial.println("FAILURE: QSPI flash init failed");
        return false;
    }
    
    const size_t totalSize = sizeof(CalibrationData) + sizeof(uint32_t);  // 132 bytes expected
    uint8_t readBuf[totalSize];
    
    // Read the data and CRC from flash
    err = qspi.read(readBuf, CALIBRATION_FLASH_ADDR, totalSize);
    if (err != 0) {
        Serial.println("FAILURE: QSPI flash read failed");
        qspi.deinit();
        return false;
    }
    
    // Extract the 128-byte data and the 4-byte CRC from the buffer
    uint8_t rawData[sizeof(CalibrationData)];
    memcpy(rawData, readBuf, sizeof(CalibrationData));
    uint32_t storedCrc;
    memcpy(&storedCrc, readBuf + sizeof(CalibrationData), sizeof(storedCrc));
    
    // Compute CRC over the 128 bytes of data read from flash
    uint32_t calcCrc = calculateCRC32(rawData, sizeof(CalibrationData));
    
    // Verify CRC matches
    if (calcCrc != storedCrc) {
        Serial.println("FAILURE: Calibration data CRC mismatch (data is corrupted or not written)");
        qspi.deinit();
        return false;
    }
    
    // CRC is valid – copy the raw data into the CalibrationData struct
    memcpy(&data, rawData, sizeof(CalibrationData));
    qspi.deinit();
    return true;
}
