#pragma once

#include <Arduino.h>

class PeripheralCommsController {
 public:
  enum class SpiBus {
    Dac,
    Adc,
  };

  static constexpr size_t kRegisterTransferMaxBytes = 4;

  static volatile bool dacSpiTransferInProgress;
  static volatile bool adcSpiTransferInProgress;

 private:
  static bool spiInitialized;
  int cs_pin;

  static uint8_t __attribute__((aligned(32)))
      tx_buffer[kRegisterTransferMaxBytes];
  static uint8_t __attribute__((aligned(32)))
      rx_buffer[kRegisterTransferMaxBytes];

  bool performRegisterTransfer(SpiBus bus, uint8_t* tx, uint8_t* rx,
                               size_t count, bool dacReadMode = false);
  bool transferBytes(SpiBus bus, uint8_t* bytes, size_t count,
                     bool dacReadMode = false);
  uint8_t transferByte(SpiBus bus, uint8_t data);

 public:
  explicit PeripheralCommsController(int cs_pin);

  static void setup();
  bool transferDAC(void* buf, size_t count);
  bool transferDACRead(void* buf, size_t count);
  bool transferADC(void* buf, size_t count);
  uint8_t transferDAC(uint8_t data);
  uint8_t transferADC(uint8_t data);
  bool transferDACNoTransaction(void* buf, size_t count);
  bool transferADCNoTransaction(void* buf, size_t count);
  uint8_t transferDACNoTransaction(uint8_t data);
  uint8_t transferADCNoTransaction(uint8_t data);
  static void dataLedOn();
  static void dataLedOff();
};
