#include "Peripherals/PeripheralCommsController.h"

#include <cstring>
#include <new>

#include "Config.h"
#include "Utils/FastGpio.h"
#include "drivers/SPIMaster.h"

bool PeripheralCommsController::spiInitialized = false;
volatile bool PeripheralCommsController::dacSpiTransferInProgress = false;
volatile bool PeripheralCommsController::adcSpiTransferInProgress = false;
uint8_t PeripheralCommsController::tx_buffer
    [PeripheralCommsController::kSpiBufferSize] __attribute__((aligned(32)));
uint8_t PeripheralCommsController::rx_buffer
    [PeripheralCommsController::kSpiBufferSize] __attribute__((aligned(32)));

namespace {
struct SpiBusState {
  mbed::SPI* spi;
  uint32_t frequency_hz;
  uint8_t mode;
  bool configured;
};

alignas(mbed::SPI) uint8_t dacSpiStorage[sizeof(mbed::SPI)];
alignas(mbed::SPI) uint8_t adcSpiStorage[sizeof(mbed::SPI)];

mbed::SPI* dacSpi = nullptr;
mbed::SPI* adcSpi = nullptr;
SpiBusState dacBus{nullptr, 0, 0, false};
SpiBusState adcBus{nullptr, 0, 0, false};

uint32_t transferFrequency(bool isDac) {
  return isDac ? DAC_SPI_FREQUENCY_HZ : ADC_SPI_FREQUENCY_HZ;
}

uint8_t transferMode(bool isDac) {
  return isDac ? DAC_SPI_MODE : ADC_SPI_MODE;
}

SpiBusState& busForTransfer(bool isDac) {
  return isDac ? dacBus : adcBus;
}

void constructSpiBuses() {
  if (dacSpi == nullptr) {
    dacSpi = new (dacSpiStorage) mbed::SPI(PD_7, PG_9, PB_3, NC);
    dacBus.spi = dacSpi;
  }

  if (adcSpi == nullptr) {
    adcSpi = new (adcSpiStorage) mbed::SPI(PJ_10, PJ_11, PH_6, NC);
    adcBus.spi = adcSpi;
  }
}

bool configureBusForTransfer(bool isDac) {
  SpiBusState& bus = busForTransfer(isDac);
  if (bus.spi == nullptr) {
    return false;
  }

  const uint32_t frequency = transferFrequency(isDac);
  const uint8_t mode = transferMode(isDac);
  if (!bus.configured || bus.frequency_hz != frequency || bus.mode != mode) {
    bus.spi->format(8, mode);
    bus.spi->frequency(frequency);
    bus.frequency_hz = frequency;
    bus.mode = mode;
    bus.configured = true;
  }
  return true;
}

void clearCallerBuffer(uint8_t* tx, uint8_t* rx, size_t count) {
  if (rx != nullptr) {
    memset(rx, 0, count);
  } else if (tx != nullptr) {
    memset(tx, 0, count);
  }
}
}  // namespace

PeripheralCommsController::PeripheralCommsController(int cs_pin)
    : cs_pin(cs_pin) {}

bool PeripheralCommsController::performMbedTransfer(bool is_dac, uint8_t* tx,
                                                    uint8_t* rx,
                                                    size_t count) {
  if (count == 0) {
    return true;
  }

  if (!spiInitialized || count > kSpiBufferSize ||
      !configureBusForTransfer(is_dac)) {
    clearCallerBuffer(tx, rx, count);
    return false;
  }

  if (tx != nullptr) {
    memcpy(tx_buffer, tx, count);
  } else {
    memset(tx_buffer, 0, count);
  }
  memset(rx_buffer, 0, count);

  SpiBusState& bus = busForTransfer(is_dac);
  volatile bool& inProgress =
      is_dac ? dacSpiTransferInProgress : adcSpiTransferInProgress;

  FastGpio::digitalWrite(cs_pin, false);
  inProgress = true;
  const int transferred = bus.spi->write(
      reinterpret_cast<const char*>(tx_buffer), static_cast<int>(count),
      reinterpret_cast<char*>(rx_buffer), static_cast<int>(count));
  inProgress = false;
  FastGpio::digitalWrite(cs_pin, true);

  if (transferred != static_cast<int>(count)) {
    clearCallerBuffer(tx, rx, count);
    return false;
  }

  if (rx != nullptr) {
    memcpy(rx, rx_buffer, count);
  } else if (tx != nullptr) {
    memcpy(tx, rx_buffer, count);
  }

  return true;
}

void PeripheralCommsController::setup() {
  if (spiInitialized) {
    return;
  }

  constructSpiBuses();
  bool initialized = configureBusForTransfer(true);
  initialized = configureBusForTransfer(false) && initialized;

  spiInitialized = initialized;
}

bool PeripheralCommsController::transferDAC(void* buf, size_t count) {
  return performMbedTransfer(true, static_cast<uint8_t*>(buf),
                             static_cast<uint8_t*>(buf), count);
}

bool PeripheralCommsController::transferADC(void* buf, size_t count) {
  return performMbedTransfer(false, static_cast<uint8_t*>(buf),
                             static_cast<uint8_t*>(buf), count);
}

uint8_t PeripheralCommsController::transferDAC(uint8_t data) {
  uint8_t tx_byte = data;
  return performMbedTransfer(true, &tx_byte, &tx_byte, 1) ? tx_byte : 0;
}

uint8_t PeripheralCommsController::transferADC(uint8_t data) {
  uint8_t tx_byte = data;
  return performMbedTransfer(false, &tx_byte, &tx_byte, 1) ? tx_byte : 0;
}

bool PeripheralCommsController::transferDACNoTransaction(void* buf,
                                                        size_t count) {
  return performMbedTransfer(true, static_cast<uint8_t*>(buf),
                             static_cast<uint8_t*>(buf), count);
}

bool PeripheralCommsController::transferADCNoTransaction(void* buf,
                                                        size_t count) {
  return performMbedTransfer(false, static_cast<uint8_t*>(buf),
                             static_cast<uint8_t*>(buf), count);
}

uint8_t PeripheralCommsController::transferDACNoTransaction(uint8_t data) {
  uint8_t tx_byte = data;
  return performMbedTransfer(true, &tx_byte, &tx_byte, 1) ? tx_byte : 0;
}

uint8_t PeripheralCommsController::transferADCNoTransaction(uint8_t data) {
  uint8_t tx_byte = data;
  return performMbedTransfer(false, &tx_byte, &tx_byte, 1) ? tx_byte : 0;
}

void PeripheralCommsController::dataLedOn() {}

void PeripheralCommsController::dataLedOff() {}
