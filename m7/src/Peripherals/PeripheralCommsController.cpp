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
    [PeripheralCommsController::kRegisterTransferMaxBytes]
    __attribute__((aligned(32)));
uint8_t PeripheralCommsController::rx_buffer
    [PeripheralCommsController::kRegisterTransferMaxBytes]
    __attribute__((aligned(32)));

namespace {
constexpr uint32_t kSpiPollTimeout = 100000;
constexpr uint32_t kSpiRxReadyMask =
    SPI_SR_RXP | SPI_SR_RXWNE | SPI_SR_RXPLVL;
constexpr uint32_t kSpiErrorMask = SPI_SR_OVR | SPI_SR_MODF | SPI_SR_TIFRE;
constexpr uint32_t kSpiFlagClearMask =
    SPI_IFCR_EOTC | SPI_IFCR_TXTFC | SPI_IFCR_OVRC | SPI_IFCR_MODFC |
    SPI_IFCR_TIFREC | SPI_IFCR_TSERFC | SPI_IFCR_SUSPC;

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

using SpiBus = PeripheralCommsController::SpiBus;

bool isDacBus(SpiBus bus) {
  return bus == SpiBus::Dac;
}

uint32_t transferFrequency(SpiBus bus) {
  return isDacBus(bus) ? DAC_SPI_FREQUENCY_HZ : ADC_SPI_FREQUENCY_HZ;
}

uint8_t transferMode(SpiBus bus, bool dacReadMode) {
  if (!isDacBus(bus)) {
    return ADC_SPI_MODE;
  }
  return dacReadMode ? DAC_READ_SPI_MODE : DAC_SPI_MODE;
}

SpiBusState& busForTransfer(SpiBus bus) {
  return isDacBus(bus) ? dacBus : adcBus;
}

SPI_TypeDef* spiPeripheralForTransfer(SpiBus bus) {
  return isDacBus(bus) ? SPI1 : SPI5;
}

volatile bool& transferInProgressForTransfer(SpiBus bus) {
  return isDacBus(bus) ? PeripheralCommsController::dacSpiTransferInProgress
                       : PeripheralCommsController::adcSpiTransferInProgress;
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

bool configureBusForTransfer(SpiBus selectedBus, bool dacReadMode = false) {
  SpiBusState& bus = busForTransfer(selectedBus);
  if (bus.spi == nullptr) {
    return false;
  }

  const uint32_t frequency = transferFrequency(selectedBus);
  const uint8_t mode = transferMode(selectedBus, dacReadMode);
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

void loadTransferBuffers(uint8_t* txScratch, uint8_t* rxScratch,
                         const uint8_t* tx, size_t count) {
  if (tx != nullptr) {
    memcpy(txScratch, tx, count);
  } else {
    memset(txScratch, 0, count);
  }
  memset(rxScratch, 0, count);
}

void copyTransferResult(uint8_t* tx, uint8_t* rx, const uint8_t* rxScratch,
                        size_t count) {
  if (rx != nullptr) {
    memcpy(rx, rxScratch, count);
  } else if (tx != nullptr) {
    memcpy(tx, rxScratch, count);
  }
}

void invalidateBusConfig(SpiBus bus) {
  busForTransfer(bus).configured = false;
}

void clearSpiFlags(SPI_TypeDef* spi) {
  spi->IFCR = kSpiFlagClearMask;
}

uint8_t readSpiByte(SPI_TypeDef* spi) {
  return *reinterpret_cast<volatile uint8_t*>(&spi->RXDR);
}

void writeSpiByte(SPI_TypeDef* spi, uint8_t value) {
  *reinterpret_cast<volatile uint8_t*>(&spi->TXDR) = value;
}

void drainRxFifo(SPI_TypeDef* spi) {
  for (size_t i = 0;
       i < PeripheralCommsController::kRegisterTransferMaxBytes * 2 &&
       (spi->SR & kSpiRxReadyMask) != 0;
       ++i) {
    static_cast<void>(readSpiByte(spi));
  }
}

bool transferThroughRegisters(SPI_TypeDef* spi, int cs_pin, const uint8_t* tx,
                              uint8_t* rx, size_t count) {
  if (count == 0 ||
      count > PeripheralCommsController::kRegisterTransferMaxBytes) {
    return false;
  }

  auto fail = [&]() {
    FastGpio::digitalWrite(cs_pin, true);
    clearSpiFlags(spi);
    spi->CR1 &= ~SPI_CR1_SPE;
    return false;
  };

  clearSpiFlags(spi);
  if ((spi->CR1 & SPI_CR1_SPE) != 0) {
    uint32_t timeout = kSpiPollTimeout;
    while ((spi->SR & SPI_SR_TXC) == 0) {
      if (--timeout == 0) {
        return fail();
      }
    }
    spi->CR1 &= ~SPI_CR1_SPE;
  }
  drainRxFifo(spi);
  spi->CFG1 &= ~SPI_CFG1_FTHLV;
  spi->CR2 = (spi->CR2 & ~SPI_CR2_TSIZE) | count;
  spi->CR1 |= SPI_CR1_SPE;

  for (size_t written = 0; written < count; ++written) {
    uint32_t timeout = kSpiPollTimeout;
    while ((spi->SR & SPI_SR_TXP) == 0) {
      if ((spi->SR & kSpiErrorMask) != 0 || --timeout == 0) {
        return fail();
      }
    }
    writeSpiByte(spi, tx != nullptr ? tx[written] : 0);
  }

  FastGpio::digitalWrite(cs_pin, false);
  spi->CR1 |= SPI_CR1_CSTART;

  uint32_t timeout = kSpiPollTimeout;
  while ((spi->SR & SPI_SR_EOT) == 0) {
    if ((spi->SR & kSpiErrorMask) != 0 || --timeout == 0) {
      return fail();
    }
  }
  FastGpio::digitalWrite(cs_pin, true);

  for (size_t read = 0; read < count; ++read) {
    timeout = kSpiPollTimeout;
    while ((spi->SR & kSpiRxReadyMask) == 0) {
      if ((spi->SR & kSpiErrorMask) != 0 || --timeout == 0) {
        return fail();
      }
    }
    rx[read] = readSpiByte(spi);
  }

  clearSpiFlags(spi);
  spi->CR1 &= ~SPI_CR1_SPE;
  return true;
}
}  // namespace

PeripheralCommsController::PeripheralCommsController(int cs_pin)
    : cs_pin(cs_pin) {}

bool PeripheralCommsController::performRegisterTransfer(SpiBus bus,
                                                        uint8_t* tx,
                                                        uint8_t* rx,
                                                        size_t count,
                                                        bool dacReadMode) {
  if (count == 0) {
    return true;
  }

  if (!spiInitialized ||
      count > PeripheralCommsController::kRegisterTransferMaxBytes ||
      !configureBusForTransfer(bus, dacReadMode)) {
    return false;
  }

  loadTransferBuffers(tx_buffer, rx_buffer, tx, count);

  volatile bool& inProgress = transferInProgressForTransfer(bus);
  inProgress = true;
  const bool transferred = transferThroughRegisters(
      spiPeripheralForTransfer(bus), cs_pin, tx_buffer, rx_buffer, count);
  inProgress = false;

  if (!transferred) {
    invalidateBusConfig(bus);
    return false;
  }

  copyTransferResult(tx, rx, rx_buffer, count);
  return true;
}

bool PeripheralCommsController::transferBytes(SpiBus bus, uint8_t* bytes,
                                              size_t count, bool dacReadMode) {
  if (performRegisterTransfer(bus, bytes, bytes, count, dacReadMode)) {
    return true;
  }
  clearCallerBuffer(bytes, bytes, count);
  return false;
}

uint8_t PeripheralCommsController::transferByte(SpiBus bus, uint8_t data) {
  return transferBytes(bus, &data, 1) ? data : 0;
}

void PeripheralCommsController::setup() {
  if (spiInitialized) {
    return;
  }

  constructSpiBuses();
  const bool dacReady = configureBusForTransfer(SpiBus::Dac);
  const bool adcReady = configureBusForTransfer(SpiBus::Adc);
  spiInitialized = dacReady && adcReady;
}

bool PeripheralCommsController::transferDAC(void* buf, size_t count) {
  return transferBytes(SpiBus::Dac, static_cast<uint8_t*>(buf), count);
}

bool PeripheralCommsController::transferDACRead(void* buf, size_t count) {
  return transferBytes(SpiBus::Dac, static_cast<uint8_t*>(buf), count, true);
}

bool PeripheralCommsController::transferADC(void* buf, size_t count) {
  return transferBytes(SpiBus::Adc, static_cast<uint8_t*>(buf), count);
}

uint8_t PeripheralCommsController::transferDAC(uint8_t data) {
  return transferByte(SpiBus::Dac, data);
}

uint8_t PeripheralCommsController::transferADC(uint8_t data) {
  return transferByte(SpiBus::Adc, data);
}

bool PeripheralCommsController::transferDACNoTransaction(void* buf,
                                                        size_t count) {
  return transferBytes(SpiBus::Dac, static_cast<uint8_t*>(buf), count);
}

bool PeripheralCommsController::transferADCNoTransaction(void* buf,
                                                        size_t count) {
  return transferBytes(SpiBus::Adc, static_cast<uint8_t*>(buf), count);
}

uint8_t PeripheralCommsController::transferDACNoTransaction(uint8_t data) {
  return transferByte(SpiBus::Dac, data);
}

uint8_t PeripheralCommsController::transferADCNoTransaction(uint8_t data) {
  return transferByte(SpiBus::Adc, data);
}

void PeripheralCommsController::dataLedOn() {}

void PeripheralCommsController::dataLedOff() {}
