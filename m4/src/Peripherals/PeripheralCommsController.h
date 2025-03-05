#pragma once

#include <Arduino.h>
#include "SPI.h"

struct PeripheralCommsController {

  inline static bool spiInitialized = false;

  PeripheralCommsController() {}

  #if !defined(__OLD_SHIELD__)
  static void setup() {
    if (!spiInitialized) {
      SPI.begin();
      SPI.beginTransaction(DAC_SPI_SETTINGS);
      spiInitialized = true;
      SPI1.begin();
      SPI1.beginTransaction(ADC_SPI_SETTINGS);
    }
  }
  static void transferDAC(void* buf, size_t count) {
    transferDACNoTransaction(buf, count);
  }
  static void transferADC(void* buf, size_t count) {
    transferADCNoTransaction(buf, count);
  }
  static uint8_t transferDAC(uint8_t data) {
    return transferDACNoTransaction(data);
  }
  static uint8_t transferADC(uint8_t data) {
    return transferADCNoTransaction(data);
  }

  #else
  static void setup() {
    if (!spiInitialized) {
      SPI.begin();
      spiInitialized = true;
    }
  }
  static void transferDAC(void* buf, size_t count) {
    SPI.beginTransaction(DAC_SPI_SETTINGS);
    transferDACNoTransaction(buf, count);
    SPI.endTransaction();
  }
  static void transferADC(void* buf, size_t count) {
    SPI.beginTransaction(ADC_SPI_SETTINGS);
    transferADCNoTransaction(buf, count);
    SPI.endTransaction();
  }
  static uint8_t transferDAC(uint8_t data) {
    SPI.beginTransaction(DAC_SPI_SETTINGS);
    uint8_t result = transferDACNoTransaction(data);
    SPI.endTransaction();
    return result;
  }
  static uint8_t transferADC(uint8_t data) {
    SPI.beginTransaction(ADC_SPI_SETTINGS);
    uint8_t result = transferADCNoTransaction(data);
    SPI.endTransaction();
    return result;
  }

  #endif

  static void transferDACNoTransaction(void* buf, size_t count) {
    SPI.transfer(buf, count);
  }
  static void transferADCNoTransaction(void* buf, size_t count) {
    SPI.transfer(buf, count);
  }
  static uint8_t transferDACNoTransaction(uint8_t data) {
    return SPI.transfer(data);
  }
  static uint8_t transferADCNoTransaction(uint8_t data) {
    return SPI.transfer(data);
  }

  static void dataLedOn() { /*digitalWrite(led, HIGH);*/ }

  static void dataLedOff() { /*digitalWrite(led, LOW);*/ }


  static void beginDacTransaction() { SPI.beginTransaction(DAC_SPI_SETTINGS); }

  static void beginAdcTransaction() { SPI.beginTransaction(DAC_SPI_SETTINGS); }

  static void endTransaction() { SPI.endTransaction(); }

  
};