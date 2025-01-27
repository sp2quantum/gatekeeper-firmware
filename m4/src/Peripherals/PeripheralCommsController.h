#pragma once

#include <Arduino.h>
#include "SPI.h"

class PeripheralCommsController {

 public:
  static bool spiInitialized;

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
  void transferDAC(void* buf, size_t count) {
    SPI.transfer(buf, count);
  }
  void transferADC(void* buf, size_t count) {
    SPI1.transfer(buf, count);
  }
  uint8_t transferDAC(uint8_t data) {
    return SPI.transfer(data);
  }
  uint8_t transferADC(uint8_t data) {
    return SPI1.transfer(data);
  }
  #else
  static void setup() {
    if (!spiInitialized) {
      SPI.begin();
      spiInitialized = true;
    }
  }
  void transferDAC(void* buf, size_t count) {
    SPI.beginTransaction(DAC_SPI_SETTINGS);
    SPI.transfer(buf, count);
    SPI.endTransaction();
  }
  void transferADC(void* buf, size_t count) {
    SPI.beginTransaction(ADC_SPI_SETTINGS);
    SPI.transfer(buf, count);
    SPI.endTransaction();
  }
  uint8_t transferDAC(uint8_t data) {
    SPI.beginTransaction(DAC_SPI_SETTINGS);
    uint8_t result = SPI.transfer(data);
    SPI.endTransaction();
    return result;
  }
  uint8_t transferADC(uint8_t data) {
    SPI.beginTransaction(ADC_SPI_SETTINGS);
    uint8_t result = SPI.transfer(data);
    SPI.endTransaction();
    return result;
  }
  #endif

  static void dataLedOn() { /*digitalWrite(led, HIGH);*/ }

  static void dataLedOff() { /*digitalWrite(led, LOW);*/ }


  // void beginTransaction() { SPI.beginTransaction(DAC_SPI_SETTINGS); }

  // void endTransaction() { SPI.endTransaction(); }

  
};

bool PeripheralCommsController::spiInitialized = false;
