#pragma once

#include <Arduino.h>
#include "SPI.h"

class PeripheralCommsController {

  private:
    inline static bool spiInitialized = false;

    int cs_pin;

  public:
      PeripheralCommsController(int cs_pin) : cs_pin(cs_pin) {}

      #ifdef __NEW_SHIELD__
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
        transferDACNoTransaction(buf, count);
      }
      void transferADC(void* buf, size_t count) {
        transferADCNoTransaction(buf, count);
      }
      uint8_t transferDAC(uint8_t data) {
        return transferDACNoTransaction(data);
      }
      uint8_t transferADC(uint8_t data) {
        return transferADCNoTransaction(data);
      }


      void transferDACNoTransaction(void* buf, size_t count) {
        digitalWrite(cs_pin, LOW);
        SPI.transfer(buf, count);
        digitalWrite(cs_pin, HIGH);
      }
      void transferADCNoTransaction(void* buf, size_t count) {
        digitalWrite(cs_pin, LOW);
        SPI1.transfer(buf, count);
        digitalWrite(cs_pin, HIGH);
      }
      uint8_t transferDACNoTransaction(uint8_t data) {
        digitalWrite(cs_pin, LOW);
        uint8_t output = SPI.transfer(data);
        digitalWrite(cs_pin, HIGH);
        return output;
      }
      uint8_t transferADCNoTransaction(uint8_t data) {
        digitalWrite(cs_pin, LOW);
        uint8_t output = SPI1.transfer(data);
        digitalWrite(cs_pin, HIGH);
        return output;
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
        transferDACNoTransaction(buf, count);
        SPI.endTransaction();
      }
      void transferADC(void* buf, size_t count) {
        SPI.beginTransaction(ADC_SPI_SETTINGS);
        transferADCNoTransaction(buf, count);
        SPI.endTransaction();
      }
      uint8_t transferDAC(uint8_t data) {
        SPI.beginTransaction(DAC_SPI_SETTINGS);
        uint8_t result = transferDACNoTransaction(data);
        SPI.endTransaction();
        return result;
      }
      uint8_t transferADC(uint8_t data) {
        SPI.beginTransaction(ADC_SPI_SETTINGS);
        uint8_t result = transferADCNoTransaction(data);
        SPI.endTransaction();
        return result;
      }

      void transferDACNoTransaction(void* buf, size_t count) {
        digitalWrite(cs_pin, LOW);
        SPI.transfer(buf, count);
        digitalWrite(cs_pin, HIGH);
      }
      void transferADCNoTransaction(void* buf, size_t count) {
        digitalWrite(cs_pin, LOW);
        SPI.transfer(buf, count);
        digitalWrite(cs_pin, HIGH);
      }
      uint8_t transferDACNoTransaction(uint8_t data) {
        digitalWrite(cs_pin, LOW);
        uint8_t output = SPI.transfer(data);
        digitalWrite(cs_pin, HIGH);
        return output;
      }
      uint8_t transferADCNoTransaction(uint8_t data) {
        digitalWrite(cs_pin, LOW);
        uint8_t output = SPI.transfer(data);
digitalWrite(cs_pin, HIGH);                                 
        return output;
      }

      static void beginDacTransaction() {
        SPI.beginTransaction(DAC_SPI_SETTINGS);
      }
      static void beginAdcTransaction() {
        SPI.beginTransaction(ADC_SPI_SETTINGS);
      }

      #endif


      static void dataLedOn() { /*digitalWrite(led, HIGH);*/ }

      static void dataLedOff() { /*digitalWrite(led, LOW);*/ }

      static void endTransaction() { SPI.endTransaction(); }

  
};