#include "SerialActivityLed.h"

#include <Arduino.h>

#include <cstdint>

namespace SerialActivityLed {
namespace {

// D6 (PD13) is the data indicator LED on the Gatekeeper shield.
constexpr pin_size_t kDataLedPin = 6;
constexpr uint32_t kMinimumVisibleTimeMs = 30;

// Both USB callbacks run in the same USB interrupt context. The main loop only
// reads this generation, so an event cannot be lost to a read/clear race.
volatile uint32_t transferGeneration = 0;

uint32_t observedGeneration = 0;
uint32_t lastTransferMs = 0;
bool ledOn = false;

}  // namespace

void setup() {
  pinMode(kDataLedPin, OUTPUT);
  digitalWrite(kDataLedPin, LOW);
}

void signalTransferFromIsr() { ++transferGeneration; }

void service() {
  const uint32_t generation = transferGeneration;
  if (generation != observedGeneration) {
    observedGeneration = generation;
    lastTransferMs = HAL_GetTick();
    if (!ledOn) {
      digitalWrite(kDataLedPin, HIGH);
      ledOn = true;
    }
    return;
  }

  if (ledOn &&
      static_cast<uint32_t>(HAL_GetTick() - lastTransferMs) >=
          kMinimumVisibleTimeMs) {
    digitalWrite(kDataLedPin, LOW);
    ledOn = false;
  }
}

}  // namespace SerialActivityLed
