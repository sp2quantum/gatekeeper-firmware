#pragma once

namespace SerialActivityLed {

// Configures the shield's data indicator LED and leaves it off.
void setup();

// Called from the USB ISR after a nonempty CDC transfer completes. The ISR
// only records the event; GPIO and timing work stays in the main loop.
void signalTransferFromIsr();

// Turns the LED on for a short, visible interval after the latest transfer.
// This is nonblocking; while idle it performs only a counter read/compare.
void service();

}  // namespace SerialActivityLed
