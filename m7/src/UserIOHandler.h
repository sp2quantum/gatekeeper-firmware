#pragma once

#include <Arduino.h>

#include "Utils/OperationResult.h"

namespace UserIOHandler {

OperationResult getFirmwareVersion();
OperationResult nop();
OperationResult getEnvironment();
OperationResult id();
OperationResult rdy();

__attribute__((section(".serial_number"))) extern const char serial_number[29];

OperationResult serialNumber();
void handleUserIO();

}  // namespace UserIOHandler
