#pragma once

#include <Arduino.h>

#include "Calibration/CalibrationData.h"
#include "Config.h"
#include "FunctionRegistry/FunctionRegistryArgumentParser.h"
#include "Utils/OperationResult.h"

using FunctionRegistryParsing::List;

namespace DACController {

void setup();
void initialize();
bool isChannelIndexValid(int channelIndex);

OperationResult setVoltage(int channel_index, float voltage);
bool setVoltageNoLdac(int channel_index, float voltage);
bool encodeVoltagePacket(int channel_index, float voltage, byte packet[3]);
bool writeVoltagePacketNoLdac(int channel_index, const byte packet[3]);
int getCsPin(int channel_index);

void toggleLdac();

OperationResult getVoltage(int channel_index);
void applyCalibration(int channel_index, float offset, float gain);
void setCalibration(int channel_index, float offset, float gain);
float getLowerBound(int channel);
float getUpperBound(int channel);

}  // namespace DACController
