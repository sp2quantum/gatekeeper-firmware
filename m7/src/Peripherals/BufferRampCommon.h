#pragma once

#include <Arduino.h>

#include "Config.h"
#include "Peripherals/OperationResult.h"

namespace BufferRampCommon {

bool isValidDacChannelCount(int count);
bool isValidAdcChannelCount(int count);
bool isUint32AtLeast(float value, uint32_t minimum);

uint8_t adcBoardForChannel(int channel);

OperationResult validateDacChannels(const int* channels, int count,
                                    bool rejectDuplicates = true);
OperationResult validateAdcChannels(const int* channels, int count,
                                    bool rejectDuplicates = true);
OperationResult validateRampChannels(const int* dacChannels,
                                     int numDacChannels,
                                     const int* adcChannels,
                                     int numAdcChannels,
                                     bool rejectDuplicateDacChannels = true,
                                     bool rejectDuplicateAdcChannels = true);
OperationResult finishRampTimingWatchdog(
    bool includeAdcConversionMissteps = true);
OperationResult dacWriteFailure(int channel, double voltage);

bool sendVoltageFrame(const double* packets, size_t length);
bool encodeDacVoltagePackets(int numDacChannels, const int* dacChannels,
                             const double* voltages,
                             byte packets[NUM_DAC_CHANNELS][3]);
bool writeDacPackets(int numDacChannels, const int* dacChannels,
                     byte packets[NUM_DAC_CHANNELS][3]);

}  // namespace BufferRampCommon
