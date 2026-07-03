#pragma once

#include <Arduino.h>

#include "Config.h"
#include "Utils/OperationResult.h"

namespace BufferRampCommon {

enum class TimeSeriesTimingMode {
  OneD,
  TwoDNormal,
  TwoDRetrace,
  TwoDSnake,
};

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
// Diagnoses a failed multi-channel DAC write: reports the first channel whose
// requested voltage is invalid, falling back to an SPI failure on channel 0.
OperationResult dacSetWriteFailure(int numDacChannels, const int* dacChannels,
                                   const double* voltages);

float maxAdcConversionTimePerBoard(const int* adcChannels,
                                   int numAdcChannels);

uint32_t minimumDacLedIntervalUs(uint32_t dacSettlingTimeUs,
                                 const int* adcChannels, int numAdcChannels,
                                 int numAdcAverages);
uint32_t minimumAwgWithAdcIntervalUs(int numDacChannels,
                                     const int* adcChannels,
                                     int numAdcChannels);
uint32_t minimumDacOnlyIntervalUs(int numDacChannels);
uint32_t minimumTimeSeriesAdcIntervalUs(const int* adcChannels,
                                        int numAdcChannels,
                                        TimeSeriesTimingMode mode);
uint32_t minimumBoxcarConversionTimeUs(const int* adcChannels,
                                       int numAdcChannels);
OperationResult validateDacLedTiming(float dacIntervalArg,
                                     float dacSettlingTimeArg,
                                     const int* adcChannels,
                                     int numAdcChannels,
                                     int numAdcAverages);
OperationResult validateTimeSeriesTiming(float adcIntervalArg,
                                         const int* adcChannels,
                                         int numAdcChannels,
                                         TimeSeriesTimingMode mode);
OperationResult validateAwgWithAdcTiming(float dacIntervalArg,
                                         int numDacChannels,
                                         const int* adcChannels,
                                         int numAdcChannels);
OperationResult validateDacOnlyTiming(float dacIntervalArg,
                                      int numDacChannels);
OperationResult validateBoxcarTiming(float adcConversionTimeArg,
                                     const int* adcChannels,
                                     int numAdcChannels);

bool sendVoltageFrame(const double* packets, size_t length);
bool encodeDacVoltagePackets(int numDacChannels, const int* dacChannels,
                             const double* voltages,
                             byte packets[NUM_DAC_CHANNELS][3]);
bool writeDacPackets(int numDacChannels, const int* dacChannels,
                     byte packets[NUM_DAC_CHANNELS][3]);

}  // namespace BufferRampCommon
