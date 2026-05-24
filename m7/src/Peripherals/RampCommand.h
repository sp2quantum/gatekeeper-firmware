#pragma once

#include <Arduino.h>

#include "Peripherals/OperationResult.h"

namespace RampCommand {

enum class DacBoundsMode {
  Calibrated,
  CalibratedAndGlobal,
};

bool isBooleanArg(float value);

OperationResult validateDacEndpoints(int numDacChannels,
                                     const int* dacChannels,
                                     const float* dacV0s,
                                     const float* dacVfs);
OperationResult validateDacVoltageListBounds(int numDacChannels,
                                             int numSteps,
                                             const int* dacChannels,
                                             const float* channelMajorVoltages);
OperationResult validateDacVoltageListBounds(int numDacChannels,
                                             int numSteps,
                                             const int* dacChannels,
                                             float* const* dacVoltageLists);
OperationResult validateDac2DScanBounds(int numDacChannels,
                                        const int* dacChannels,
                                        const float* startPoint,
                                        const float* fastAxisVector,
                                        const float* slowAxisVector,
                                        DacBoundsMode mode);
OperationResult validateBoxcarDacEndpoints(int numDacChannels,
                                           const int* dacChannels,
                                           const float* dacV0_1,
                                           const float* dacVf_1,
                                           const float* dacV0_2,
                                           const float* dacVf_2);

}  // namespace RampCommand
