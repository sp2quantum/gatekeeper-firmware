#include "Peripherals/RampCommand.h"

#include "Peripherals/BufferRampCommon.h"
#include "Peripherals/DAC/DACController.h"

namespace RampCommand {

bool isBooleanArg(float value) { return value == 0.0f || value == 1.0f; }

OperationResult validateDacEndpoints(int numDacChannels,
                                     const int* dacChannels,
                                     const float* dacV0s,
                                     const float* dacVfs) {
  for (int i = 0; i < numDacChannels; i++) {
    const int ch = dacChannels[i];
    const float lowerBound = DACController::getLowerBound(ch);
    const float upperBound = DACController::getUpperBound(ch);

    if (dacV0s[i] < lowerBound || dacV0s[i] > upperBound) {
      return OperationResult::Failure("DAC " + String(ch) +
                                      " start voltage " +
                                      String(dacV0s[i], 6) +
                                      "V out of bounds [" +
                                      String(lowerBound, 6) + ", " +
                                      String(upperBound, 6) + "]");
    }
    if (dacVfs[i] < lowerBound || dacVfs[i] > upperBound) {
      return OperationResult::Failure("DAC " + String(ch) +
                                      " end voltage " +
                                      String(dacVfs[i], 6) +
                                      "V out of bounds [" +
                                      String(lowerBound, 6) + ", " +
                                      String(upperBound, 6) + "]");
    }
  }

  return OperationResult::Success();
}

OperationResult validateDacVoltageListBounds(
    int numDacChannels, int numSteps, const int* dacChannels,
    const float* channelMajorVoltages) {
  for (int i = 0; i < numDacChannels; i++) {
    const int ch = dacChannels[i];
    const float lowerBound = DACController::getLowerBound(ch);
    const float upperBound = DACController::getUpperBound(ch);
    const float* vlist =
        &channelMajorVoltages[static_cast<size_t>(i) *
                              static_cast<size_t>(numSteps)];
    for (int j = 0; j < numSteps; j++) {
      const float v = vlist[j];
      if (v < lowerBound || v > upperBound) {
        return OperationResult::Failure(
            "DAC " + String(ch) + " voltage[" + String(j) +
            "] = " + String(v, 6) + "V out of bounds [" +
            String(lowerBound, 6) + ", " + String(upperBound, 6) + "]");
      }
    }
  }
  return OperationResult::Success();
}

OperationResult validateDacVoltageListBounds(int numDacChannels,
                                             int numSteps,
                                             const int* dacChannels,
                                             float* const* dacVoltageLists) {
  for (int i = 0; i < numDacChannels; i++) {
    const int ch = dacChannels[i];
    const float lowerBound = DACController::getLowerBound(ch);
    const float upperBound = DACController::getUpperBound(ch);
    for (int j = 0; j < numSteps; j++) {
      const float voltage = dacVoltageLists[i][j];
      if (voltage < lowerBound || voltage > upperBound) {
        return OperationResult::Failure(
            "DAC " + String(ch) + " voltage[" + String(j) +
            "] = " + String(voltage, 6) + "V out of bounds [" +
            String(lowerBound, 6) + ", " + String(upperBound, 6) + "]");
      }
    }
  }
  return OperationResult::Success();
}

OperationResult validateDac2DScanBounds(int numDacChannels,
                                        const int* dacChannels,
                                        const float* startPoint,
                                        const float* fastAxisVector,
                                        const float* slowAxisVector,
                                        DacBoundsMode mode) {
  for (int i = 0; i < numDacChannels; i++) {
    const int ch = dacChannels[i];
    float lowerBound = DACController::getLowerBound(ch);
    float upperBound = DACController::getUpperBound(ch);
    if (mode == DacBoundsMode::CalibratedAndGlobal) {
      lowerBound = max(lowerBound, DACLimits::lower_voltage_limit[ch]);
      upperBound = min(upperBound, DACLimits::upper_voltage_limit[ch]);
    }

    const float corner1 = startPoint[i];
    const float corner2 = startPoint[i] + fastAxisVector[i];
    const float corner3 = startPoint[i] + slowAxisVector[i];
    const float corner4 =
        startPoint[i] + fastAxisVector[i] + slowAxisVector[i];
    const float minVoltage =
        min(min(corner1, corner2), min(corner3, corner4));
    const float maxVoltage =
        max(max(corner1, corner2), max(corner3, corner4));

    if (minVoltage < lowerBound || maxVoltage > upperBound) {
      return OperationResult::Failure("DAC " + String(ch) +
                                      " 2D scan range [" +
                                      String(minVoltage, 6) + ", " +
                                      String(maxVoltage, 6) +
                                      "]V exceeds bounds [" +
                                      String(lowerBound, 6) + ", " +
                                      String(upperBound, 6) + "]");
    }
  }

  return OperationResult::Success();
}

OperationResult validateBoxcarDacEndpoints(int numDacChannels,
                                           const int* dacChannels,
                                           const float* dacV0_1,
                                           const float* dacVf_1,
                                           const float* dacV0_2,
                                           const float* dacVf_2) {
  for (int i = 0; i < numDacChannels; i++) {
    const int ch = dacChannels[i];
    const float lowerBound = DACController::getLowerBound(ch);
    const float upperBound = DACController::getUpperBound(ch);

    if (dacV0_1[i] < lowerBound || dacV0_1[i] > upperBound) {
      return OperationResult::Failure(
          "DAC " + String(ch) + " start voltage 1 " +
          String(dacV0_1[i], 6) + "V out of bounds [" +
          String(lowerBound, 6) + ", " + String(upperBound, 6) + "]");
    }
    if (dacVf_1[i] < lowerBound || dacVf_1[i] > upperBound) {
      return OperationResult::Failure(
          "DAC " + String(ch) + " end voltage 1 " +
          String(dacVf_1[i], 6) + "V out of bounds [" +
          String(lowerBound, 6) + ", " + String(upperBound, 6) + "]");
    }
    if (dacV0_2[i] < lowerBound || dacV0_2[i] > upperBound) {
      return OperationResult::Failure(
          "DAC " + String(ch) + " start voltage 2 " +
          String(dacV0_2[i], 6) + "V out of bounds [" +
          String(lowerBound, 6) + ", " + String(upperBound, 6) + "]");
    }
    if (dacVf_2[i] < lowerBound || dacVf_2[i] > upperBound) {
      return OperationResult::Failure(
          "DAC " + String(ch) + " end voltage 2 " +
          String(dacVf_2[i], 6) + "V out of bounds [" +
          String(lowerBound, 6) + ", " + String(upperBound, 6) + "]");
    }
  }
  return OperationResult::Success();
}

}  // namespace RampCommand
