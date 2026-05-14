#include "Peripherals/God2D.h"

#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Peripherals/God.h"

namespace {
bool isValidDacChannelCount(int count) {
  return count >= 1 && count <= NUM_DAC_CHANNELS;
}

bool isValidAdcChannelCount(int count) {
  return count >= 1 && count <= NUM_ADC_CHANNELS;
}

bool isUint32AtLeast(float value, uint32_t minimum) {
  return value >= static_cast<float>(minimum) &&
         static_cast<double>(value) <= 4294967295.0;
}

bool isBooleanArg(float value) {
  return value == 0.0f || value == 1.0f;
}

OperationResult validateDacChannels(const int* channels, int count) {
  for (int i = 0; i < count; i++) {
    if (!DACController::isChannelIndexValid(channels[i])) {
      return OperationResult::Failure("Invalid DAC channel index " +
                                      String(channels[i]));
    }
  }
  return OperationResult::Success();
}

OperationResult validateAdcChannels(const int* channels, int count) {
  for (int i = 0; i < count; i++) {
    if (!ADCController::isChannelIndexValid(channels[i])) {
      return OperationResult::Failure("Invalid ADC channel index " +
                                      String(channels[i]));
    }
  }
  return OperationResult::Success();
}

OperationResult finishRampTimingWatchdog(
    bool includeAdcConversionMissteps = true) {
  const uint32_t dacSpiMissteps = TimingUtil::dacSpiMisstepEvents;
  const uint32_t adcSpiMissteps = TimingUtil::adcSpiMisstepEvents;
  const uint32_t adcConversionMissteps =
      TimingUtil::adcConversionMisstepEvents;
  if (dacSpiMissteps == 0 && adcSpiMissteps == 0 &&
      (!includeAdcConversionMissteps || adcConversionMissteps == 0)) {
    return OperationResult::Success();
  }
  String message =
      "Ramp timing misstep during ramp dac_spi_missteps=" +
      String(dacSpiMissteps) + " adc_spi_missteps=" +
      String(adcSpiMissteps);
  if (includeAdcConversionMissteps) {
    message += " adc_conversion_missteps=" + String(adcConversionMissteps);
  }
  return OperationResult::Failure(message);
}

}

  void God2D::setup() { initializeRegistry(); }



  void God2D::initializeRegistry() {
    registerMemberFunctionVector(timeSeriesBufferRamp2D,
                                 "2D_TIME_SERIES_BUFFER_RAMP");
    registerMemberFunctionVector(dacLedBufferRamp2D, "2D_DAC_LED_BUFFER_RAMP");
  }



  // timeSeriesBufferRamp2D:
  // Arguments (in order):
  // numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
  // dacInterval_us, adcInterval_us, retrace (0.0f = false, 1.0f = true), snake (0.0f = false, 1.0f = true),
  // [dacChannelID] * numDacChannels,
  // [startPoint] * numDacChannels,
  // [fastAxisVector] * numDacChannels,
  // [slowAxisVector] * numDacChannels,
  // [adcChannelID] * numAdcChannels
  //
  // The fast/slow axis vectors define a 2D plane in the N-dimensional DAC phase space.
  // Position(s,f) = startPoint + s*slowAxisVector + f*fastAxisVector where s,f is in [0,1]
  // This allows probing arbitrary 2D planar subspaces anywhere in the full DAC phase space.
  OperationResult God2D::timeSeriesBufferRamp2D(
      const std::vector<float> &args) {
    // Minimum required arguments:
    // 8 initial params + numDacChannels + 3*numDacChannels vectors + numAdcChannels
    if (args.size() < 8) {
      return OperationResult::Failure(
          "Not enough arguments provided for 2D ramp");
    }

    size_t currentIndex = 0;

    // Parse initial parameters
    int numDacChannels = static_cast<int>(args[currentIndex++]);
    int numAdcChannels = static_cast<int>(args[currentIndex++]);
    int numStepsFast = static_cast<int>(args[currentIndex++]);
    int numStepsSlow = static_cast<int>(args[currentIndex++]);
    const float dacIntervalArg = args[currentIndex++];
    const float adcIntervalArg = args[currentIndex++];
    const float retraceArg = args[currentIndex++];
    const float snakeArg = args[currentIndex++];

    if (!isValidDacChannelCount(numDacChannels) ||
        !isValidAdcChannelCount(numAdcChannels)) {
      return OperationResult::Failure("Invalid number of channels");
    }
    if (!isUint32AtLeast(adcIntervalArg, 1) ||
        !isUint32AtLeast(dacIntervalArg, 1)) {
      return OperationResult::Failure("Invalid interval");
    }
    if (!isBooleanArg(retraceArg) || !isBooleanArg(snakeArg)) {
      return OperationResult::Failure("Invalid 2D scan boolean argument");
    }
    if (numStepsFast < 1 || numStepsSlow < 1) {
      return OperationResult::Failure("Invalid number of steps");
    }
    uint32_t dac_interval_us = static_cast<uint32_t>(dacIntervalArg);
    uint32_t adc_interval_us = static_cast<uint32_t>(adcIntervalArg);
    bool retrace = retraceArg != 0.0f;
    bool snake = snakeArg != 0.0f;

    const size_t expected =
        currentIndex + static_cast<size_t>(numDacChannels) +
        3u * static_cast<size_t>(numDacChannels) +
        static_cast<size_t>(numAdcChannels);
    if (args.size() != expected) {
      return OperationResult::Failure("Incorrect number of arguments for 2D ramp");
    }

    // Parse DAC channel IDs
    int dacChannels[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      dacChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    // Parse start point
    float startPoint[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      startPoint[i] = args[currentIndex++];
    }

    // Parse fast axis vector
    float fastAxisVector[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      fastAxisVector[i] = args[currentIndex++];
    }

    // Parse slow axis vector
    float slowAxisVector[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      slowAxisVector[i] = args[currentIndex++];
    }

    // Parse ADC Channels
    if (args.size() < currentIndex + numAdcChannels) {
      return OperationResult::Failure("Not enough arguments for ADC channels");
    }

    int adcChannels[NUM_ADC_CHANNELS] = {};
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    OperationResult dacValidation =
        validateDacChannels(dacChannels, numDacChannels);
    if (!dacValidation.isSuccess()) {
      return dacValidation;
    }
    OperationResult adcValidation =
        validateAdcChannels(adcChannels, numAdcChannels);
    if (!adcValidation.isSuccess()) {
      return adcValidation;
    }

    // Check voltage bounds for all four corners of the 2D scan rectangle (both calibrated bounds AND global limits)
    for (int i = 0; i < numDacChannels; i++) {
      int ch = dacChannels[i];
      float lowerBound = DACController::getLowerBound(ch);
      float upperBound = DACController::getUpperBound(ch);
      
      // Four corners: startPoint, startPoint+fast, startPoint+slow, startPoint+fast+slow
      float corner1 = startPoint[i];
      float corner2 = startPoint[i] + fastAxisVector[i];
      float corner3 = startPoint[i] + slowAxisVector[i];
      float corner4 = startPoint[i] + fastAxisVector[i] + slowAxisVector[i];
      
      float minVoltage = min(min(corner1, corner2), min(corner3, corner4));
      float maxVoltage = max(max(corner1, corner2), max(corner3, corner4));
      
      if (minVoltage < lowerBound || maxVoltage > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " 2D scan range [" + String(minVoltage, 6) + 
                                        ", " + String(maxVoltage, 6) + 
                                        "]V exceeds bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
    }

    // Calculate slow axis step sizes for each DAC channel
    float slowStepSize[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; i++) {
      slowStepSize[i] =
          numStepsSlow > 1 ? slowAxisVector[i] / (numStepsSlow - 1) : 0.0f;
    }

    // Track current position in phase space (starting at startPoint)
    float currentSlowPosition[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; i++) {
      currentSlowPosition[i] = startPoint[i];
    }

    clearWorkerStopRequest();
    PeripheralCommsController::dataLedOn();

    uint8_t adcMask = 0u;
    God::BoardUsage boardUsage{0, std::vector<uint8_t>()};
    OperationResult prepareResult = God::prepareTimeSeriesBufferRampHardware(
        numAdcChannels, dac_interval_us, adc_interval_us, adcChannels, adcMask,
        boardUsage);
    if (!prepareResult.isSuccess()) {
      PeripheralCommsController::dataLedOff();
      return prepareResult;
    }

    OperationResult rampResult = OperationResult::Success();

    // Iterate over slow steps
    for (int slowStep = 0; slowStep < numStepsSlow && !isWorkerStopRequested(); ++slowStep) {
      // Determine ramp direction for snake mode
      bool isReverse = false;
      if (snake) {
        isReverse = (slowStep % 2 != 0);  // Reverse on odd slow steps
      }

      // Calculate start and end voltages for fast axis ramp
      // Position = currentSlowPosition + t * fastAxisVector (where t goes from 0 to 1)
      float fastV0s[NUM_DAC_CHANNELS] = {};
      float fastVfs[NUM_DAC_CHANNELS] = {};
      
      for (int i = 0; i < numDacChannels; ++i) {
        if (isReverse) {
          fastV0s[i] = currentSlowPosition[i] + fastAxisVector[i];
          fastVfs[i] = currentSlowPosition[i];
        } else {
          fastV0s[i] = currentSlowPosition[i];
          fastVfs[i] = currentSlowPosition[i] + fastAxisVector[i];
        }
      }

      // Execute fast axis ramp
      OperationResult ramp1Result = God::runPreparedTimeSeriesBufferRamp(
          numDacChannels, numAdcChannels, numStepsFast, dac_interval_us,
          adc_interval_us, dacChannels, fastV0s, fastVfs, adcChannels,
          adcMask);

      // Execute retrace if requested (and not in snake mode)
      OperationResult ramp2Result = OperationResult::Success();
      if (retrace && !snake) {
        ramp2Result = God::runPreparedTimeSeriesBufferRamp(
            numDacChannels, numAdcChannels, numStepsFast, dac_interval_us,
            adc_interval_us, dacChannels, fastVfs, fastV0s, adcChannels,
            adcMask);
      }

      // Check for errors
      if (!ramp1Result.isSuccess() && !ramp2Result.isSuccess()) {
        rampResult = OperationResult::Failure(ramp1Result.getMessage() + "\n" +
                                              ramp2Result.getMessage());
        break;
      } else if (!ramp1Result.isSuccess()) {
        rampResult = OperationResult::Failure(ramp1Result.getMessage());
        break;
      } else if (!ramp2Result.isSuccess()) {
        rampResult = OperationResult::Failure(ramp2Result.getMessage());
        break;
      }

      // Advance along slow axis
      for (int i = 0; i < numDacChannels; ++i) {
        currentSlowPosition[i] += slowStepSize[i];
      }
    }

    God::cleanupTimeSeriesBufferRampHardware(numAdcChannels, adcChannels,
                                             boardUsage);

    PeripheralCommsController::dataLedOff();

    if (!rampResult.isSuccess()) {
      if (isWorkerStopRequested()) {
        clearWorkerStopRequest();
      }
      return rampResult;
    }

    if (isWorkerStopRequested()) {
      clearWorkerStopRequest();
      return OperationResult::Failure("2D RAMPING_STOPPED");
    }

    return finishRampTimingWatchdog(false);
  }




  // dacLedBufferRamp2D:
  // Arguments (in order):
  // numDacChannels, numAdcChannels, numStepsFast, numStepsSlow,
  // dacInterval_us, dacSettlingTime_us, retrace (0.0f = false, 1.0f = true), snake (0.0f = false, 1.0f = true),
  // numAdcAverages,
  // [dacChannelID] * numDacChannels,
  // [startPoint] * numDacChannels,
  // [fastAxisVector] * numDacChannels,
  // [slowAxisVector] * numDacChannels,
  // [adcChannelID] * numAdcChannels
  //
  // The fast/slow axis vectors define a 2D plane in the N-dimensional DAC phase space.
  // Position(s,f) = startPoint + s*slowAxisVector + f*fastAxisVector where s,f is in [0,1]
  // This allows probing arbitrary 2D planar subspaces anywhere in the full DAC phase space.
  OperationResult God2D::dacLedBufferRamp2D(const std::vector<float> &args) {
    // Minimum required arguments:
    // 9 initial params + numDacChannels + 3*numDacChannels vectors + numAdcChannels
    if (args.size() < 9) {
      return OperationResult::Failure(
          "Not enough arguments provided for 2D ramp");
    }

    size_t currentIndex = 0;

    // Parse initial parameters
    int numDacChannels = static_cast<int>(args[currentIndex++]);
    int numAdcChannels = static_cast<int>(args[currentIndex++]);
    int numStepsFast = static_cast<int>(args[currentIndex++]);
    int numStepsSlow = static_cast<int>(args[currentIndex++]);
    const float dacIntervalArg = args[currentIndex++];
    const float dacSettlingTimeArg = args[currentIndex++];
    const float retraceArg = args[currentIndex++];
    const float snakeArg = args[currentIndex++];
    int numAdcAverages = static_cast<int>(args[currentIndex++]);

    if (!isValidDacChannelCount(numDacChannels) ||
        !isValidAdcChannelCount(numAdcChannels)) {
      return OperationResult::Failure("Invalid number of channels");
    }
    if (!isUint32AtLeast(dacSettlingTimeArg, 1) ||
        !isUint32AtLeast(dacIntervalArg, 1) ||
        dacSettlingTimeArg >= dacIntervalArg) {
      return OperationResult::Failure("Invalid interval or settling time");
    }
    if (!isBooleanArg(retraceArg) || !isBooleanArg(snakeArg)) {
      return OperationResult::Failure("Invalid 2D scan boolean argument");
    }
    if (numAdcAverages < 1) {
      return OperationResult::Failure("Invalid number of ADC averages");
    }
    if (numStepsFast < 1 || numStepsSlow < 1) {
      return OperationResult::Failure("Invalid number of steps");
    }
    uint32_t dac_interval_us = static_cast<uint32_t>(dacIntervalArg);
    uint32_t dac_settling_time_us =
        static_cast<uint32_t>(dacSettlingTimeArg);
    bool retrace = retraceArg != 0.0f;
    bool snake = snakeArg != 0.0f;

    const size_t expected =
        currentIndex + static_cast<size_t>(numDacChannels) +
        3u * static_cast<size_t>(numDacChannels) +
        static_cast<size_t>(numAdcChannels);
    if (args.size() != expected) {
      return OperationResult::Failure("Incorrect number of arguments for 2D ramp");
    }

    // Parse DAC channel IDs
    int dacChannels[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      dacChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    // Parse start point
    float startPoint[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      startPoint[i] = args[currentIndex++];
    }

    // Parse fast axis vector
    float fastAxisVector[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      fastAxisVector[i] = args[currentIndex++];
    }

    // Parse slow axis vector
    float slowAxisVector[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      slowAxisVector[i] = args[currentIndex++];
    }

    // Parse ADC Channels
    if (args.size() < currentIndex + numAdcChannels) {
      return OperationResult::Failure("Not enough arguments for ADC channels");
    }

    int adcChannels[NUM_ADC_CHANNELS] = {};
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[currentIndex++]);
    }

    OperationResult dacValidation =
        validateDacChannels(dacChannels, numDacChannels);
    if (!dacValidation.isSuccess()) {
      return dacValidation;
    }
    OperationResult adcValidation =
        validateAdcChannels(adcChannels, numAdcChannels);
    if (!adcValidation.isSuccess()) {
      return adcValidation;
    }

    // Check voltage bounds for all four corners of the 2D scan rectangle (both calibrated bounds AND global limits)
    for (int i = 0; i < numDacChannels; i++) {
      int ch = dacChannels[i];
      float lowerBound = max(DACController::getLowerBound(ch), DACLimits::lower_voltage_limit[ch]);
      float upperBound = min(DACController::getUpperBound(ch), DACLimits::upper_voltage_limit[ch]);
      
      // Four corners: startPoint, startPoint+fast, startPoint+slow, startPoint+fast+slow
      float corner1 = startPoint[i];
      float corner2 = startPoint[i] + fastAxisVector[i];
      float corner3 = startPoint[i] + slowAxisVector[i];
      float corner4 = startPoint[i] + fastAxisVector[i] + slowAxisVector[i];
      
      float minVoltage = min(min(corner1, corner2), min(corner3, corner4));
      float maxVoltage = max(max(corner1, corner2), max(corner3, corner4));
      
      if (minVoltage < lowerBound || maxVoltage > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " 2D scan range [" + String(minVoltage, 6) + 
                                        ", " + String(maxVoltage, 6) + 
                                        "]V exceeds bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
    }

    // Calculate slow axis step sizes for each DAC channel
    float slowStepSize[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; i++) {
      slowStepSize[i] =
          numStepsSlow > 1 ? slowAxisVector[i] / (numStepsSlow - 1) : 0.0f;
    }

    // Track current position in phase space (starting at startPoint)
    float currentSlowPosition[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; i++) {
      currentSlowPosition[i] = startPoint[i];
    }

    float fastV0s[NUM_DAC_CHANNELS] = {};
    float fastVfs[NUM_DAC_CHANNELS] = {};

    clearWorkerStopRequest();
    PeripheralCommsController::dataLedOn();

    uint8_t adcMask = 0u;
    God::BoardUsage boardUsage{0, std::vector<uint8_t>()};
    OperationResult prepareResult = God::prepareDacLedBufferRampHardware(
        numAdcChannels, numAdcAverages, dac_interval_us, dac_settling_time_us,
        adcChannels, adcMask, boardUsage);
    if (!prepareResult.isSuccess()) {
      PeripheralCommsController::dataLedOff();
      return prepareResult;
    }

    OperationResult rampResult = OperationResult::Success();

    // Iterate over slow steps
    for (int slowStep = 0; slowStep < numStepsSlow && !isWorkerStopRequested(); ++slowStep) {
      // Determine ramp direction for snake mode
      bool isReverse = false;
      if (snake) {
        isReverse = (slowStep % 2 != 0);  // Reverse on odd slow steps
      }

      // Calculate start and end voltages for fast axis ramp
      // Position = currentSlowPosition + t * fastAxisVector (where t goes from 0 to 1)
      if (isReverse) {
        for (int i = 0; i < numDacChannels; ++i) {
          fastV0s[i] = currentSlowPosition[i] + fastAxisVector[i];
          fastVfs[i] = currentSlowPosition[i];
        }
      } else {
        for (int i = 0; i < numDacChannels; ++i) {
          fastV0s[i] = currentSlowPosition[i];
          fastVfs[i] = currentSlowPosition[i] + fastAxisVector[i];
        }
      }

      // Execute fast axis ramp
      OperationResult ramp1Result = God::runPreparedDacLedBufferRamp(
          numDacChannels, numAdcChannels, numStepsFast, numAdcAverages,
          dacChannels, fastV0s, fastVfs, adcChannels, adcMask);

      // Execute retrace if requested (and not in snake mode)
      OperationResult ramp2Result = OperationResult::Success();
      if (retrace && !snake) {
        ramp2Result = God::runPreparedDacLedBufferRamp(
            numDacChannels, numAdcChannels, numStepsFast, numAdcAverages,
            dacChannels, fastVfs, fastV0s, adcChannels, adcMask);
      }

      // Check for errors
      if (!ramp1Result.isSuccess() && !ramp2Result.isSuccess()) {
        rampResult = OperationResult::Failure(ramp1Result.getMessage() + "\n" +
                                              ramp2Result.getMessage());
        break;
      } else if (!ramp1Result.isSuccess()) {
        rampResult = OperationResult::Failure(ramp1Result.getMessage());
        break;
      } else if (!ramp2Result.isSuccess()) {
        rampResult = OperationResult::Failure(ramp2Result.getMessage());
        break;
      }

      // Advance along slow axis
      for (int i = 0; i < numDacChannels; ++i) {
        currentSlowPosition[i] += slowStepSize[i];
      }
    }

    God::cleanupDacLedBufferRampHardware(numAdcChannels, adcChannels,
                                         boardUsage);

    PeripheralCommsController::dataLedOff();

    if (!rampResult.isSuccess()) {
      if (isWorkerStopRequested()) {
        clearWorkerStopRequest();
      }
      return rampResult;
    }

    if (isWorkerStopRequested()) {
      clearWorkerStopRequest();
      return OperationResult::Failure("2D RAMPING_STOPPED");
    }

    return finishRampTimingWatchdog();
  }
