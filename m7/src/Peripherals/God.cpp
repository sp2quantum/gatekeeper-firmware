#include "Peripherals/God.h"

#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Peripherals/PeripheralCommsController.h"
#include "Utils/FastGpio.h"

namespace {
using AdcIsr = void (*)();

AdcIsr kAdcSyncIsrFunctions[] = {
    TimingUtil::adcSyncISR<0>,
    TimingUtil::adcSyncISR<1>,
    TimingUtil::adcSyncISR<2>,
    TimingUtil::adcSyncISR<3>
};

void attachAdcSyncInterrupts(const God::BoardUsage& boardUsage) {
  for (int i = 0; i < boardUsage.numBoards; i++) {
    const int pin = ADCController::getDataReadyPin(boardUsage.idx[i]);
    if (pin == NC) {
      continue;
    }
    attachInterrupt(digitalPinToInterrupt(pin), kAdcSyncIsrFunctions[i],
                    FALLING);
  }
}

void detachAdcSyncInterrupts(const God::BoardUsage& boardUsage) {
  for (int i = 0; i < boardUsage.numBoards; i++) {
    const int pin = ADCController::getDataReadyPin(boardUsage.idx[i]);
    if (pin == NC) {
      continue;
    }
    detachInterrupt(digitalPinToInterrupt(pin));
  }
}

uint8_t adcMaskForBoardUsage(const God::BoardUsage& boardUsage) {
  uint8_t adcMask = 0u;
  for (int i = 0; i < boardUsage.numBoards; i++) {
    adcMask |= 1 << i;
  }
  return adcMask;
}

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

constexpr uint32_t kTimeSeriesAdcStartDelayUs = 10;

void setupTimeSeriesTimers(uint32_t dacIntervalUs, uint32_t adcIntervalUs,
                           uint8_t adcMask) {
  if (dacIntervalUs == adcIntervalUs &&
      kTimeSeriesAdcStartDelayUs < adcIntervalUs) {
    TimingUtil::setupTimersDacLed(dacIntervalUs, kTimeSeriesAdcStartDelayUs,
                                  adcMask);
    return;
  }
  TimingUtil::setupTimersTimeSeries(dacIntervalUs, adcIntervalUs, adcMask);
}

uint8_t adcBoardForChannel(int channel) {
  return static_cast<uint8_t>(channel / NUM_CHANNELS_PER_ADC_BOARD);
}

int maxSelectedAdcChannelsPerBoard(const int* adcChannels, int numAdcChannels) {
  int boardDepth[NUM_ADC_BOARDS] = {};
  for (int i = 0; i < numAdcChannels; i++) {
    const int channel = adcChannels[i];
    if (channel < 0 || channel >= NUM_ADC_CHANNELS) {
      continue;
    }
    boardDepth[adcBoardForChannel(channel)]++;
  }
  return *std::max_element(boardDepth, boardDepth + NUM_ADC_BOARDS);
}

float maxAdcConversionTimePerBoard(const int* adcChannels,
                                   int numAdcChannels) {
  float boardConversionTimeUs[NUM_ADC_BOARDS] = {};
  for (int i = 0; i < numAdcChannels; i++) {
    const int channel = adcChannels[i];
    if (channel < 0 || channel >= NUM_ADC_CHANNELS) {
      continue;
    }
    boardConversionTimeUs[adcBoardForChannel(channel)] +=
        ADCController::getConversionTimeFloat(channel);
  }
  return *std::max_element(boardConversionTimeUs,
                           boardConversionTimeUs + NUM_ADC_BOARDS);
}

OperationResult dacWriteFailure(int channel, double voltage) {
  String message = "DAC write failed ch=" + String(channel) +
                   " v=" + String(voltage, 9);

  if (!DACController::isChannelIndexValid(channel)) {
    return OperationResult::Failure(message + " source=invalid_channel");
  }

  const float lowerBound = DACController::getLowerBound(channel);
  const float upperBound = DACController::getUpperBound(channel);
  if (voltage < lowerBound || voltage > upperBound) {
    message += " source=bounds bounds=[" + String(lowerBound, 9) + "," +
               String(upperBound, 9) + "]";
    return OperationResult::Failure(message);
  }

  return OperationResult::Failure(message + " source=spi");
}

OperationResult validateDacChannels(const int* channels, int count) {
  bool seen[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < count; i++) {
    if (!DACController::isChannelIndexValid(channels[i])) {
      return OperationResult::Failure("Invalid DAC channel index " +
                                      String(channels[i]));
    }
    if (seen[channels[i]]) {
      return OperationResult::Failure("Duplicate DAC channel index " +
                                      String(channels[i]));
    }
    seen[channels[i]] = true;
  }
  return OperationResult::Success();
}

OperationResult validateAdcChannels(const int* channels, int count) {
  bool seen[NUM_ADC_CHANNELS] = {};
  for (int i = 0; i < count; i++) {
    if (!ADCController::isChannelIndexValid(channels[i])) {
      return OperationResult::Failure("Invalid ADC channel index " +
                                      String(channels[i]));
    }
    if (seen[channels[i]]) {
      return OperationResult::Failure("Duplicate ADC channel index " +
                                      String(channels[i]));
    }
    seen[channels[i]] = true;
  }
  return OperationResult::Success();
}

OperationResult validateDacLedBufferRampAdcConversionTimes(
    int numAdcChannels, const int* adcChannels) {
  bool selectedAdcChannels[NUM_ADC_CHANNELS] = {};
  uint8_t boardDepth[NUM_ADC_BOARDS] = {};

  for (int i = 0; i < numAdcChannels; i++) {
    const int channel = adcChannels[i];
    if (channel < 0 || channel >= NUM_ADC_CHANNELS ||
        selectedAdcChannels[channel]) {
      continue;
    }
    selectedAdcChannels[channel] = true;
    boardDepth[adcBoardForChannel(channel)]++;
  }

  for (int channel = 0; channel < NUM_ADC_CHANNELS; channel++) {
    if (!selectedAdcChannels[channel]) {
      continue;
    }
    const uint8_t board = adcBoardForChannel(channel);
    const bool multiChannelScan = boardDepth[board] > 1;
    const float conversionTimeUs =
        ADCController::getConversionTimeFloat(channel, multiChannelScan);
    if (conversionTimeUs < 0.0f) {
      return OperationResult::Failure("Invalid ADC conversion time");
    }
  }

  return OperationResult::Success();
}

bool sendVoltageFrame(const double* packets, size_t length) {
  if (sendVoltageFrameToGateway(packets, length)) {
    return true;
  }
  requestWorkerStop();
  return false;
}

bool encodeDacVoltagePackets(int numDacChannels, const int* dacChannels,
                             const double* voltages,
                             byte packets[NUM_DAC_CHANNELS][3]) {
  for (int i = 0; i < numDacChannels; i++) {
    if (!DACController::encodeVoltagePacket(
            dacChannels[i], static_cast<float>(voltages[i]), packets[i])) {
      return false;
    }
  }
  return true;
}

bool writeDacPackets(int numDacChannels, const int* dacChannels,
                     byte packets[NUM_DAC_CHANNELS][3]) {
  for (int i = 0; i < numDacChannels; i++) {
    if (!DACController::writeVoltagePacketNoLdac(dacChannels[i],
                                                 packets[i])) {
      return false;
    }
  }
  return true;
}

bool readAdcPackets(int numAdcChannels, const int* adcChannels,
                    double* packets, int startIndex = 0) {
  for (int i = startIndex; i < numAdcChannels; i++) {
    packets[i] = ADCController::getVoltageDataNoTransaction(adcChannels[i]);
  }
  return true;
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

  void God::setup() {
    initializeRegistry();
  }



  void God::initializeRegistry() {
    registerMemberFunction(initialize, "INITIALIZE");
    registerMemberFunction(initialize, "INIT");
    registerMemberFunction(initialize, "INNIT");
    registerMemberFunctionVector(timeSeriesBufferRampBase, "TIME_SERIES_BUFFER_RAMP");
    registerMemberFunctionVector(dacLedBufferRampBase, "DAC_LED_BUFFER_RAMP");
    registerMemberFunctionVector(AWGBufferRampWrapper, "AWG_BUFFER_RAMP");
    registerMemberFunctionVector(AWGWithADCWrapper, "AWG_WITH_ADC");
    registerMemberFunctionVector(timeSeriesAdcRead, "TIME_SERIES_ADC_READ");
    registerMemberFunction(dacChannelCalibration, "DAC_CH_CAL");
    registerMemberFunctionVector(boxcarAverageRamp, "BOXCAR_BUFFER_RAMP");
    registerMemberFunction(hardResetCalibrationToDefaults, "HARD_RESET_CALIBRATION");
  }



  OperationResult God::initialize() {
    DACController::initialize();
    ADCController::initialize();
    return OperationResult::Success("INITIALIZATION COMPLETE");
  }

  OperationResult God::hardResetCalibrationToDefaults() {
    CalibrationData calibrationData;
    readCalibrationData(calibrationData);
    for (int i = 0; i < NUM_DAC_CALIBRATION_CHANNELS; i++) {
      calibrationData.gain[i] = 1.0f;
      calibrationData.offset[i] = 0.0f;
    }
    for (int i = 0; i < NUM_ADC_CALIBRATION_CHANNELS; i++) {
      calibrationData.adc_offset[i] = 0x800000; // Default ADC offset
      calibrationData.adc_gain[i] = 0x200000; // Default ADC gain
    }
    calibrationData.adcCalibrated = false;
    updateCalibrationData(calibrationData);

    return OperationResult::Success("Calibration data reset to defaults");
  }



  God::BoardUsage God::getUsedBoards(const int *adcChannels, int numAdcChannels) {
    std::vector<uint8_t> boards;

    for (int i = 0; i < numAdcChannels; ++i) {
      const int ch = adcChannels[i];
      if (ch < 0 || ch >= NUM_ADC_CHANNELS) {
        continue;
      }
      uint8_t board = adcBoardForChannel(ch);
      if (std::find(boards.begin(), boards.end(), board) == boards.end()) {
        boards.push_back(board);
      }
    }

    std::sort(boards.begin(), boards.end());

    return BoardUsage{ static_cast<uint8_t>(boards.size()), boards };
  }



  OperationResult God::timeSeriesAdcRead(const std::vector<float>& args) {
    /**************************************************************************/
    // args: num_channels, channel_indexes, conversion_time, total_duration_us
    // ex: TIME_SERIES_ADC_READ 2,0,1,1000000
    //
    // Function takes a vector of floats as input, where:
    // args[0] = number of ADC channels = numAdcChannels
    // args[1] to args[numAdcChannels] = ADC channel indexes
    // args[numAdcChannels + 1] = conversion time in microseconds
    // args[numAdcChannels + 2] = total duration in microseconds
    /**************************************************************************/

    // Check if we have enough arguments
    if (args.size() < 4) {
      return OperationResult::Failure("Not enough arguments provided");
    }

    // Check if the first argument is a valid number of ADC channels
    int numAdcChannels = static_cast<int>(args[0]);
    if (!isValidAdcChannelCount(numAdcChannels)) {
      return OperationResult::Failure("Invalid number of ADC channels");
    }
    if (args.size() != static_cast<size_t>(numAdcChannels + 3)) {
      return OperationResult::Failure("Incorrect number of arguments");
    }

    // Setting up a vector of ADC channels from arguments directly after numAdcChannels
    std::vector<int> adcChannels_vec;
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels_vec.push_back(static_cast<int>(args[i + 1]));
    }
    int* adcChannels = adcChannels_vec.data();
    OperationResult adcValidation =
        validateAdcChannels(adcChannels, numAdcChannels);
    if (!adcValidation.isSuccess()) {
      return adcValidation;
    }

    // Check to see if the total duration is valid (needs to be at least 82 microseconds)
    const float conversionTimeArg = args[numAdcChannels + 1];
    const float totalDurationArg = args[numAdcChannels + 2];
    if (!isUint32AtLeast(conversionTimeArg, 1) ||
        !isUint32AtLeast(totalDurationArg, 82)) {
      return OperationResult::Failure("Invalid total duration");
    }
    uint32_t conversion_time = static_cast<uint32_t>(conversionTimeArg);
    uint32_t totalDuration = static_cast<uint32_t>(totalDurationArg);

    //Set conversion time for each channel (same conversion time for all channels)
    float realConversionTime = 0;
    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::setConversionTime(adcChannels[i], conversion_time);
    }
    realConversionTime = ADCController::getConversionTimeFloat(adcChannels[0]);

    const int max_indep_ADCs =
        maxSelectedAdcChannelsPerBoard(adcChannels, numAdcChannels);
    
    //set sampling rate based on +5% conversion time and the number of ADC channels
    const double sample_rate_float = max_indep_ADCs * realConversionTime * 1.5f;
    const int sample_rate = static_cast<int>(sample_rate_float);

    // Calculate the number of data points to save
    const int saved_data_size = totalDuration / sample_rate;

    // Send the number of bytes to expect before starting the ADC read
    if (!sendVoltageFrame(&sample_rate_float, 1)) {
      clearWorkerStopRequest();
      return OperationResult::Failure("Voltage output buffer overflow");
    }

    // Toggle stop flag and turn on data LED
    clearWorkerStopRequest();
    PeripheralCommsController::dataLedOn();

    FastGpio::digitalWrite(adc_sync, false); //Set the sync pin low to prevent ADCs from triggering
    BoardUsage boardUsage = getUsedBoards(adcChannels, numAdcChannels);
    attachAdcSyncInterrupts(boardUsage);

    // reset all ADCs before ramp
    ADCController::resetToPreviousConversionTimes();

    // Start continous conversion for each ADC channel
    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::startContinuousConversion(adcChannels[i]);
      ADCController::setRDYFN(adcChannels[i]);
    }

    // initialize ADC mask
    uint8_t adcMask = 0u;
    adcMask = adcMaskForBoardUsage(boardUsage);

    // Set up timers for ADC sampling
    TimingUtil::setupTimersOnlyADC(sample_rate);

    // initialize loop counter
    int x = 0;
    bool voltageOverflow = false;

    //Begin main loop
    while (x < saved_data_size && !isWorkerStopRequested()) {
      __WFE(); // Wait for event (WFE) to reduce CPU usage
      if (TimingUtil::consumeAdcFlag(adcMask)) {
        double packets[NUM_ADC_CHANNELS] = {};
        for (int i = 0; i < numAdcChannels; i++) {
          double v = ADCController::getVoltageDataNoTransaction(adcChannels[i]);
          packets[i] = v;
        }
        FastGpio::digitalWrite(adc_sync, false);
        if (!sendVoltageFrame(packets, numAdcChannels)) {
          voltageOverflow = true;
          break;
        }
        x++;
      }
    }

    // Clean up after the loop
    //Disable ADC timer interrupt
    TimingUtil::disableAdcInterrupt();

    // Set the ADCs into idle mode and unset the readyfnc bit (this allows single channel ADC conversions -- ready flag goes high after any ADC has unread data)
    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
      ADCController::unsetRDYFN(adcChannels[i]);
    }

    // reset all ADCs after ramp
    ADCController::resetToPreviousConversionTimes();

    //Detach hardware interrupt for ready pin on ADCs
    detachAdcSyncInterrupts(boardUsage);

    PeripheralCommsController::dataLedOff();

    if (isWorkerStopRequested()) {
      clearWorkerStopRequest();
      if (voltageOverflow) {
        return OperationResult::Failure("Voltage output buffer overflow");
      }
      return OperationResult::Failure("RAMPING_STOPPED");
    }

    return OperationResult::Success();
  }



  // args:
  // numDacChannels, numAdcChannels, numSteps, dacInterval_us, adcInterval_us,
  // dacchannel0, dacv00, dacvf0, dacchannel1, dacv01, dacvf1, ..., adc0, adc1,
  // adc2, ...
  OperationResult God::timeSeriesBufferRampBase(
      const std::vector<float>& args) {
    if (args.size() < 5) {
      return OperationResult::Failure("Not enough arguments provided");
    }

    int index = 0;

    int numDacChannels = static_cast<int>(args[index++]);
    int numAdcChannels = static_cast<int>(args[index++]);
    int numSteps = static_cast<int>(args[index++]);
    const float dacIntervalArg = args[index++];
    const float adcIntervalArg = args[index++];

    if (!isValidDacChannelCount(numDacChannels) ||
        !isValidAdcChannelCount(numAdcChannels)) {
      return OperationResult::Failure("Invalid number of channels");
    }
    if (!isUint32AtLeast(adcIntervalArg, 1) ||
        !isUint32AtLeast(dacIntervalArg, 1)) {
      return OperationResult::Failure("Invalid interval");
    }
    uint32_t dac_interval_us = static_cast<uint32_t>(dacIntervalArg);
    uint32_t adc_interval_us = static_cast<uint32_t>(adcIntervalArg);
    if (numSteps < 1) {
      return OperationResult::Failure("Invalid number of steps");
    }

    // Check if we have enough arguments for all DAC and ADC channels
    if (args.size() !=
        static_cast<size_t>(index + numDacChannels * 3 + numAdcChannels)) {
      return OperationResult::Failure("Incorrect number of arguments");
    }

    // Allocate memory for DAC and ADC channel information
    int dacChannels[NUM_DAC_CHANNELS] = {};
    float dacV0s[NUM_DAC_CHANNELS] = {};
    float dacVfs[NUM_DAC_CHANNELS] = {};
    int adcChannels[NUM_ADC_CHANNELS] = {};

    // Parse DAC channel information
    for (int i = 0; i < numDacChannels; ++i) {
      dacChannels[i] = static_cast<int>(args[index++]);
      dacV0s[i] = static_cast<float>(args[index++]);
      dacVfs[i] = static_cast<float>(args[index++]);
    }

    // Parse ADC channel information
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[index++]);
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

    for (int i = 0; i < numDacChannels; i++) {
      int ch = dacChannels[i];
      float lowerBound = DACController::getLowerBound(ch);
      float upperBound = DACController::getUpperBound(ch);

      if (dacV0s[i] < lowerBound || dacV0s[i] > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) +
                                        " start voltage " + String(dacV0s[i], 6) +
                                        "V out of bounds [" + String(lowerBound, 6) +
                                        ", " + String(upperBound, 6) + "]");
      }
      if (dacVfs[i] < lowerBound || dacVfs[i] > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) +
                                        " end voltage " + String(dacVfs[i], 6) +
                                        "V out of bounds [" + String(lowerBound, 6) +
                                        ", " + String(upperBound, 6) + "]");
      }
    }

    uint8_t adcMask = 0u;
    BoardUsage boardUsage{0, std::vector<uint8_t>()};
    clearWorkerStopRequest();
    PeripheralCommsController::dataLedOn();

    OperationResult prepareResult = prepareTimeSeriesBufferRampHardware(
        numAdcChannels, dac_interval_us, adc_interval_us, adcChannels, adcMask,
        boardUsage);
    if (!prepareResult.isSuccess()) {
      PeripheralCommsController::dataLedOff();
      return prepareResult;
    }

    OperationResult rampResult = runPreparedTimeSeriesBufferRamp(
        numDacChannels, numAdcChannels, numSteps, dac_interval_us,
        adc_interval_us, dacChannels, dacV0s, dacVfs, adcChannels, adcMask);

    cleanupTimeSeriesBufferRampHardware(numAdcChannels, adcChannels, boardUsage);
    PeripheralCommsController::dataLedOff();

    if (!rampResult.isSuccess()) {
      if (isWorkerStopRequested()) {
        clearWorkerStopRequest();
      }
      return rampResult;
    }

    if (isWorkerStopRequested()) {
      clearWorkerStopRequest();
      return OperationResult::Failure("RAMPING_STOPPED");
    }

    return finishRampTimingWatchdog(false);
  }



  OperationResult God::prepareTimeSeriesBufferRampHardware(
      int numAdcChannels, uint32_t dac_interval_us, uint32_t adc_interval_us,
      int* adcChannels, uint8_t& adcMask, BoardUsage& boardUsage) {
    adcMask = 0u;
    boardUsage = BoardUsage{0, std::vector<uint8_t>()};

    FastGpio::digitalWrite(adc_sync, false);

    // if (maxConvTime + 300 >= adc_interval_us) {
    //   return OperationResult::Failure(
    //       "ADC delay time is too short, please increase it");
    // }

    // if (max_indep_ADCs <= 0) {
    //   return OperationResult::Failure("No ADC channels provided");
    // } else if (max_indep_ADCs == 1) {
    //   if (dac_interval_us < 60) {
    //     return OperationResult::Failure("DAC interval too short, please increase it");
    //   }
    // } else if (max_indep_ADCs == 2) {
    //   if (dac_interval_us < 120) {
    //     return OperationResult::Failure("DAC interval too short, please increase it");
    //   }
    // } else if (max_indep_ADCs == 3) {
    //   if (dac_interval_us < 180) {
    //     return OperationResult::Failure("DAC interval too short, please increase it");
    //   }
    // } else if (max_indep_ADCs == 4) {
    //   if (dac_interval_us < 250) {
    //     return OperationResult::Failure("DAC interval too short, please increase it");
    //   }
    // }

    boardUsage = getUsedBoards(adcChannels, numAdcChannels);
    attachAdcSyncInterrupts(boardUsage);
    adcMask = adcMaskForBoardUsage(boardUsage);

    ADCController::resetToPreviousConversionTimes();
    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::startContinuousConversion(adcChannels[i]);
      ADCController::setRDYFN(adcChannels[i]);
    }

    setupTimeSeriesTimers(dac_interval_us, adc_interval_us, adcMask);
    TimingUtil::dacFlag = false;
    TimingUtil::adcFlag = 0;

    return OperationResult::Success();
  }



  OperationResult God::runPreparedTimeSeriesBufferRamp(
      int numDacChannels, int numAdcChannels, int numSteps,
      uint32_t dac_interval_us, uint32_t adc_interval_us, int* dacChannels,
      float* dacV0s, float* dacVfs, int* adcChannels, uint8_t adcMask,
      int initialAdcSamplesToDiscard,
      bool holdInitialDacUntilDiscarded,
      bool discardAdcSampleAfterDacStep,
      bool holdInitialDacThroughFirstVisibleSample) {
    int steps = 0;
    int x = 0;
    const int saved_data_size = numSteps * dac_interval_us / adc_interval_us;
    int discardedAdcSamples = 0;
    const int discardCount =
        initialAdcSamplesToDiscard > 0 ? initialAdcSamplesToDiscard : 0;
    bool discardCurrentDacSample = false;
    bool waitingForVisibleCurrentDacSample = false;
    bool voltageOverflow = false;

    double voltageStepSize[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; i++) {
      voltageStepSize[i] =
          numSteps > 1 ? (dacVfs[i] - dacV0s[i]) / (numSteps - 1) : 0.0;
    }

    double nextVoltageSet[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; i++) {
      nextVoltageSet[i] = dacV0s[i];
    }
    byte nextDacPackets[NUM_DAC_CHANNELS][3] = {};
    bool nextDacPacketsReady = false;
    auto prepareNextDacPackets = [&]() {
      if (steps >= numSteps) {
        nextDacPacketsReady = false;
        return true;
      }
      nextDacPacketsReady =
          encodeDacVoltagePackets(numDacChannels, dacChannels, nextVoltageSet,
                                  nextDacPackets);
      return nextDacPacketsReady;
    };

    for (int i = 0; i < numDacChannels; i++) {
      if (!DACController::setVoltageNoTransactionNoLdac(dacChannels[i],
                                                        dacV0s[i])) {
        return dacWriteFailure(dacChannels[i], dacV0s[i]);
      }
      nextVoltageSet[i] += voltageStepSize[i];
    }
    DACController::toggleLdac();
    steps++;
    byte holdDacPackets[NUM_DAC_CHANNELS][3] = {};
    double holdVoltageSet[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; i++) {
      holdVoltageSet[i] = dacV0s[i];
    }
    if ((holdInitialDacUntilDiscarded || discardAdcSampleAfterDacStep) &&
        !encodeDacVoltagePackets(numDacChannels, dacChannels,
                                 holdVoltageSet, holdDacPackets)) {
      return dacWriteFailure(dacChannels[0], holdVoltageSet[0]);
    }
    if (!prepareNextDacPackets()) {
      return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
    }
    FastGpio::digitalWrite(adc_sync, false);
    TimingUtil::dacFlag = false;
    TimingUtil::adcFlag = 0;
    bool dacWriteQueued = false;

    while ((discardedAdcSamples < discardCount || x < saved_data_size ||
            steps < numSteps) &&
           !isWorkerStopRequested()) {
      __WFE();
      const bool adcPending =
          (discardedAdcSamples < discardCount || x < saved_data_size) &&
          TimingUtil::consumeAdcFlag(adcMask);
      const bool holdingInitialDac =
          holdInitialDacUntilDiscarded &&
          (discardedAdcSamples < discardCount ||
           (holdInitialDacThroughFirstVisibleSample && x == 0));
      const bool holdingInitialDacBeforeAdc = holdingInitialDac;
      if ((holdingInitialDac || discardCurrentDacSample ||
           waitingForVisibleCurrentDacSample || steps < numSteps) &&
          TimingUtil::consumeDacFlag()) {
        dacWriteQueued = true;
      }

      double packets[NUM_ADC_CHANNELS] = {};
      bool haveAdcPackets = false;
      if (adcPending) {
        readAdcPackets(numAdcChannels, adcChannels, packets);
        FastGpio::digitalWrite(adc_sync, false);
        if (discardedAdcSamples < discardCount) {
          discardedAdcSamples++;
        } else if (discardAdcSampleAfterDacStep &&
                   discardCurrentDacSample) {
          discardCurrentDacSample = false;
          waitingForVisibleCurrentDacSample = true;
        } else {
          haveAdcPackets = true;
          waitingForVisibleCurrentDacSample = false;
        }
      }
      if (dacWriteQueued && TimingUtil::adcConversionInProgressMask == 0) {
        if (holdingInitialDacBeforeAdc) {
          // The initial DAC code was written before the timers started; keep it
          // latched without burning SPI cycles during the hidden ADC sample.
          dacWriteQueued = false;
        } else if (discardCurrentDacSample ||
                   waitingForVisibleCurrentDacSample) {
          if (!writeDacPackets(numDacChannels, dacChannels, holdDacPackets)) {
            return dacWriteFailure(dacChannels[0], holdVoltageSet[0]);
          }
          dacWriteQueued = false;
        } else if (!nextDacPacketsReady ||
                   !writeDacPackets(numDacChannels, dacChannels,
                                    nextDacPackets)) {
          return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
        } else {
          dacWriteQueued = false;
          for (int i = 0; i < numDacChannels; i++) {
            nextVoltageSet[i] += voltageStepSize[i];
          }
          steps++;
          for (int i = 0; i < numDacChannels; i++) {
            holdVoltageSet[i] = nextVoltageSet[i] - voltageStepSize[i];
            for (int j = 0; j < 3; j++) {
              holdDacPackets[i][j] = nextDacPackets[i][j];
            }
          }
          if (discardAdcSampleAfterDacStep) {
            discardCurrentDacSample = true;
            waitingForVisibleCurrentDacSample = false;
          }
          if (!prepareNextDacPackets()) {
            return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
          }
        }
      }
      if (haveAdcPackets) {
        if (!sendVoltageFrame(packets, numAdcChannels)) {
          voltageOverflow = true;
          break;
        }
        x++;
      }
    }

    if (isWorkerStopRequested()) {
      if (voltageOverflow) {
        return OperationResult::Failure("Voltage output buffer overflow");
      }
      return OperationResult::Failure("RAMPING_STOPPED");
    }
    if (voltageOverflow) {
      return OperationResult::Failure("Voltage output buffer overflow");
    }

    return OperationResult::Success();
  }



  void God::cleanupTimeSeriesBufferRampHardware(
      int numAdcChannels, int* adcChannels, const BoardUsage& boardUsage) {
    TimingUtil::disableDacInterrupt();
    TimingUtil::disableAdcInterrupt();
    TimingUtil::dacFlag = false;
    TimingUtil::adcFlag = 0;

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
      ADCController::unsetRDYFN(adcChannels[i]);
    }

    ADCController::resetToPreviousConversionTimes();

    detachAdcSyncInterrupts(boardUsage);
  }



  // args:
  // numDacChannels, numAdcChannels, numSteps, numAdcAverages, dacInterval_us,
  // dacSettlingTime_us, dacchannel0, dacv00, dacvf0, dacchannel1, dacv01,
  // dacvf1, ..., adc0, adc1, adc2, ...
  OperationResult God::dacLedBufferRampBase(
      const std::vector<float>& args) {
    if (args.size() < 10) {
      return OperationResult::Failure("Not enough arguments provided");
    }

    int index = 0;

    int numDacChannels = static_cast<int>(args[index++]);
    int numAdcChannels = static_cast<int>(args[index++]);
    int numSteps = static_cast<int>(args[index++]);
    int numAdcAverages = static_cast<int>(args[index++]);
    const float dacIntervalArg = args[index++];
    const float dacSettlingTimeArg = args[index++];

    if (!isValidDacChannelCount(numDacChannels) ||
        !isValidAdcChannelCount(numAdcChannels)) {
      return OperationResult::Failure("Invalid number of channels");
    }
    if (!isUint32AtLeast(dacSettlingTimeArg, 1) ||
        !isUint32AtLeast(dacIntervalArg, 1) ||
        dacSettlingTimeArg >= dacIntervalArg) {
      return OperationResult::Failure("Invalid interval or settling time");
    }
    uint32_t dac_interval_us = static_cast<uint32_t>(dacIntervalArg);
    uint32_t dac_settling_time_us =
        static_cast<uint32_t>(dacSettlingTimeArg);
    if (numAdcAverages < 1) {
      return OperationResult::Failure("Invalid number of ADC averages");
    }
    if (numSteps < 1) {
      return OperationResult::Failure("Invalid number of steps");
    }

    // Check if we have enough arguments for all DAC and ADC channels
    if (args.size() !=
        static_cast<size_t>(index + numDacChannels * 3 + numAdcChannels)) {
      return OperationResult::Failure("Incorrect number of arguments");
    }

    // Allocate memory for DAC and ADC channel information
    int dacChannels[NUM_DAC_CHANNELS] = {};
    float dacV0s[NUM_DAC_CHANNELS] = {};
    float dacVfs[NUM_DAC_CHANNELS] = {};
    int adcChannels[NUM_ADC_CHANNELS] = {};

    // Parse DAC channel information
    for (int i = 0; i < numDacChannels; ++i) {
      dacChannels[i] = static_cast<int>(args[index++]);
      dacV0s[i] = static_cast<float>(args[index++]);
      dacVfs[i] = static_cast<float>(args[index++]);
    }

    // Parse ADC channel information
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[index++]);
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

    for (int i = 0; i < numDacChannels; i++) {
      int ch = dacChannels[i];
      float lowerBound = DACController::getLowerBound(ch);
      float upperBound = DACController::getUpperBound(ch);

      if (dacV0s[i] < lowerBound || dacV0s[i] > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) +
                                        " start voltage " + String(dacV0s[i], 6) +
                                        "V out of bounds [" + String(lowerBound, 6) +
                                        ", " + String(upperBound, 6) + "]");
      }
      if (dacVfs[i] < lowerBound || dacVfs[i] > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) +
                                        " end voltage " + String(dacVfs[i], 6) +
                                        "V out of bounds [" + String(lowerBound, 6) +
                                        ", " + String(upperBound, 6) + "]");
      }
    }

    uint8_t adcMask = 0u;
    BoardUsage boardUsage{0, std::vector<uint8_t>()};
    clearWorkerStopRequest();
    PeripheralCommsController::dataLedOn();

    OperationResult prepareResult = prepareDacLedBufferRampHardware(
        numAdcChannels, numAdcAverages, dac_interval_us, dac_settling_time_us,
        adcChannels, adcMask, boardUsage);
    if (!prepareResult.isSuccess()) {
      PeripheralCommsController::dataLedOff();
      return prepareResult;
    }

    OperationResult rampResult = runPreparedDacLedBufferRamp(
        numDacChannels, numAdcChannels, numSteps, numAdcAverages, dacChannels,
        dacV0s, dacVfs, adcChannels, adcMask);

    cleanupDacLedBufferRampHardware(numAdcChannels, adcChannels, boardUsage);
    PeripheralCommsController::dataLedOff();

    if (!rampResult.isSuccess()) {
      if (isWorkerStopRequested()) {
        clearWorkerStopRequest();
      }
      return rampResult;
    }

    if (isWorkerStopRequested()) {
      clearWorkerStopRequest();
      return OperationResult::Failure("RAMPING_STOPPED");
    }

    return finishRampTimingWatchdog();
  }



  OperationResult God::prepareDacLedBufferRampHardware(
      int numAdcChannels, int numAdcAverages, uint32_t dac_interval_us,
      uint32_t dac_settling_time_us, int* adcChannels, uint8_t& adcMask,
      BoardUsage& boardUsage, bool startTimers) {
    adcMask = 0u;
    boardUsage = BoardUsage{0, std::vector<uint8_t>()};

    ADCController::resetToPreviousConversionTimes();

    FastGpio::digitalWrite(adc_sync, false);

    OperationResult timingValidation =
        validateDacLedBufferRampAdcConversionTimes(numAdcChannels, adcChannels);
    if (!timingValidation.isSuccess()) {
      return timingValidation;
    }

    boardUsage = getUsedBoards(adcChannels, numAdcChannels);


    attachAdcSyncInterrupts(boardUsage);
    adcMask = adcMaskForBoardUsage(boardUsage);

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::startContinuousConversion(adcChannels[i]);
      ADCController::setRDYFN(adcChannels[i]);
    }

    if (startTimers) {
      TimingUtil::setupTimersDacLed(dac_interval_us, dac_settling_time_us,
                                    adcMask);
      TimingUtil::dacFlag = false;
      TimingUtil::adcFlag = 0;
    }

    return OperationResult::Success();
  }



  OperationResult God::runPreparedDacLedBufferRamp(
      int numDacChannels, int numAdcChannels, int numSteps, int numAdcAverages,
      int* dacChannels, float* dacV0s, float* dacVfs, int* adcChannels,
      uint8_t adcMask) {
    double packets[NUM_ADC_CHANNELS] = {};
    double voltageStepSize[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; i++) {
      voltageStepSize[i] =
          numSteps > 1 ? (dacVfs[i] - dacV0s[i]) / (numSteps - 1) : 0.0;
    }

    double numAdcAveragesInv = 1.0 / static_cast<double>(numAdcAverages);
    int dacIncrements = 0;
    double nextVoltageSet[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; i++) {
      nextVoltageSet[i] = dacV0s[i];
    }
    byte nextDacPackets[NUM_DAC_CHANNELS][3] = {};
    bool nextDacPacketsReady = false;
    auto prepareNextDacPackets = [&]() {
      if (dacIncrements >= numSteps) {
        nextDacPacketsReady = false;
        return true;
      }
      nextDacPacketsReady =
          encodeDacVoltagePackets(numDacChannels, dacChannels, nextVoltageSet,
                                  nextDacPackets);
      return nextDacPacketsReady;
    };

    for (int i = 0; i < numDacChannels; i++) {
      if (!DACController::setVoltageNoTransactionNoLdac(dacChannels[i],
                                                        dacV0s[i])) {
        return dacWriteFailure(dacChannels[i], dacV0s[i]);
      }
      nextVoltageSet[i] += voltageStepSize[i];
    }
    dacIncrements++;
    if (!prepareNextDacPackets()) {
      return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
    }
    FastGpio::digitalWrite(adc_sync, false);
    int x = 0;
    bool voltageOverflow = false;

    while (x < numSteps && !isWorkerStopRequested()) {
      __WFE();

      const bool adcPending = TimingUtil::consumeAdcFlag(adcMask);

      bool haveAdcPackets = false;
      if (adcPending) {
        x++;
        for (int i = 0; i < numAdcChannels; i++) {
          double total = 0.0;
          for (int j = 0; j < numAdcAverages; j++) {
            total += ADCController::getVoltageDataNoTransaction(adcChannels[i]);
          }
          packets[i] = total * numAdcAveragesInv;
        }
        FastGpio::digitalWrite(adc_sync, false);
        haveAdcPackets = true;
      }

      const bool dacPending =
          dacIncrements < numSteps && TimingUtil::consumeDacFlag();
      if (dacPending) {
        if (!nextDacPacketsReady ||
            !writeDacPackets(numDacChannels, dacChannels, nextDacPackets)) {
          return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
        }
        for (int i = 0; i < numDacChannels; i++) {
          nextVoltageSet[i] += voltageStepSize[i];
        }
        dacIncrements++;
        if (!prepareNextDacPackets()) {
          return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
        }
      }

      if (haveAdcPackets) {
        if (!sendVoltageFrame(packets, numAdcChannels)) {
          voltageOverflow = true;
          break;
        }
      }
    }

    if (isWorkerStopRequested()) {
      if (voltageOverflow) {
        return OperationResult::Failure("Voltage output buffer overflow");
      }
      return OperationResult::Failure("RAMPING_STOPPED");
    }
    if (voltageOverflow) {
      return OperationResult::Failure("Voltage output buffer overflow");
    }

    return OperationResult::Success();
  }



  void God::cleanupDacLedBufferRampHardware(
      int numAdcChannels, int* adcChannels, const BoardUsage& boardUsage) {
    TimingUtil::disableDacInterrupt();
    TimingUtil::disableAdcInterrupt();
    TimingUtil::dacFlag = false;
    TimingUtil::adcFlag = 0;

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
      ADCController::unsetRDYFN(adcChannels[i]);
    }

    detachAdcSyncInterrupts(boardUsage);

    ADCController::resetToPreviousConversionTimes();
  }



  OperationResult God::OwenRampWrapper(std::vector<float> args) {
    // Expected argument order:
    // [numDacChannels, numAdcChannels, numLoops, numDacStepsPerLoop, numAdcAverages, dac_interval_us, <dacChannels...>, <adcChannels...>, <dacVoltageLists...>, specialIndex, specialWidth, numStepsPerSpecialRamp, <specialDacV0s...>, <specialDacVfs...>]
    // The number of DAC and ADC channels determines how many channel indices and voltage lists to expect.

    if (args.size() < 6) {
      return OperationResult::Failure("Insufficient arguments for OwenRampWrapper");
    }

    int idx = 0;
    int numDacChannels = static_cast<int>(args[idx++]);
    int numAdcChannels = static_cast<int>(args[idx++]);
    int numLoops = static_cast<int>(args[idx++]);
    int numDacStepsPerLoop = static_cast<int>(args[idx++]);
    int numAdcAverages = static_cast<int>(args[idx++]);
    const float dacIntervalArg = args[idx++];

    // Check for valid channel counts
    if (!isValidDacChannelCount(numDacChannels) ||
        !isValidAdcChannelCount(numAdcChannels) ||
        numLoops < 1 || numDacStepsPerLoop < 1 || numAdcAverages < 1 ||
        !isUint32AtLeast(dacIntervalArg, 1)) {
      return OperationResult::Failure("Invalid channel or loop/step/average count");
    }
    uint32_t dac_interval_us = static_cast<uint32_t>(dacIntervalArg);

    const size_t expected =
        6u + static_cast<size_t>(numDacChannels) +
        static_cast<size_t>(numAdcChannels) +
        static_cast<size_t>(numDacChannels) *
            static_cast<size_t>(numDacStepsPerLoop) +
        3u + 2u * static_cast<size_t>(numDacChannels);
    if (args.size() != expected) {
      return OperationResult::Failure("Invalid argument count for OwenRampWrapper");
    }

    // Parse DAC channel indices
    int dacChannels[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      dacChannels[i] = static_cast<int>(args[idx++]);
    }

    // Parse ADC channel indices
    int adcChannels[NUM_ADC_CHANNELS] = {};
    for (int i = 0; i < numAdcChannels; ++i) {
      adcChannels[i] = static_cast<int>(args[idx++]);
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

    // Parse DAC voltage lists
    std::vector<float> dacVoltageStorage(
        static_cast<size_t>(numDacChannels) *
        static_cast<size_t>(numDacStepsPerLoop));
    float* dacVoltageLists[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      dacVoltageLists[i] =
          &dacVoltageStorage[static_cast<size_t>(i) *
                             static_cast<size_t>(numDacStepsPerLoop)];
      for (int j = 0; j < numDacStepsPerLoop; ++j) {
        dacVoltageLists[i][j] = args[idx++];
      }
    }

    // Parse special ramp parameters
    int specialIndex = static_cast<int>(args[idx++]);
    int specialWidth = static_cast<int>(args[idx++]);
    int numStepsPerSpecialRamp = static_cast<int>(args[idx++]);
    if (specialIndex < 0 || specialIndex >= numDacStepsPerLoop ||
        specialWidth < 0 || numStepsPerSpecialRamp < 1) {
      return OperationResult::Failure("Invalid Owen special ramp parameters");
    }

    float specialDacV0s[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      specialDacV0s[i] = args[idx++];
    }
    float specialDacVfs[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < numDacChannels; ++i) {
      specialDacVfs[i] = args[idx++];
    }

    return OwenRampBase(numDacChannels, numAdcChannels, numLoops, numDacStepsPerLoop, numAdcAverages, dac_interval_us, dacChannels, dacVoltageLists, adcChannels, specialIndex, specialWidth, numStepsPerSpecialRamp, specialDacV0s, specialDacVfs);
  }



  OperationResult God::OwenRampBase(
    int numDacChannels, int numAdcChannels, int numLoops, int numDacStepsPerLoop, int numAdcAverages,
    uint32_t dac_interval_us, int* dacChannels,
    float** dacVoltageLists, int* adcChannels, int specialIndex, int specialWidth, int numStepsPerSpecialRamp, float* specialDacV0s, float* specialDacVfs) {

      if (dac_interval_us < 1) {
        return OperationResult::Failure("Invalid interval or settling time");
      }
      if (numAdcAverages < 1) {
        return OperationResult::Failure("Invalid number of ADC averages");
      }
      if (numLoops < 1 || numDacStepsPerLoop < 1) {
        return OperationResult::Failure("Invalid number of loops or steps per loop");
      }
      if (!isValidDacChannelCount(numDacChannels) ||
          !isValidAdcChannelCount(numAdcChannels)) {
        return OperationResult::Failure("Invalid number of channels");
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

      for (int i = 0; i < numDacChannels; i++) {
        int ch = dacChannels[i];
        float lowerBound = DACController::getLowerBound(ch);
        float upperBound = DACController::getUpperBound(ch);
        for (int j = 0; j < numDacStepsPerLoop; j++) {
          float voltage = dacVoltageLists[i][j];
          if (voltage < lowerBound || voltage > upperBound) {
            return OperationResult::Failure("DAC " + String(ch) +
                                            " voltage[" + String(j) + "] = " +
                                            String(voltage, 6) +
                                            "V out of bounds [" +
                                            String(lowerBound, 6) + ", " +
                                            String(upperBound, 6) + "]");
          }
        }
        if (specialDacV0s[i] < lowerBound || specialDacV0s[i] > upperBound ||
            specialDacVfs[i] < lowerBound || specialDacVfs[i] > upperBound) {
          return OperationResult::Failure("DAC " + String(ch) +
                                          " special ramp voltage out of bounds");
        }
      }
      
      double packets[NUM_ADC_CHANNELS] = {};
      double numAdcAveragesInv = 1.0 / static_cast<double>(numAdcAverages);
  
      clearWorkerStopRequest();
      PeripheralCommsController::dataLedOn();
  
      ADCController::resetToPreviousConversionTimes();
  
      FastGpio::digitalWrite(adc_sync, false);
  
      BoardUsage boardUsage = getUsedBoards(adcChannels, numAdcChannels);
  
      const float maxConvTime =
          maxAdcConversionTimePerBoard(adcChannels, numAdcChannels);
      uint32_t totalDacSweepTime = numDacStepsPerLoop * dac_interval_us;
      if (maxConvTime * numAdcAverages + 180 >= totalDacSweepTime) {
        PeripheralCommsController::dataLedOff();
        return OperationResult::Failure(
            "DAC sweep time is too short for specified ADC conversion time, "
            "please increase dac_interval_us or reduce numDacStepsPerLoop");
      }
  
      attachAdcSyncInterrupts(boardUsage);
  
      // Initialize timing flags
      TimingUtil::dacFlag = false;
      TimingUtil::adcFlag = 0;
  
      // Track current position in voltage lists and loop
      int currentLoop = 0;
      int totalDacSteps = numLoops * numDacStepsPerLoop;
      int currentDacStep = 0;
      int currentAdcReads = 0;
      bool voltageOverflow = false;

      float currentSpecialDacVoltages[NUM_DAC_CHANNELS] = {};
      float specialDacVoltageStep[NUM_DAC_CHANNELS] = {};

      for (int i = 0; i < numDacChannels; i++) {
        currentSpecialDacVoltages[i] = specialDacV0s[i];
      }

      for (int i = 0; i < numDacChannels; i++) {
        specialDacVoltageStep[i] = (specialDacVfs[i] - specialDacV0s[i]) / numLoops;
      }

  
      // Set initial DAC voltages (first step of first loop)
      for (int i = 0; i < numDacChannels; i++) {
        DACController::setVoltageNoTransactionNoLdac(dacChannels[i], dacVoltageLists[i][0]);
      }
      currentDacStep++;
  
      // Start ADC continuous conversion
      for (int i = 0; i < numAdcChannels; i++) {
        ADCController::startContinuousConversion(adcChannels[i]);
        ADCController::setRDYFN(adcChannels[i]);
      }
  
      // Setup timers for DAC and ADC events
      TimingUtil::setupTimerOnlyDac(dac_interval_us);
      TimingUtil::dacFlag = false;
  
      bool done = false;

      int subIndex = 0;
  
      // Main event loop using interrupt-based timing
      while (currentLoop < numLoops && !isWorkerStopRequested()) {
        __WFE(); // Wait for event (interrupt)
        
        // Handle DAC flag - time to set next DAC voltage
        if (currentDacStep < totalDacSteps && TimingUtil::consumeDacFlag()) {

          if (currentDacStep == specialIndex) {
            for (int i = 0; i < numDacChannels; i++) {
              DACController::setVoltageNoTransactionNoLdac(dacChannels[i], currentSpecialDacVoltages[i]);
            }
            subIndex++;
            if (subIndex >= numStepsPerSpecialRamp) {
              subIndex = 0;
              currentDacStep++;
              for (int i = 0; i < numDacChannels; i++) {
                currentSpecialDacVoltages[i] += specialDacVoltageStep[i];
              }
            }
          } else {
            for (int i = 0; i < numDacChannels; i++) {
              float voltage = dacVoltageLists[i][currentDacStep];
              DACController::setVoltageNoTransactionNoLdac(dacChannels[i], voltage);
            }
            currentDacStep++;
          }
          
  
          // Check if we've completed a full sweep of voltages for this loop
          if (currentDacStep >= numDacStepsPerLoop) {
            currentDacStep = 0; // Reset to beginning of voltage list for next loop
            done = true; // Mark that we need to read ADC after settling
          }
          
        }
        
        // Handle ADC flag - time to read ADC after settling
        if (done) {
          done = false; // Reset done flag for next ADC read
          for (int i = 0; i < numAdcChannels; i++) {
            double total = 0.0;
            for (int j = 0; j < numAdcAverages; j++) {
              total += ADCController::getVoltage(adcChannels[i]);
            }
            packets[i] = total * numAdcAveragesInv;
          }
          if (!sendVoltageFrame(packets, numAdcChannels)) {
            voltageOverflow = true;
          }
          
          FastGpio::digitalWrite(adc_sync, false);
          currentAdcReads++;
          currentLoop++; // Each ADC read marks completion of one loop
        }
      }
  
      // Clean up timers
      TimingUtil::disableDacInterrupt();
      TimingUtil::disableAdcInterrupt();
      TimingUtil::dacFlag = false;
      TimingUtil::adcFlag = 0;
  
      // Clean up
      for (int i = 0; i < numAdcChannels; i++) {
        ADCController::idleMode(adcChannels[i]);
        ADCController::unsetRDYFN(adcChannels[i]);
      }
  
      detachAdcSyncInterrupts(boardUsage);
  
      ADCController::resetToPreviousConversionTimes();
      PeripheralCommsController::dataLedOff();
  
      if (isWorkerStopRequested()) {
        clearWorkerStopRequest();
        if (voltageOverflow) {
          return OperationResult::Failure("Voltage output buffer overflow");
        }
        return OperationResult::Failure("RAMPING_STOPPED");
      }
  
      return finishRampTimingWatchdog();
    }




  OperationResult God::AWGBufferRampWrapper(std::vector<float> args) {
    //   AWG_BUFFER_RAMP,<dacN>,<numSteps>,<dacInterval_us>,<dacPorts...>,<voltages...>
    //
    // Voltages are channel-major: all points for DAC0, then all points for DAC1, ...

    if (args.size() < 3) {
      return OperationResult::Failure("Insufficient arguments for AWG_BUFFER_RAMP");
    }

    int idx = 0;
    const int dacN = static_cast<int>(args[idx++]);
    const int numSteps = static_cast<int>(args[idx++]);
    const float dacIntervalArg = args[idx++];

    if (!isValidDacChannelCount(dacN) || numSteps < 1 ||
        !isUint32AtLeast(dacIntervalArg, 1)) {
      return OperationResult::Failure("Invalid number of channels or steps");
    }
    const uint32_t dac_interval_us = static_cast<uint32_t>(dacIntervalArg);

    const size_t expected =
        3u + static_cast<size_t>(dacN) +
        static_cast<size_t>(dacN) * static_cast<size_t>(numSteps);

    if (args.size() != expected) {
      return OperationResult::Failure("Invalid argument count for AWG_BUFFER_RAMP");
    }

    int dacChannels[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < dacN; ++i) {
      dacChannels[i] = static_cast<int>(args[idx++]);
    }

    OperationResult dacValidation = validateDacChannels(dacChannels, dacN);
    if (!dacValidation.isSuccess()) {
      return dacValidation;
    }

    const float* voltages = &args[idx];
    return AWGDacOnlyRampBase(dacN, numSteps, dac_interval_us, dacChannels, voltages);
  }



  OperationResult God::AWGDacOnlyRampBase(
      int numDacChannels,
      int numSteps,
      uint32_t dac_interval_us,
      int* dacChannels,
      const float* channelMajorVoltages) {
    if (dac_interval_us < 1) {
      return OperationResult::Failure("Invalid dac interval");
    }
    if (!isValidDacChannelCount(numDacChannels) || numSteps < 1) {
      return OperationResult::Failure("Invalid number of channels or steps");
    }
    OperationResult dacValidation =
        validateDacChannels(dacChannels, numDacChannels);
    if (!dacValidation.isSuccess()) {
      return dacValidation;
    }

    // Bounds check before starting
    std::vector<uint8_t> dacPackets(
        static_cast<size_t>(numDacChannels) * static_cast<size_t>(numSteps) *
        3u);
    for (int i = 0; i < numDacChannels; i++) {
      int ch = dacChannels[i];
      float lowerBound = DACController::getLowerBound(ch);
      float upperBound = DACController::getUpperBound(ch);
      const float* vlist = &channelMajorVoltages[static_cast<size_t>(i) * static_cast<size_t>(numSteps)];
      for (int j = 0; j < numSteps; j++) {
        float v = vlist[j];
        if (v < lowerBound || v > upperBound) {
          return OperationResult::Failure("DAC " + String(ch) +
                                          " voltage[" + String(j) + "] = " + String(v, 6) +
                                          "V out of bounds [" + String(lowerBound, 6) +
                                          ", " + String(upperBound, 6) + "]");
        }
        uint8_t* packet =
            &dacPackets[(static_cast<size_t>(i) * static_cast<size_t>(numSteps) +
                         static_cast<size_t>(j)) *
                        3u];
        if (!DACController::encodeVoltagePacket(ch, v, packet)) {
          return dacWriteFailure(ch, v);
        }
      }
    }

    clearWorkerStopRequest();
    PeripheralCommsController::dataLedOn();

    // Apply initial step immediately; first LDAC edge will latch these values.
    for (int i = 0; i < numDacChannels; i++) {
      const uint8_t* packet =
          &dacPackets[static_cast<size_t>(i) * static_cast<size_t>(numSteps) *
                      3u];
      DACController::writeVoltagePacketNoLdac(dacChannels[i], packet);
    }

    TimingUtil::setupTimerOnlyDac(dac_interval_us);
    TimingUtil::dacFlag = false;

    // Run continuously (repeat waveform) until STOP is requested.
    int step = 1; // step 0 already written
    while (!isWorkerStopRequested()) {
      __WFE();
      if (TimingUtil::consumeDacFlag()) {
        for (int i = 0; i < numDacChannels; i++) {
          const uint8_t* packet =
              &dacPackets[(static_cast<size_t>(i) *
                               static_cast<size_t>(numSteps) +
                           static_cast<size_t>(step)) *
                          3u];
          DACController::writeVoltagePacketNoLdac(dacChannels[i], packet);
        }
        step++;
        if (step >= numSteps) {
          step = 0;
        }
      }
    }

    TimingUtil::disableDacInterrupt();
    TimingUtil::dacFlag = false;
    PeripheralCommsController::dataLedOff();

    if (isWorkerStopRequested()) {
      clearWorkerStopRequest();
      return OperationResult::Failure("RAMPING_STOPPED");
    }
    return OperationResult::Success();
  }



  // AWG_WITH_ADC: AWG waveform with ADC reading at each step
  // Format: AWG_WITH_ADC,dacN,adcN,numSteps,dac_interval_us,numCycles,dacChannels...,adcChannels...,voltages...
  // Voltages are channel-major: all points for DAC0, then all for DAC1, etc.
  OperationResult God::AWGWithADCWrapper(std::vector<float> args) {
    if (args.size() < 5) {
      return OperationResult::Failure("Insufficient arguments for AWG_WITH_ADC");
    }

    int idx = 0;
    const int dacN = static_cast<int>(args[idx++]);
    const int adcN = static_cast<int>(args[idx++]);
    const int numSteps = static_cast<int>(args[idx++]);
    const float dacIntervalArg = args[idx++];
    const int numCycles = static_cast<int>(args[idx++]);

    if (!isValidDacChannelCount(dacN) ||
        !isValidAdcChannelCount(adcN) ||
        numSteps < 1 || numCycles < 1 ||
        !isUint32AtLeast(dacIntervalArg, 1)) {
      return OperationResult::Failure("Invalid channel counts or step/cycle count");
    }
    const uint32_t dac_interval_us = static_cast<uint32_t>(dacIntervalArg);

    const size_t expected = 5u + dacN + adcN + (static_cast<size_t>(dacN) * numSteps);
    if (args.size() != expected) {
      return OperationResult::Failure("Invalid argument count for AWG_WITH_ADC");
    }

    int dacChannels[NUM_DAC_CHANNELS] = {};
    for (int i = 0; i < dacN; ++i) {
      dacChannels[i] = static_cast<int>(args[idx++]);
    }

    int adcChannels[NUM_ADC_CHANNELS] = {};
    for (int i = 0; i < adcN; ++i) {
      adcChannels[i] = static_cast<int>(args[idx++]);
    }

    OperationResult dacValidation = validateDacChannels(dacChannels, dacN);
    if (!dacValidation.isSuccess()) {
      return dacValidation;
    }
    OperationResult adcValidation = validateAdcChannels(adcChannels, adcN);
    if (!adcValidation.isSuccess()) {
      return adcValidation;
    }

    const float* voltages = &args[idx];
    return AWGWithADCBase(dacN, adcN, numSteps, dac_interval_us, numCycles, dacChannels, adcChannels, voltages);
  }



  OperationResult God::AWGWithADCBase(
      int numDacChannels, int numAdcChannels, int numSteps,
      uint32_t dac_interval_us, int numCycles,
      int* dacChannels, int* adcChannels, const float* channelMajorVoltages) {

    if (dac_interval_us < 1) {
      return OperationResult::Failure("Invalid dac interval");
    }
    if (!isValidDacChannelCount(numDacChannels) ||
        !isValidAdcChannelCount(numAdcChannels) ||
        numSteps < 1) {
      return OperationResult::Failure("Invalid number of channels or steps");
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

    // Bounds check DAC voltages
    for (int i = 0; i < numDacChannels; i++) {
      int ch = dacChannels[i];
      float lowerBound = DACController::getLowerBound(ch);
      float upperBound = DACController::getUpperBound(ch);
      const float* vlist = &channelMajorVoltages[static_cast<size_t>(i) * static_cast<size_t>(numSteps)];
      for (int j = 0; j < numSteps; j++) {
        float v = vlist[j];
        if (v < lowerBound || v > upperBound) {
          return OperationResult::Failure("DAC " + String(ch) +
                                          " voltage[" + String(j) + "] = " + String(v, 6) +
                                          "V out of bounds [" + String(lowerBound, 6) +
                                          ", " + String(upperBound, 6) + "]");
        }
      }
    }

      FastGpio::digitalWrite(adc_sync, false);

      BoardUsage boardUsage = getUsedBoards(adcChannels, numAdcChannels);
      attachAdcSyncInterrupts(boardUsage);

    clearWorkerStopRequest();
    PeripheralCommsController::dataLedOn();
    ADCController::resetToPreviousConversionTimes();

    double packets[NUM_ADC_CHANNELS] = {};
    bool voltageOverflow = false;

    // Start ADC continuous conversion
    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::startContinuousConversion(adcChannels[i]);
      ADCController::setRDYFN(adcChannels[i]);
    }

    // Apply initial step
    for (int i = 0; i < numDacChannels; i++) {
      const float v0 = channelMajorVoltages[static_cast<size_t>(i) * static_cast<size_t>(numSteps)];
      DACController::setVoltageNoTransactionNoLdac(dacChannels[i], v0);
    }

    // DAC timer only; ADC is triggered by data_ready interrupts.
    TimingUtil::setupTimerOnlyDac(dac_interval_us);
    TimingUtil::dacFlag = false;
    TimingUtil::adcFlag = 0;

    // Main loop: for each cycle, for each step, set DAC and read ADC
    for (int cycle = 0; cycle < numCycles && !isWorkerStopRequested(); cycle++) {
      int step = 0;
      while (step < numSteps && !isWorkerStopRequested()) {
        // Wait for timer flag
        __WFE();
        if (TimingUtil::consumeDacFlag()) {
          // Set DAC voltages for this step
          for (int i = 0; i < numDacChannels; i++) {
            const float v = channelMajorVoltages[static_cast<size_t>(i) * static_cast<size_t>(numSteps) +
                                                static_cast<size_t>(step)];
            DACController::setVoltageNoTransactionNoLdac(dacChannels[i], v);
          }
          FastGpio::digitalWrite(adc_sync, true);
          step++;
        }

        if (TimingUtil::consumeAnyAdcFlag()) {
          // Read ADC channels (data already converted, just read it)
          FastGpio::digitalWrite(adc_sync, false);
          for (int i = 0; i < numAdcChannels; i++) {
            packets[i] = ADCController::getVoltageData(adcChannels[i]);
          }

          // Send ADC data back
          if (!sendVoltageFrame(packets, numAdcChannels)) {
            voltageOverflow = true;
            break;
          }
        }
      }
    }

    TimingUtil::disableDacInterrupt();
    TimingUtil::dacFlag = false;
    PeripheralCommsController::dataLedOff();

    // Stop continuous conversion
    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
      ADCController::unsetRDYFN(adcChannels[i]);
    }

    detachAdcSyncInterrupts(boardUsage);

    ADCController::resetToPreviousConversionTimes();

    if (isWorkerStopRequested()) {
      clearWorkerStopRequest();
      if (voltageOverflow) {
        return OperationResult::Failure("Voltage output buffer overflow");
      }
      return OperationResult::Failure("RAMPING_STOPPED");
    }
    return finishRampTimingWatchdog();
  }




  OperationResult God::AWGBufferRampBase(
      int numDacChannels, int numAdcChannels, int numLoops, int numDacStepsPerLoop, int numAdcAverages,
      uint32_t dac_interval_us, int* dacChannels,
      float** dacVoltageLists, int* adcChannels) {
    if (dac_interval_us < 1) {
      return OperationResult::Failure("Invalid interval or settling time");
    }
    if (numAdcAverages < 1) {
      return OperationResult::Failure("Invalid number of ADC averages");
    }
    if (numLoops < 1 || numDacStepsPerLoop < 1) {
      return OperationResult::Failure("Invalid number of loops or steps per loop");
    }
    if (!isValidDacChannelCount(numDacChannels) ||
        !isValidAdcChannelCount(numAdcChannels)) {
      return OperationResult::Failure("Invalid number of channels");
    }

    // Check voltage bounds before executing ramp
    for (int i = 0; i < numDacChannels; i++) {
      int ch = dacChannels[i];
      float lowerBound = DACController::getLowerBound(ch);
      float upperBound = DACController::getUpperBound(ch);
      
      for (int j = 0; j < numDacStepsPerLoop; j++) {
        float voltage = dacVoltageLists[i][j];
        if (voltage < lowerBound || voltage > upperBound) {
          return OperationResult::Failure("DAC " + String(ch) + 
                                          " voltage[" + String(j) + "] = " + String(voltage, 6) + 
                                          "V out of bounds [" + String(lowerBound, 6) + 
                                          ", " + String(upperBound, 6) + "]");
        }
      }
    }
    
    double packets[NUM_ADC_CHANNELS] = {};
    double numAdcAveragesInv = 1.0 / static_cast<double>(numAdcAverages);

    clearWorkerStopRequest();
    PeripheralCommsController::dataLedOn();

    ADCController::resetToPreviousConversionTimes();

    FastGpio::digitalWrite(adc_sync, false);

    BoardUsage boardUsage = getUsedBoards(adcChannels, numAdcChannels);

    const float maxConvTime =
        maxAdcConversionTimePerBoard(adcChannels, numAdcChannels);
    uint32_t totalDacSweepTime = numDacStepsPerLoop * dac_interval_us;
    if (maxConvTime * numAdcAverages + 180 >= totalDacSweepTime) {
      PeripheralCommsController::dataLedOff();
      return OperationResult::Failure(
          "DAC sweep time is too short for specified ADC conversion time, "
          "please increase dac_interval_us or reduce numDacStepsPerLoop");
    }

    attachAdcSyncInterrupts(boardUsage);

    // Initialize timing flags
    TimingUtil::dacFlag = false;
    TimingUtil::adcFlag = 0;

    // Track current position in voltage lists and loop
    int currentLoop = 0;
    int totalDacSteps = numLoops * numDacStepsPerLoop;
    int currentDacStep = 0;
    int currentAdcReads = 0;
    bool voltageOverflow = false;

    // Set initial DAC voltages (first step of first loop)
    for (int i = 0; i < numDacChannels; i++) {
      DACController::setVoltageNoTransactionNoLdac(dacChannels[i], dacVoltageLists[i][0]);
    }
    currentDacStep++;

    // Start ADC continuous conversion
    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::startContinuousConversion(adcChannels[i]);
      ADCController::setRDYFN(adcChannels[i]);
    }

    // Setup timers for DAC and ADC events
    TimingUtil::setupTimerOnlyDac(dac_interval_us);
    TimingUtil::dacFlag = false;

    bool done = false;

    // Main event loop using interrupt-based timing
    while (currentLoop < numLoops && !isWorkerStopRequested()) {
      __WFE(); // Wait for event (interrupt)
      
      // Handle DAC flag - time to set next DAC voltage
      if (currentDacStep < totalDacSteps && TimingUtil::consumeDacFlag()) {
        for (int i = 0; i < numDacChannels; i++) {
          float voltage = dacVoltageLists[i][currentDacStep];
          DACController::setVoltageNoTransactionNoLdac(dacChannels[i], voltage);
        }
        

        currentDacStep++;

        // Check if we've completed a full sweep of voltages for this loop
        if (currentDacStep >= numDacStepsPerLoop) {
          currentDacStep = 0; // Reset to beginning of voltage list for next loop
          done = true; // Mark that we need to read ADC after settling
        }
        
      }
      
      // Handle ADC flag - time to read ADC after settling
      if (done) {
        done = false; // Reset done flag for next ADC read
        for (int i = 0; i < numAdcChannels; i++) {
          double total = 0.0;
          for (int j = 0; j < numAdcAverages; j++) {
            total += ADCController::getVoltage(adcChannels[i]);
          }
          packets[i] = total * numAdcAveragesInv;
        }
        if (!sendVoltageFrame(packets, numAdcChannels)) {
          voltageOverflow = true;
        }
        
        FastGpio::digitalWrite(adc_sync, false);
        currentAdcReads++;
        currentLoop++; // Each ADC read marks completion of one loop
      }
    }

    // Clean up timers
    TimingUtil::disableDacInterrupt();
    TimingUtil::disableAdcInterrupt();
    TimingUtil::dacFlag = false;
    TimingUtil::adcFlag = 0;

    // Clean up
    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
      ADCController::unsetRDYFN(adcChannels[i]);
    }

    detachAdcSyncInterrupts(boardUsage);

    ADCController::resetToPreviousConversionTimes();
    PeripheralCommsController::dataLedOff();

    if (isWorkerStopRequested()) {
      clearWorkerStopRequest();
      if (voltageOverflow) {
        return OperationResult::Failure("Voltage output buffer overflow");
      }
      return OperationResult::Failure("RAMPING_STOPPED");
    }

    return finishRampTimingWatchdog();
  }








  OperationResult God::dacChannelCalibration() {
    CalibrationData calibrationData;
    for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
      DACController::initialize();
      DACController::setCalibration(i, 0, 1);
      DACController::setVoltage(i, 0);
      delay(1);
      float offsetError = ADCController::getVoltage(i);
      DACController::setCalibration(i, offsetError, 1);
      float voltSet = 9.0;
      DACController::setVoltage(i, voltSet);
      delay(1);
      float gainError = (ADCController::getVoltage(i) - offsetError) / voltSet;
      DACController::setCalibration(i, offsetError, gainError);
      DACController::setVoltage(i, 0);
      calibrationData.offset[i] = offsetError;
      calibrationData.gain[i] = gainError;
    }
    updateCalibrationData(calibrationData);
    return OperationResult::Success("CALIBRATION_FINISHED");
  }


  OperationResult God::boxcarAverageRamp(const std::vector<float>& args) {
    if (args.size() < 7) {
      return OperationResult::Failure("Not enough arguments provided");
    }

    size_t currentIndex = 0;

    // Parse initial parameters
    int numDacChannels = static_cast<int>(args[currentIndex++]);
    int numAdcChannels = static_cast<int>(args[currentIndex++]);
    int numDacSteps = static_cast<int>(args[currentIndex++]);
    int numAdcMeasuresPerDacStep = static_cast<int>(args[currentIndex++]);
    int numAdcAverages = static_cast<int>(args[currentIndex++]);
    int numAdcConversionSkips = static_cast<int>(args[currentIndex++]);
    const float adcConversionTimeArg = args[currentIndex++];

    if (!isValidDacChannelCount(numDacChannels) ||
        !isValidAdcChannelCount(numAdcChannels)) {
      return OperationResult::Failure("Invalid number of channels");
    }
    if (numDacSteps < 1 || numAdcMeasuresPerDacStep < 1 ||
        numAdcAverages < 1 || numAdcConversionSkips < 0 ||
        !isUint32AtLeast(adcConversionTimeArg, 1)) {
      return OperationResult::Failure("Invalid boxcar timing/count argument");
    }
    uint32_t adcConversionTime_us =
        static_cast<uint32_t>(adcConversionTimeArg);

    const size_t expected =
        7u + static_cast<size_t>(numDacChannels) * 5u +
        static_cast<size_t>(numAdcChannels);
    if (args.size() != expected) {
      return OperationResult::Failure("Incorrect number of arguments");
    }

    int dacChannels[NUM_DAC_CHANNELS] = {};
    float dacV0_1[NUM_DAC_CHANNELS] = {};
    float dacVf_1[NUM_DAC_CHANNELS] = {};
    float dacV0_2[NUM_DAC_CHANNELS] = {};
    float dacVf_2[NUM_DAC_CHANNELS] = {};

    for (int i = 0; i < numDacChannels; ++i) {
      dacChannels[i] = static_cast<int>(args[currentIndex++]);
      dacV0_1[i] = args[currentIndex++];
      dacVf_1[i] = args[currentIndex++];
      dacV0_2[i] = args[currentIndex++];
      dacVf_2[i] = args[currentIndex++];
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

    uint32_t actualConversionTime_us = ADCController::presetConversionTime(
        adcChannels[0], adcConversionTime_us, numAdcChannels > 1);
    for (int i = 1; i < numAdcChannels; ++i) {
      ADCController::presetConversionTime(adcChannels[i], adcConversionTime_us,
                                          numAdcChannels > 1);
    }

    const uint64_t dacPeriod64 =
        static_cast<uint64_t>(numAdcMeasuresPerDacStep +
                              numAdcConversionSkips) *
        static_cast<uint64_t>(actualConversionTime_us + 5) *
        static_cast<uint64_t>(numAdcChannels) *
        static_cast<uint64_t>(numAdcAverages);
    if (dacPeriod64 == 0 || dacPeriod64 > 0xFFFFFFFFULL) {
      return OperationResult::Failure("Boxcar DAC period is out of range");
    }
    uint32_t dacPeriod_us = static_cast<uint32_t>(dacPeriod64);

    // Check voltage bounds before executing ramp (both calibrated bounds AND global limits)
    for (int i = 0; i < numDacChannels; i++) {
      int ch = dacChannels[i];
      float lowerBound = DACController::getLowerBound(ch);
      float upperBound = DACController::getUpperBound(ch);
      
      if (dacV0_1[i] < lowerBound || dacV0_1[i] > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " start voltage 1 " + String(dacV0_1[i], 6) + 
                                        "V out of bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
      if (dacVf_1[i] < lowerBound || dacVf_1[i] > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " end voltage 1 " + String(dacVf_1[i], 6) + 
                                        "V out of bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
      if (dacV0_2[i] < lowerBound || dacV0_2[i] > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " start voltage 2 " + String(dacV0_2[i], 6) + 
                                        "V out of bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
      if (dacVf_2[i] < lowerBound || dacVf_2[i] > upperBound) {
        return OperationResult::Failure("DAC " + String(ch) + 
                                        " end voltage 2 " + String(dacVf_2[i], 6) + 
                                        "V out of bounds [" + String(lowerBound, 6) + 
                                        ", " + String(upperBound, 6) + "]");
      }
    }

    clearWorkerStopRequest();
    PeripheralCommsController::dataLedOn();

    double voltageStepSizeLow[NUM_DAC_CHANNELS] = {};
    double voltageStepSizeHigh[NUM_DAC_CHANNELS] = {};

    for (int i = 0; i < numDacChannels; i++) {
      voltageStepSizeLow[i] =
          numDacSteps > 1 ? (dacVf_1[i] - dacV0_1[i]) /
                                static_cast<double>(numDacSteps - 1)
                          : 0.0;
      voltageStepSizeHigh[i] =
          numDacSteps > 1 ? (dacVf_2[i] - dacV0_2[i]) /
                                static_cast<double>(numDacSteps - 1)
                          : 0.0;
    }

    double previousVoltageSetLow[NUM_DAC_CHANNELS] = {};
    double previousVoltageSetHigh[NUM_DAC_CHANNELS] = {};

    for (int i = 0; i < numDacChannels; i++) {
      previousVoltageSetLow[i] = dacV0_1[i];
      previousVoltageSetHigh[i] = dacV0_2[i];
    }

    FastGpio::digitalWrite(adc_sync, false);

    BoardUsage boardUsage = getUsedBoards(adcChannels, numAdcChannels);
    attachAdcSyncInterrupts(boardUsage);

    uint8_t adcMask = 0u;
    adcMask = adcMaskForBoardUsage(boardUsage);

    int steps = 0;
    int totalSteps = 2 * numDacSteps * numAdcAverages;
    int x = 0;
    int total_data_size = totalSteps * numAdcMeasuresPerDacStep;
    int adcGetsSinceLastDacSet = 0;
    bool voltageOverflow = false;

    for (int i = 0; i < numAdcChannels; ++i) {
      ADCController::startContinuousConversion(adcChannels[i]);
      ADCController::setRDYFN(adcChannels[i]);
    }

    for (int i = 0; i < numDacChannels; i++) {
      double currentVoltage;
      if (steps % 2 == 0) {
        currentVoltage = previousVoltageSetLow[i];
      } else {
        currentVoltage = previousVoltageSetHigh[i];
      }

      DACController::setVoltageNoTransactionNoLdac(dacChannels[i],
                                                   currentVoltage);
    }

    DACController::toggleLdac();
    steps++;

    TimingUtil::setupTimersTimeSeries(dacPeriod_us, actualConversionTime_us,
                                      adcMask);

    while (x < total_data_size && !isWorkerStopRequested()) {
      if (TimingUtil::consumeAdcFlag(adcMask)) {
        if (adcGetsSinceLastDacSet >= numAdcConversionSkips) {
          double packets[NUM_ADC_CHANNELS] = {};
          for (int i = 0; i < numAdcChannels; i++) {
            packets[i] = ADCController::getVoltageDataNoTransaction(adcChannels[i]);
          }
          if (!sendVoltageFrame(packets, numAdcChannels)) {
            voltageOverflow = true;
            break;
          }
          x++;
        }
        adcGetsSinceLastDacSet++;
      }
      if (steps < totalSteps && TimingUtil::consumeDacFlag()) {
        for (int i = 0; i < numDacChannels; i++) {
          double currentVoltage;
          if (steps % (2 * numAdcAverages) != 0) {
            if (steps % 2 == 0) {
              currentVoltage = previousVoltageSetLow[i];
            } else {
              currentVoltage = previousVoltageSetHigh[i];
            }
          } else if (steps % 2 == 0) {
            previousVoltageSetLow[i] += voltageStepSizeLow[i];
            previousVoltageSetHigh[i] += voltageStepSizeHigh[i];
            currentVoltage = previousVoltageSetLow[i];
          } else {
            previousVoltageSetLow[i] += voltageStepSizeLow[i];
            previousVoltageSetHigh[i] += voltageStepSizeHigh[i];
            currentVoltage = previousVoltageSetHigh[i];
          }
          DACController::setVoltageNoTransactionNoLdac(dacChannels[i],
                                                       currentVoltage);
        }
        steps++;
        adcGetsSinceLastDacSet = 0;
        TIM8->CNT = 0;
      }
    }

    TimingUtil::disableDacInterrupt();
    TimingUtil::disableAdcInterrupt();

    for (int i = 0; i < numAdcChannels; i++) {
      ADCController::idleMode(adcChannels[i]);
      ADCController::unsetRDYFN(adcChannels[i]);
    }

    detachAdcSyncInterrupts(boardUsage);

    PeripheralCommsController::dataLedOff();

    if (isWorkerStopRequested()) {
      clearWorkerStopRequest();
      if (voltageOverflow) {
        return OperationResult::Failure("Voltage output buffer overflow");
      }
      return OperationResult::Failure("RAMPING_STOPPED");
    }

    return finishRampTimingWatchdog();
  }
