#include "Peripherals/BufferRamp.h"

#include "Config.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Peripherals/ADC/ADCController.h"
#include "Peripherals/BufferRampCommon.h"
#include "Peripherals/DAC/DACController.h"
#include "Peripherals/PeripheralCommsController.h"
#include "Peripherals/RampCommand.h"
#include "Utils/FastGpio.h"
#include "Utils/TimingUtil.h"
#include "Utils/shared_memory.h"

#include <algorithm>

namespace {
using BufferRampCommon::adcBoardForChannel;
using BufferRampCommon::dacWriteFailure;
using BufferRampCommon::encodeDacVoltagePackets;
using BufferRampCommon::finishRampTimingWatchdog;
using BufferRampCommon::isValidAdcChannelCount;
using BufferRampCommon::isValidDacChannelCount;
using BufferRampCommon::sendVoltageFrame;
using BufferRampCommon::validateDacChannels;
using BufferRampCommon::validateRampChannels;
using BufferRampCommon::writeDacPackets;
using RampCommand::validateDacVoltageListBounds;

using AdcIsr = void (*)();

AdcIsr kAdcSyncIsrFunctions[] = {
    TimingUtil::adcSyncISR<0>,
    TimingUtil::adcSyncISR<1>,
    TimingUtil::adcSyncISR<2>,
    TimingUtil::adcSyncISR<3>
};

void attachAdcSyncInterrupts(const BufferRamp::BoardUsage& boardUsage) {
  for (int i = 0; i < boardUsage.numBoards; i++) {
    const int pin = ADCController::getDataReadyPin(boardUsage.idx[i]);
    if (pin == NC) {
      continue;
    }
    attachInterrupt(digitalPinToInterrupt(pin), kAdcSyncIsrFunctions[i],
                    FALLING);
  }
}

void detachAdcSyncInterrupts(const BufferRamp::BoardUsage& boardUsage) {
  for (int i = 0; i < boardUsage.numBoards; i++) {
    const int pin = ADCController::getDataReadyPin(boardUsage.idx[i]);
    if (pin == NC) {
      continue;
    }
    detachInterrupt(digitalPinToInterrupt(pin));
  }
}

uint8_t adcMaskForBoardUsage(const BufferRamp::BoardUsage& boardUsage) {
  uint8_t adcMask = 0u;
  for (int i = 0; i < boardUsage.numBoards; i++) {
    adcMask |= 1 << i;
  }
  return adcMask;
}

constexpr int kTimeSeries2DRowStartAdcDiscards = 2;

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

bool readAdcPackets(int numAdcChannels, const int* adcChannels,
                    double* packets, int startIndex = 0) {
  for (int i = startIndex; i < numAdcChannels; i++) {
    packets[i] = ADCController::getVoltageDataNoTransaction(adcChannels[i]);
  }
  return true;
}

OperationResult validateLinearDacAdcRamp(
    int numDacChannels, int numAdcChannels, const int* dacChannels,
    const float* dacV0s, const float* dacVfs, const int* adcChannels) {
  OperationResult channelValidation = validateRampChannels(
      dacChannels, numDacChannels, adcChannels, numAdcChannels);
  if (!channelValidation.isSuccess()) {
    return channelValidation;
  }
  return RampCommand::validateDacEndpoints(numDacChannels, dacChannels,
                                           dacV0s, dacVfs);
}

}

void BufferRamp::setup() {
  initializeRegistry();
}



void BufferRamp::initializeRegistry() {
  registerFunction(initialize, "INITIALIZE");
  registerFunction(initialize, "INIT");
  registerFunction(initialize, "INNIT");

  registerFunction(timeSeriesBufferRampBase, "TIME_SERIES_BUFFER_RAMP");
  registerFunction(dacLedBufferRampBase, "DAC_LED_BUFFER_RAMP");
  registerFunction(AWGBufferRampWrapper, "AWG_BUFFER_RAMP");
  registerFunction(AWGWithADCWrapper, "AWG_WITH_ADC");
  registerFunction(timeSeriesAdcRead, "TIME_SERIES_ADC_READ");
  registerFunction(boxcarAverageRamp, "BOXCAR_BUFFER_RAMP");

  registerFunction(dacChannelCalibration, "DAC_CH_CAL");
  registerFunction(hardResetCalibrationToDefaults, "HARD_RESET_CALIBRATION");
}



OperationResult BufferRamp::initialize() {
  DACController::initialize();
  ADCController::initialize();
  return OperationResult::Success("INITIALIZATION COMPLETE");
}

OperationResult BufferRamp::hardResetCalibrationToDefaults() {
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



BufferRamp::BoardUsage BufferRamp::getUsedBoards(const int *adcChannels, int numAdcChannels) {
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



OperationResult BufferRamp::timeSeriesAdcRead(
    int numAdcChannels, List<int, 0>& adcChannelsList,
    float conversionTimeArg, float totalDurationArg) {
  if (!isValidAdcChannelCount(numAdcChannels)) {
    return OperationResult::Failure("Invalid number of ADC channels");
  }
  OperationResult adcValidation = BufferRampCommon::validateAdcChannels(
      adcChannelsList.data(), numAdcChannels);
  if (!adcValidation.isSuccess()) {
    return adcValidation;
  }
  if (!BufferRampCommon::isUint32AtLeast(conversionTimeArg, 1) ||
      !BufferRampCommon::isUint32AtLeast(totalDurationArg, 82)) {
    return OperationResult::Failure("Invalid total duration");
  }

  int* adcChannels = adcChannelsList.data();
  const uint32_t conversionTimeUs =
      static_cast<uint32_t>(conversionTimeArg);
  const uint32_t totalDurationUs = static_cast<uint32_t>(totalDurationArg);

  float realConversionTime = 0;
  for (int i = 0; i < numAdcChannels; i++) {
    ADCController::setConversionTime(adcChannels[i], conversionTimeUs);
  }
  realConversionTime = ADCController::getConversionTimeFloat(adcChannels[0]);

  const int maxIndependentAdcs =
      maxSelectedAdcChannelsPerBoard(adcChannels, numAdcChannels);

  const double samplePeriodUsFloat =
      maxIndependentAdcs * realConversionTime * 1.5f;
  const int samplePeriodUs = static_cast<int>(samplePeriodUsFloat);

  const int savedDataSize = totalDurationUs / samplePeriodUs;

  if (!sendVoltageFrame(&samplePeriodUsFloat, 1)) {
    clearWorkerStopRequest();
    return OperationResult::Failure("Voltage output buffer overflow");
  }

  clearWorkerStopRequest();
  PeripheralCommsController::dataLedOn();

  FastGpio::digitalWrite(adc_sync, false);
  BoardUsage boardUsage = getUsedBoards(adcChannels, numAdcChannels);
  attachAdcSyncInterrupts(boardUsage);

  ADCController::resetToPreviousConversionTimes();

  for (int i = 0; i < numAdcChannels; i++) {
    ADCController::startContinuousConversion(adcChannels[i]);
    ADCController::setRDYFN(adcChannels[i]);
  }

  uint8_t adcMask = adcMaskForBoardUsage(boardUsage);

  TimingUtil::setupTimersOnlyADC(samplePeriodUs);

  int samplesCaptured = 0;
  bool voltageOverflow = false;

  while (samplesCaptured < savedDataSize && !isWorkerStopRequested()) {
    __WFE();
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
      samplesCaptured++;
    }
  }

  TimingUtil::disableAdcInterrupt();

  for (int i = 0; i < numAdcChannels; i++) {
    ADCController::idleMode(adcChannels[i]);
    ADCController::unsetRDYFN(adcChannels[i]);
  }

  ADCController::resetToPreviousConversionTimes();

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
// dacChannels..., dacV0s..., dacVfs..., adcChannels...
OperationResult BufferRamp::timeSeriesBufferRampBase(
    int numDacChannels, int numAdcChannels, int numSteps,
    float dacIntervalArg, float adcIntervalArg,
    List<int, 0>& dacChannelsList,
    List<float, 0>& dacV0sList,
    List<float, 0>& dacVfsList,
    List<int, 1>& adcChannelsList) {
  if (!isValidDacChannelCount(numDacChannels) ||
      !isValidAdcChannelCount(numAdcChannels)) {
    return OperationResult::Failure("Invalid number of channels");
  }
  if (!BufferRampCommon::isUint32AtLeast(adcIntervalArg, 1) ||
      !BufferRampCommon::isUint32AtLeast(dacIntervalArg, 1)) {
    return OperationResult::Failure("Invalid interval");
  }
  if (numSteps < 1) {
    return OperationResult::Failure("Invalid number of steps");
  }

  int* dacChannels = dacChannelsList.data();
  float* dacV0s = dacV0sList.data();
  float* dacVfs = dacVfsList.data();
  int* adcChannels = adcChannelsList.data();

  OperationResult requestValidation = validateLinearDacAdcRamp(
      numDacChannels, numAdcChannels, dacChannels, dacV0s, dacVfs,
      adcChannels);
  if (!requestValidation.isSuccess()) {
    return requestValidation;
  }

  const uint32_t dacIntervalUs = static_cast<uint32_t>(dacIntervalArg);
  const uint32_t adcIntervalUs = static_cast<uint32_t>(adcIntervalArg);

  uint8_t adcMask = 0u;
  BoardUsage boardUsage{0, std::vector<uint8_t>()};
  clearWorkerStopRequest();
  PeripheralCommsController::dataLedOn();

  OperationResult prepareResult = prepareTimeSeriesBufferRampHardware(
      numAdcChannels, adcChannels, adcMask, boardUsage);
  if (!prepareResult.isSuccess()) {
    PeripheralCommsController::dataLedOff();
    return prepareResult;
  }

  OperationResult rampResult = runPreparedTimeSeriesBufferRamp(
      numDacChannels, numAdcChannels, numSteps, dacIntervalUs,
      adcIntervalUs, dacChannels, dacV0s, dacVfs, adcChannels, adcMask);

  cleanupTimeSeriesBufferRampHardware(numAdcChannels, adcChannels,
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
    return OperationResult::Failure("RAMPING_STOPPED");
  }

  return finishRampTimingWatchdog(false);
}



OperationResult BufferRamp::prepareTimeSeriesBufferRampHardware(
    int numAdcChannels, int* adcChannels, uint8_t& adcMask,
    BoardUsage& boardUsage) {
  adcMask = 0u;
  boardUsage = BoardUsage{0, std::vector<uint8_t>()};

  FastGpio::digitalWrite(adc_sync, false);

  boardUsage = getUsedBoards(adcChannels, numAdcChannels);
  attachAdcSyncInterrupts(boardUsage);
  adcMask = adcMaskForBoardUsage(boardUsage);

  ADCController::resetToPreviousConversionTimes();
  for (int i = 0; i < numAdcChannels; i++) {
    ADCController::startContinuousConversion(adcChannels[i]);
    ADCController::setRDYFN(adcChannels[i]);
  }

  TimingUtil::dacFlag = false;
  TimingUtil::adcFlag = 0;

  return OperationResult::Success();
}



OperationResult BufferRamp::runPreparedTimeSeriesBufferRamp(
    int numDacChannels, int numAdcChannels, int numSteps,
    uint32_t dac_interval_us, uint32_t adc_interval_us, int* dacChannels,
    float* dacV0s, float* dacVfs, int* adcChannels, uint8_t adcMask,
    TimeSeriesRampMode mode) {
  const bool buffered2DRow = mode == TimeSeriesRampMode::Buffered2DRow;
  int dacStepsLoaded = 0;
  int framesCaptured = 0;
  const uint64_t savedDataSize64 =
      (static_cast<uint64_t>(numSteps) * dac_interval_us) / adc_interval_us;
  if (savedDataSize64 == 0 || savedDataSize64 > 2147483647ULL) {
    return OperationResult::Failure("Invalid time-series sample count");
  }
  const int savedDataSize = static_cast<int>(savedDataSize64);
  int discardedAdcSamples = 0;
  const int discardCount =
      buffered2DRow ? kTimeSeries2DRowStartAdcDiscards : 0;
  bool voltageOverflow = false;
  std::vector<double> bufferedFrames;
  if (buffered2DRow) {
    bufferedFrames.resize(static_cast<size_t>(savedDataSize) *
                          static_cast<size_t>(numAdcChannels));
  }

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
    if (dacStepsLoaded >= numSteps) {
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
  dacStepsLoaded++;
  if (!prepareNextDacPackets()) {
    return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
  }
  FastGpio::digitalWrite(adc_sync, false);
  for (int i = 0; i < numAdcChannels; i++) {
    ADCController::startContinuousConversion(adcChannels[i]);
    ADCController::setRDYFN(adcChannels[i]);
  }
  TimingUtil::setupTimersTimeSeriesRamp(dac_interval_us, adc_interval_us,
                                        adcMask);
  TimingUtil::dacFlag = false;
  TimingUtil::adcFlag = 0;
  bool dacTimerPending = false;

  while ((discardedAdcSamples < discardCount ||
          framesCaptured < savedDataSize || dacStepsLoaded < numSteps) &&
         !isWorkerStopRequested()) {
    __WFE();
    const bool adcPending =
        (discardedAdcSamples < discardCount ||
         framesCaptured < savedDataSize) &&
        TimingUtil::consumeAdcFlag(adcMask);
    const bool holdingInitialDac = discardedAdcSamples < discardCount;
    if ((holdingInitialDac || dacStepsLoaded < numSteps) &&
        TimingUtil::consumeDacFlag()) {
      dacTimerPending = true;
    }

    double packets[NUM_ADC_CHANNELS] = {};
    bool haveAdcPackets = false;
    if (adcPending) {
      readAdcPackets(numAdcChannels, adcChannels, packets);
      FastGpio::digitalWrite(adc_sync, false);
      if (discardedAdcSamples < discardCount) {
        discardedAdcSamples++;
      } else {
        haveAdcPackets = true;
      }
    }
    if (dacTimerPending && TimingUtil::adcConversionInProgressMask == 0) {
      if (holdingInitialDac) {
        dacTimerPending = false;
      } else if (!nextDacPacketsReady ||
                 !writeDacPackets(numDacChannels, dacChannels,
                                  nextDacPackets)) {
        TimingUtil::stopTimeSeriesTimers();
        return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
      } else {
        dacTimerPending = false;
        for (int i = 0; i < numDacChannels; i++) {
          nextVoltageSet[i] += voltageStepSize[i];
        }
        dacStepsLoaded++;
        if (!prepareNextDacPackets()) {
          TimingUtil::stopTimeSeriesTimers();
          return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
        }
      }
    }
    if (haveAdcPackets) {
      if (buffered2DRow) {
        const size_t frameOffset =
            static_cast<size_t>(framesCaptured) *
            static_cast<size_t>(numAdcChannels);
        for (int i = 0; i < numAdcChannels; i++) {
          bufferedFrames[frameOffset + static_cast<size_t>(i)] = packets[i];
        }
      } else if (!sendVoltageFrame(packets, numAdcChannels)) {
        voltageOverflow = true;
        break;
      }
      framesCaptured++;
    }
  }

  TimingUtil::stopTimeSeriesTimers();

  if (isWorkerStopRequested()) {
    if (voltageOverflow) {
      return OperationResult::Failure("Voltage output buffer overflow");
    }
    return OperationResult::Failure("RAMPING_STOPPED");
  }
  if (voltageOverflow) {
    return OperationResult::Failure("Voltage output buffer overflow");
  }

  if (buffered2DRow) {
    for (int frame = 0; frame < savedDataSize && !isWorkerStopRequested();
         frame++) {
      const size_t frameOffset = static_cast<size_t>(frame) *
                                 static_cast<size_t>(numAdcChannels);
      if (!sendVoltageFrame(&bufferedFrames[frameOffset], numAdcChannels)) {
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



void BufferRamp::cleanupTimeSeriesBufferRampHardware(
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
// dacSettlingTime_us, dacChannels..., dacV0s..., dacVfs..., adcChannels...
OperationResult BufferRamp::dacLedBufferRampBase(
    int numDacChannels, int numAdcChannels, int numSteps,
    int numAdcAverages, float dacIntervalArg, float dacSettlingTimeArg,
    List<int, 0>& dacChannelsList,
    List<float, 0>& dacV0sList,
    List<float, 0>& dacVfsList,
    List<int, 1>& adcChannelsList) {
  if (!isValidDacChannelCount(numDacChannels) ||
      !isValidAdcChannelCount(numAdcChannels)) {
    return OperationResult::Failure("Invalid number of channels");
  }
  if (!BufferRampCommon::isUint32AtLeast(dacSettlingTimeArg, 1) ||
      !BufferRampCommon::isUint32AtLeast(dacIntervalArg, 1) ||
      dacSettlingTimeArg >= dacIntervalArg) {
    return OperationResult::Failure("Invalid interval or settling time");
  }
  if (numAdcAverages < 1) {
    return OperationResult::Failure("Invalid number of ADC averages");
  }
  if (numSteps < 1) {
    return OperationResult::Failure("Invalid number of steps");
  }

  int* dacChannels = dacChannelsList.data();
  float* dacV0s = dacV0sList.data();
  float* dacVfs = dacVfsList.data();
  int* adcChannels = adcChannelsList.data();

  OperationResult requestValidation = validateLinearDacAdcRamp(
      numDacChannels, numAdcChannels, dacChannels, dacV0s, dacVfs,
      adcChannels);
  if (!requestValidation.isSuccess()) {
    return requestValidation;
  }

  const uint32_t dacIntervalUs = static_cast<uint32_t>(dacIntervalArg);
  const uint32_t dacSettlingTimeUs =
      static_cast<uint32_t>(dacSettlingTimeArg);

  uint8_t adcMask = 0u;
  BoardUsage boardUsage{0, std::vector<uint8_t>()};
  clearWorkerStopRequest();
  PeripheralCommsController::dataLedOn();

  OperationResult prepareResult = prepareDacLedBufferRampHardware(
      numAdcChannels, adcChannels, adcMask, boardUsage);
  if (!prepareResult.isSuccess()) {
    PeripheralCommsController::dataLedOff();
    return prepareResult;
  }

  TimingUtil::setupTimersDacLed(dacIntervalUs, dacSettlingTimeUs, adcMask);
  TimingUtil::dacFlag = false;
  TimingUtil::adcFlag = 0;

  OperationResult rampResult = runPreparedDacLedBufferRamp(
      numDacChannels, numAdcChannels, numSteps, numAdcAverages,
      dacChannels, dacV0s, dacVfs, adcChannels, adcMask);

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



OperationResult BufferRamp::prepareDacLedBufferRampHardware(
    int numAdcChannels, int* adcChannels, uint8_t& adcMask,
    BoardUsage& boardUsage) {
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

  return OperationResult::Success();
}



OperationResult BufferRamp::runPreparedDacLedBufferRamp(
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
  int dacStepsLoaded = 0;
  double nextVoltageSet[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < numDacChannels; i++) {
    nextVoltageSet[i] = dacV0s[i];
  }
  byte nextDacPackets[NUM_DAC_CHANNELS][3] = {};
  bool nextDacPacketsReady = false;
  auto prepareNextDacPackets = [&]() {
    if (dacStepsLoaded >= numSteps) {
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
  dacStepsLoaded++;
  if (!prepareNextDacPackets()) {
    return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
  }
  FastGpio::digitalWrite(adc_sync, false);
  int adcFramesRead = 0;
  bool dacTimerPending = false;
  bool voltageOverflow = false;

  while (adcFramesRead < numSteps && !isWorkerStopRequested()) {
    __WFE();

    if (dacStepsLoaded < numSteps && TimingUtil::consumeDacFlag()) {
      dacTimerPending = true;
    }
    const bool adcConversionStarted =
        TimingUtil::consumeAdcConversionStartedFlag();
    const bool adcPending = TimingUtil::consumeAdcFlag(adcMask);

    bool haveAdcPackets = false;
    if (adcPending) {
      adcFramesRead++;
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

    if (dacTimerPending && adcConversionStarted) {
      if (!nextDacPacketsReady ||
          !writeDacPackets(numDacChannels, dacChannels, nextDacPackets)) {
        return dacWriteFailure(dacChannels[0], nextVoltageSet[0]);
      }
      dacTimerPending = false;
      for (int i = 0; i < numDacChannels; i++) {
        nextVoltageSet[i] += voltageStepSize[i];
      }
      dacStepsLoaded++;
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



void BufferRamp::cleanupDacLedBufferRampHardware(
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



OperationResult BufferRamp::OwenRampWrapper(
    int numDacChannels, int numAdcChannels, int numLoops,
    int numDacStepsPerLoop, int numAdcAverages, float dacIntervalArg,
    List<int, 0>& dacChannelsList,
    List<int, 1>& adcChannelsList,
    List<float, 0, 3>& dacVoltageStorage,
    int specialIndex, int specialWidth, int numStepsPerSpecialRamp,
    List<float, 0>& specialDacV0s,
    List<float, 0>& specialDacVfs) {
  // Expected argument order:
  // [numDacChannels, numAdcChannels, numLoops, numDacStepsPerLoop, numAdcAverages, dac_interval_us, <dacChannels...>, <adcChannels...>, <dacVoltageLists...>, specialIndex, specialWidth, numStepsPerSpecialRamp, <specialDacV0s...>, <specialDacVfs...>]
  // The number of DAC and ADC channels determines how many channel indices and voltage lists to expect.
  if (!isValidDacChannelCount(numDacChannels) ||
      !isValidAdcChannelCount(numAdcChannels) ||
      numLoops < 1 || numDacStepsPerLoop < 1 || numAdcAverages < 1 ||
      !BufferRampCommon::isUint32AtLeast(dacIntervalArg, 1)) {
    return OperationResult::Failure(
        "Invalid channel or loop/step/average count");
  }
  if (specialIndex < 0 || specialIndex >= numDacStepsPerLoop ||
      specialWidth < 0 || numStepsPerSpecialRamp < 1) {
    return OperationResult::Failure("Invalid Owen special ramp parameters");
  }

  float* dacVoltageLists[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < numDacChannels; ++i) {
    dacVoltageLists[i] =
        &dacVoltageStorage[static_cast<size_t>(i) *
                           static_cast<size_t>(numDacStepsPerLoop)];
  }

  return OwenRampBase(numDacChannels, numAdcChannels, numLoops,
                      numDacStepsPerLoop, numAdcAverages,
                      static_cast<uint32_t>(dacIntervalArg),
                      dacChannelsList.data(), dacVoltageLists,
                      adcChannelsList.data(), specialIndex,
                      numStepsPerSpecialRamp, specialDacV0s.data(),
                      specialDacVfs.data());
}



OperationResult BufferRamp::OwenRampBase(
  int numDacChannels, int numAdcChannels, int numLoops, int numDacStepsPerLoop, int numAdcAverages,
  uint32_t dac_interval_us, int* dacChannels,
  float** dacVoltageLists, int* adcChannels, int specialIndex,
  int numStepsPerSpecialRamp, float* specialDacV0s, float* specialDacVfs) {

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

    OperationResult channelValidation = validateRampChannels(
        dacChannels, numDacChannels, adcChannels, numAdcChannels);
    if (!channelValidation.isSuccess()) {
      return channelValidation;
    }

    OperationResult waveformBounds = validateDacVoltageListBounds(
        numDacChannels, numDacStepsPerLoop, dacChannels, dacVoltageLists);
    if (!waveformBounds.isSuccess()) {
      return waveformBounds;
    }
    for (int i = 0; i < numDacChannels; i++) {
      int ch = dacChannels[i];
      float lowerBound = DACController::getLowerBound(ch);
      float upperBound = DACController::getUpperBound(ch);
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




OperationResult BufferRamp::AWGBufferRampWrapper(
    int numDacChannels, int numSteps, float dacIntervalArg,
    List<int, 0>& dacChannels,
    List<float, 0, 1>& channelMajorVoltages) {
  //   AWG_BUFFER_RAMP,<dacN>,<numSteps>,<dacInterval_us>,<dacPorts...>,<voltages...>
  //
  // Voltages are channel-major: all points for DAC0, then all points for DAC1, ...

  if (!BufferRampCommon::isUint32AtLeast(dacIntervalArg, 1)) {
    return OperationResult::Failure("Invalid number of channels or steps");
  }

  return AWGDacOnlyRampBase(numDacChannels, numSteps,
                            static_cast<uint32_t>(dacIntervalArg),
                            dacChannels.data(),
                            channelMajorVoltages.data());
}



OperationResult BufferRamp::AWGDacOnlyRampBase(
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
  OperationResult waveformBounds = validateDacVoltageListBounds(
      numDacChannels, numSteps, dacChannels, channelMajorVoltages);
  if (!waveformBounds.isSuccess()) {
    return waveformBounds;
  }
  for (int i = 0; i < numDacChannels; i++) {
    int ch = dacChannels[i];
    const float* vlist = &channelMajorVoltages[static_cast<size_t>(i) *
                                               static_cast<size_t>(numSteps)];
    for (int j = 0; j < numSteps; j++) {
      float v = vlist[j];
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
OperationResult BufferRamp::AWGWithADCWrapper(
    int numDacChannels, int numAdcChannels, int numSteps,
    float dacIntervalArg, int numCycles,
    List<int, 0>& dacChannels,
    List<int, 1>& adcChannels,
    List<float, 0, 2>& channelMajorVoltages) {
  if (!BufferRampCommon::isUint32AtLeast(dacIntervalArg, 1) ||
      numCycles < 1) {
    return OperationResult::Failure(
        "Invalid channel counts or step/cycle count");
  }
  return AWGWithADCBase(numDacChannels, numAdcChannels, numSteps,
                        static_cast<uint32_t>(dacIntervalArg), numCycles,
                        dacChannels.data(), adcChannels.data(),
                        channelMajorVoltages.data());
}



OperationResult BufferRamp::AWGWithADCBase(
    int numDacChannels, int numAdcChannels, int numSteps,
    uint32_t dac_interval_us, int numCycles,
    int* dacChannels, int* adcChannels, const float* channelMajorVoltages) {

  if (dac_interval_us < 1) {
    return OperationResult::Failure("Invalid dac interval");
  }
  if (!isValidDacChannelCount(numDacChannels) ||
      !isValidAdcChannelCount(numAdcChannels) || numSteps < 1 ||
      numCycles < 1) {
    return OperationResult::Failure(
        "Invalid channel counts or step/cycle count");
  }
  OperationResult channelValidation = validateRampChannels(
      dacChannels, numDacChannels, adcChannels, numAdcChannels);
  if (!channelValidation.isSuccess()) {
    return channelValidation;
  }

  OperationResult waveformBounds = validateDacVoltageListBounds(
      numDacChannels, numSteps, dacChannels, channelMajorVoltages);
  if (!waveformBounds.isSuccess()) {
    return waveformBounds;
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




OperationResult BufferRamp::AWGBufferRampBase(
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

  OperationResult waveformBounds = validateDacVoltageListBounds(
      numDacChannels, numDacStepsPerLoop, dacChannels, dacVoltageLists);
  if (!waveformBounds.isSuccess()) {
    return waveformBounds;
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








OperationResult BufferRamp::dacChannelCalibration() {
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


OperationResult BufferRamp::boxcarAverageRamp(
    int numDacChannels, int numAdcChannels, int numDacSteps,
    int numAdcMeasuresPerDacStep, int numAdcAverages,
    int numAdcConversionSkips, float adcConversionTimeArg,
    List<int, 0>& dacChannelsList,
    List<float, 0>& dacV0_1List,
    List<float, 0>& dacVf_1List,
    List<float, 0>& dacV0_2List,
    List<float, 0>& dacVf_2List,
    List<int, 1>& adcChannelsList) {
  if (!isValidDacChannelCount(numDacChannels) ||
      !isValidAdcChannelCount(numAdcChannels)) {
    return OperationResult::Failure("Invalid number of channels");
  }
  if (numDacSteps < 1 || numAdcMeasuresPerDacStep < 1 ||
      numAdcAverages < 1 || numAdcConversionSkips < 0 ||
      !BufferRampCommon::isUint32AtLeast(adcConversionTimeArg, 1)) {
    return OperationResult::Failure("Invalid boxcar timing/count argument");
  }

  int* dacChannels = dacChannelsList.data();
  float* dacV0_1 = dacV0_1List.data();
  float* dacVf_1 = dacVf_1List.data();
  float* dacV0_2 = dacV0_2List.data();
  float* dacVf_2 = dacVf_2List.data();
  int* adcChannels = adcChannelsList.data();

  OperationResult channelValidation = validateRampChannels(
      dacChannels, numDacChannels, adcChannels, numAdcChannels);
  if (!channelValidation.isSuccess()) {
    return channelValidation;
  }
  OperationResult endpointValidation = RampCommand::validateBoxcarDacEndpoints(
      numDacChannels, dacChannels, dacV0_1, dacVf_1, dacV0_2, dacVf_2);
  if (!endpointValidation.isSuccess()) {
    return endpointValidation;
  }

  const uint32_t adcConversionTime_us =
      static_cast<uint32_t>(adcConversionTimeArg);

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
