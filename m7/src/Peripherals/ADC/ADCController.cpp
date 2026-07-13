#include "Peripherals/ADC/ADCController.h"

#include <array>
#include <cmath>
#include <utility>

#include "Calibration/Calibration.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "PeripheralCommsController.h"
#include "Utils/FastGpio.h"
#include "shared_memory.h"

namespace {

constexpr uint8_t kSyncEnabledIoRegister = 0b00010001;
constexpr uint8_t kReadyFunctionIoRegister = 0b00011001;
constexpr uint32_t kMaxCalibrationValue = 0xFFFFFFu;

int boardForChannel(int ch) { return ch / NUM_CHANNELS_PER_ADC_BOARD; }
int localChannel(int ch) { return ch % NUM_CHANNELS_PER_ADC_BOARD; }

template <size_t... Indices>
std::array<PeripheralCommsController, sizeof...(Indices)> makeAdcComms(
    std::index_sequence<Indices...>) {
  return {{PeripheralCommsController(adc_cs_pins[Indices])...}};
}

template <size_t... Indices>
std::array<bool, sizeof...(Indices)> makeChopEnabled(
    std::index_sequence<Indices...>) {
  return {{((void)Indices, true)...}};
}

auto comms = makeAdcComms(std::make_index_sequence<NUM_ADC_BOARDS>{});
auto chopEnabled =
    makeChopEnabled(std::make_index_sequence<NUM_ADC_BOARDS>{});

struct AdcCalibrationData {
  bool calibrated;
  uint32_t offset[NUM_ADC_CALIBRATION_CHANNELS];
  uint32_t gain[NUM_ADC_CALIBRATION_CHANNELS];
};

const uint32_t kAdcCalibrationId = CalibrationRegistry::sectionId("ADC");

AdcCalibrationData* adcCalibration(CalibrationData& data) {
  return static_cast<AdcCalibrationData*>(CalibrationRegistry::getSection(
      data, kAdcCalibrationId, sizeof(AdcCalibrationData)));
}

void setAdcCalibrationDefaults(void* section) {
  auto& data = *static_cast<AdcCalibrationData*>(section);
  data.calibrated = false;
  for (int i = 0; i < NUM_ADC_CALIBRATION_CHANNELS; i++) {
    data.offset[i] = 0x800000u;
    data.gain[i] = 0x200000u;
  }
}

bool validateAdcCalibration(const void* section) {
  const auto& data = *static_cast<const AdcCalibrationData*>(section);
  for (int i = 0; i < NUM_ADC_CALIBRATION_CHANNELS; i++) {
    if (data.offset[i] > kMaxCalibrationValue || data.gain[i] == 0 ||
        data.gain[i] > kMaxCalibrationValue) {
      return false;
    }
  }
  return true;
}
CALIBRATION_SECTION("ADC", AdcCalibrationData, setAdcCalibrationDefaults,
                    validateAdcCalibration)

uint8_t readRegister8(int board, uint8_t address, bool* success = nullptr) {
  byte data[2] = {static_cast<byte>(AdcRegister::kRead | address), 0};
  const bool transferred = comms[board].transferADC(data, 2);
  if (success) *success = transferred;
  return data[1];
}

uint32_t readRegister24(int board, uint8_t address, bool* success = nullptr) {
  byte data[4] = {static_cast<byte>(AdcRegister::kRead | address), 0, 0, 0};
  const bool transferred = comms[board].transferADC(data, 4);
  if (success) *success = transferred;
  return (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) |
         static_cast<uint32_t>(data[3]);
}

bool writeRegister8(int board, uint8_t address, uint8_t value) {
  byte data[2] = {static_cast<byte>(AdcRegister::kWrite | address), value};
  return comms[board].transferADC(data, 2);
}

bool writeRegister24(int board, uint8_t address, uint32_t value) {
  byte data[4] = {static_cast<byte>(AdcRegister::kWrite | address),
                  static_cast<byte>((value >> 16) & 0xFF),
                  static_cast<byte>((value >> 8) & 0xFF),
                  static_cast<byte>(value & 0xFF)};
  return comms[board].transferADC(data, 4);
}

bool waitDataReady(int board) {
  int count = 0;
  while (digitalRead(drdy[board]) == HIGH && count < 20000) {
    count++;
    delay(1);
  }
  FastGpio::digitalWrite(adc_sync, false);
  return count < 20000;
}

bool boardIdleMode(int board, int local_ch) {
  return writeRegister8(board, AdcRegister::mode(local_ch),
                        AdcRegister::kIdleMode);
}

bool boardReset(int board) {
  FastGpio::digitalWrite(reset[board], true);
  FastGpio::digitalWrite(reset[board], false);
  delay(5);
  FastGpio::digitalWrite(reset[board], true);
  bool success = true;
  for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
    if (!boardIdleMode(board, i)) success = false;
  }
  if (!writeRegister8(board, AdcRegister::kIo, kSyncEnabledIoRegister))
    success = false;
  if (!isCalibrationDataReady()) return success;
  CalibrationData data;
  readCalibrationData(data);
  const AdcCalibrationData* adcData = adcCalibration(data);
  if (!adcData || !adcData->calibrated) return success;
  for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
    int global = board * NUM_CHANNELS_PER_ADC_BOARD + i;
    if (!writeRegister24(board, AdcRegister::channelZeroScaleCal(i),
                         adcData->offset[global]))
      success = false;
    if (adcData->gain[global] != 0)
      if (!writeRegister24(board, AdcRegister::channelFullScaleCal(i),
                           adcData->gain[global]))
        success = false;
  }
  return success;
}

float calculateConversionTime(byte b, bool moreThanOneChannelActive) {
  byte fw = b & 0b01111111;
  bool chop = b & 0b10000000;
  if (chop) {
    return moreThanOneChannelActive ? (fw * 128.0f + 249.0f) / 6.144f
                                   : (fw * 128.0f + 248.0f) / 6.144f;
  } else {
    return moreThanOneChannelActive ? (fw * 64.0f + 206.0f) / 6.144f
                                   : (fw * 64.0f + 207.0f) / 6.144f;
  }
}

byte calculateFilterWord(float time_us, bool chop,
                         bool moreThanOneChannelActive) {
  double raw;
  int minimumFW;
  if (chop) {
    raw = moreThanOneChannelActive
              ? round((time_us * 6.144 - 249.0) / 128.0)
              : round((time_us * 6.144 - 248.0) / 128.0);
    minimumFW = 2;
  } else {
    raw = moreThanOneChannelActive
              ? round((time_us * 6.144 - 206.0) / 64.0)
              : round((time_us * 6.144 - 207.0) / 64.0);
    minimumFW = 3;
  }
  int out = static_cast<int>(raw);
  if (out < minimumFW) return static_cast<byte>(minimumFW);
  if (out > 127) return 127;
  return static_cast<byte>(out);
}

bool boardIsMoreThanOneChannelActive(int board, bool* success = nullptr) {
  bool transferred = false;
  const uint8_t status =
      readRegister8(board, AdcRegister::kAdcStatus, &transferred) &
      ((1 << NUM_CHANNELS_PER_ADC_BOARD) - 1);
  if (success) *success = transferred;
  return (status & (status - 1)) != 0;
}

float boardGetConversionTime(int board, int local_ch,
                             bool moreThanOneChannelActive,
                             bool* success = nullptr) {
  bool transferred = false;
  const uint8_t value = readRegister8(
      board, AdcRegister::channelConversionTime(local_ch), &transferred);
  if (success) *success = transferred;
  return transferred ? calculateConversionTime(value, moreThanOneChannelActive)
                     : -1.0f;
}

float boardSetConversionTime(int board, int local_ch, bool chop, byte fw,
                             bool moreThanOneChannelActive) {
  if ((fw > 127) || (chop && fw < 2) || (!chop && fw < 3)) return -1;
  byte send = (chop ? 0x80 : 0x00) | fw;
  if (!writeRegister8(board, AdcRegister::channelConversionTime(local_ch),
                      send)) {
    return -1;
  }
  bool readSucceeded = false;
  float t = boardGetConversionTime(board, local_ch, moreThanOneChannelActive,
                                   &readSucceeded);
  if (!readSucceeded) return -1;
  delayMicroseconds(100);
  return t;
}

float boardSetConversionTimeFloat(int board, int local_ch, float time_us,
                                  bool moreThanOneChannelActive) {
  return boardSetConversionTime(
      board, local_ch, chopEnabled[board],
      calculateFilterWord(time_us, chopEnabled[board],
                          moreThanOneChannelActive),
      moreThanOneChannelActive);
}

// --- command handlers (registered via function registry) ---

OperationResult setConversionTimeFW(int adc_channel, int filter_word) {
  if (!ADCController::isChannelIndexValid(adc_channel))
    return OperationResult::Failure("Invalid channel index");
  int b = boardForChannel(adc_channel);
  float sp = boardSetConversionTime(b, localChannel(adc_channel),
                                    chopEnabled[b], filter_word,
                                    boardIsMoreThanOneChannelActive(b));
  if (sp == -1.0)
    return OperationResult::Failure(
        "The filter word you selected is not valid.");
  return OperationResult::Success(String(sp, 9));
}
COMMAND("CONVERT_TIME_FW", setConversionTimeFW)

OperationResult getConversionTime(int adc_channel) {
  if (!ADCController::isChannelIndexValid(adc_channel))
    return OperationResult::Failure("Invalid channel index");
  int b = boardForChannel(adc_channel);
  bool success = false;
  float t = boardGetConversionTime(b, localChannel(adc_channel),
                                   boardIsMoreThanOneChannelActive(b),
                                   &success);
  if (!success) return OperationResult::Failure("ADC read failed");
  return OperationResult::Success(String(t, 9));
}
COMMAND("GET_CONVERT_TIME", getConversionTime)

OperationResult getRevisionRegister(int board_index) {
  if (board_index < 0 || board_index >= NUM_ADC_BOARDS)
    return OperationResult::Failure("Invalid board index");
  bool success = false;
  const uint8_t revision =
      readRegister8(board_index, AdcRegister::kRevision, &success);
  if (!success) return OperationResult::Failure("ADC read failed");
  return OperationResult::Success(String(revision));
}
COMMAND("GET_REVISION_REG", getRevisionRegister)

OperationResult continuousConvertRead(int channel_index, uint32_t frequency_us,
                                      uint32_t duration_us) {
  if (!ADCController::isChannelIndexValid(channel_index))
    return OperationResult::Failure("Invalid channel index");
  if (frequency_us < 1) return OperationResult::Failure("Invalid frequency");
  if (duration_us < 1) return OperationResult::Failure("Invalid duration");
  if (frequency_us > duration_us)
    return OperationResult::Failure("Frequency must be less than duration");

  int b = boardForChannel(channel_index);
  int lc = localChannel(channel_index);
  uint32_t num_samples = duration_us / frequency_us;

  FastGpio::digitalWrite(adc_sync, false);
  byte setup_data[4];
  setup_data[0] = AdcRegister::kWrite | AdcRegister::channelSetup(lc);
  setup_data[1] = AdcRegister::kEnableContinuousConversion;
  setup_data[2] = AdcRegister::kWrite | AdcRegister::mode(lc);
  setup_data[3] = AdcRegister::kContinuousConversionMode;
  if (!comms[b].transferADC(setup_data, 4))
    return OperationResult::Failure("ADC write failed");

  FastGpio::digitalWrite(adc_sync, true);
  String result = "";
  for (uint32_t i = 0; i < num_samples; i++) {
    delayMicroseconds(frequency_us);
    bool success = false;
    uint32_t raw =
        readRegister24(b, AdcRegister::channelData(lc), &success);
    if (!success) {
      FastGpio::digitalWrite(adc_sync, false);
      boardIdleMode(b, lc);
      return OperationResult::Failure("ADC read failed");
    }
    result += String(AdcRegister::toDouble(raw), 9) + ",";
  }
  FastGpio::digitalWrite(adc_sync, false);
  boardIdleMode(b, lc);
  result = result.substring(0, result.length() - 1);
  return OperationResult::Success(result);
}
COMMAND("CONTINUOUS_CONVERT_READ", continuousConvertRead)

OperationResult getChannelsActive() {
  String output = "";
  for (int b = 0; b < NUM_ADC_BOARDS; b++) {
    bool success = false;
    uint8_t status = readRegister8(b, AdcRegister::kAdcStatus, &success);
    if (!success) return OperationResult::Failure("ADC read failed");
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      if (status & (1 << i)) {
        if (output.length() > 0) output += ",";
        output += String(b * NUM_CHANNELS_PER_ADC_BOARD + i);
      }
    }
  }
  return OperationResult::Success(output.length() == 0 ? "NONE" : output);
}
COMMAND("GET_CHANNELS_ACTIVE", getChannelsActive)

OperationResult resetAllADCBoards() {
  if (!ADCController::resetToPreviousConversionTimes())
    return OperationResult::Failure("ADC reset failed");
  return OperationResult::Success();
}
COMMAND("RESET", resetAllADCBoards)

OperationResult talkADC(byte command) {
  String results = "";
  for (int b = 0; b < NUM_ADC_BOARDS; b++) {
    uint8_t r = comms[b].transferADC(command);
    results += String(r, 9) + "\n";
  }
  return OperationResult::Success(results);
}
COMMAND("TALK", talkADC)

OperationResult adcChannelSystemZeroScaleCal(int channel) {
  if (channel < 0 || channel >= NUM_ADC_CHANNELS) {
    return OperationResult::Failure("Invalid channel index");
  }
  int b = channel / NUM_CHANNELS_PER_ADC_BOARD;
  int i = channel % NUM_CHANNELS_PER_ADC_BOARD;
  byte data[2] = {
      static_cast<byte>(AdcRegister::kWrite | AdcRegister::mode(i)),
      AdcRegister::kChannelZeroScaleSystemCalMode};
  FastGpio::digitalWrite(adc_sync, true);
  if (!comms[b].transferADC(data, 2))
    return OperationResult::Failure("ADC write failed");
  if (!waitDataReady(b)) {
    boardIdleMode(b, i);
    return OperationResult::Failure("ADC calibration timed out for channel " +
                                    String(channel));
  }
  bool readSucceeded = false;
  uint32_t cal = readRegister24(
      b, AdcRegister::channelZeroScaleCal(i), &readSucceeded);
  if (!readSucceeded)
    return OperationResult::Failure("ADC calibration read failed");
  CalibrationData cd;
  readCalibrationData(cd);
  AdcCalibrationData* adcData = adcCalibration(cd);
  if (!adcData) {
    return OperationResult::Failure("ADC calibration section is missing");
  }
  adcData->offset[channel] = cal;
  adcData->calibrated = true;
  updateCalibrationData(cd);
  return OperationResult::Success("Calibration finished for channel " + String(channel));
}
COMMAND("CALIBRATE_ADC_CHANNEL_ZERO_SCALE", adcChannelSystemZeroScaleCal)

OperationResult adcAllChannelsSystemZeroScaleCal() {
  for (int i = 0; i < NUM_ADC_CALIBRATION_CHANNELS; i++) {
    OperationResult result = adcChannelSystemZeroScaleCal(i);
    if (!result.isSuccess()) return result;
  }
  return OperationResult::Success("Calibration finished for all channels");
}
COMMAND("CALIBRATE_ALL_ADC_CHANNELS_ZERO_SCALE", adcAllChannelsSystemZeroScaleCal)


OperationResult adcChannelSystemFullScaleCal(int channel) {
  if (channel < 0 || channel >= NUM_ADC_CHANNELS) {
    return OperationResult::Failure("Invalid channel index");
  }
  int b = channel / NUM_CHANNELS_PER_ADC_BOARD;
  int i = channel % NUM_CHANNELS_PER_ADC_BOARD;
  byte data[2] = {
      static_cast<byte>(AdcRegister::kWrite | AdcRegister::mode(i)),
      AdcRegister::kChannelFullScaleSystemCalMode};
  FastGpio::digitalWrite(adc_sync, true);
  if (!comms[b].transferADC(data, 2))
    return OperationResult::Failure("ADC write failed");
  if (!waitDataReady(b)) {
    boardIdleMode(b, i);
    return OperationResult::Failure("ADC calibration timed out for channel " +
                                    String(channel));
  }
  bool readSucceeded = false;
  uint32_t cal = readRegister24(
      b, AdcRegister::channelFullScaleCal(i), &readSucceeded);
  if (!readSucceeded)
    return OperationResult::Failure("ADC calibration read failed");
  CalibrationData cd;
  readCalibrationData(cd);
  AdcCalibrationData* adcData = adcCalibration(cd);
  if (!adcData) {
    return OperationResult::Failure("ADC calibration section is missing");
  }
  adcData->gain[channel] = cal;
  adcData->calibrated = true;
  updateCalibrationData(cd);
  return OperationResult::Success("Calibration finished for channel " + String(channel));
}
COMMAND("CALIBRATE_ADC_CHANNEL_FULL_SCALE", adcChannelSystemFullScaleCal)

OperationResult adcAllChannelsSystemFullScaleCal() {
  for (int i = 0; i < NUM_ADC_CALIBRATION_CHANNELS; i++) {
    OperationResult result = adcChannelSystemFullScaleCal(i);
    if (!result.isSuccess()) return result;
  }
  return OperationResult::Success("Calibration finished for all channels");
}
COMMAND("CALIBRATE_ALL_ADC_CHANNELS_FULL_SCALE", adcAllChannelsSystemFullScaleCal)


OperationResult getSavedChZeroScaleCalibration(int ch) {
  if (ch < 0 || ch >= NUM_ADC_CHANNELS)
    return OperationResult::Failure("Invalid channel index");
  CalibrationData data;
  readCalibrationData(data);
  const AdcCalibrationData* adcData = adcCalibration(data);
  if (!adcData) {
    return OperationResult::Failure("ADC calibration section is missing");
  }
  return OperationResult::Success(String(adcData->offset[ch]));
}
COMMAND("GET_SAVED_ZERO_SCALE_CAL", getSavedChZeroScaleCalibration)

OperationResult getSavedChFullScaleCalibration(int ch) {
  if (ch < 0 || ch >= NUM_ADC_CHANNELS)
    return OperationResult::Failure("Invalid channel index");
  CalibrationData data;
  readCalibrationData(data);
  const AdcCalibrationData* adcData = adcCalibration(data);
  if (!adcData) {
    return OperationResult::Failure("ADC calibration section is missing");
  }
  return OperationResult::Success(String(adcData->gain[ch]));
}
COMMAND("GET_SAVED_FULL_SCALE_CAL", getSavedChFullScaleCalibration)

OperationResult setSavedChZeroScaleCalibration(int ch, uint32_t value) {
  if (ch < 0 || ch >= NUM_ADC_CHANNELS)
    return OperationResult::Failure("Invalid channel index");
  if (value > kMaxCalibrationValue)
    return OperationResult::Failure("Invalid zero scale calibration value");
  CalibrationData data;
  readCalibrationData(data);
  AdcCalibrationData* adcData = adcCalibration(data);
  if (!adcData) {
    return OperationResult::Failure("ADC calibration section is missing");
  }
  adcData->offset[ch] = value;
  adcData->calibrated = true;
  updateCalibrationData(data);
  return OperationResult::Success("Saved zero scale calibration");
}
COMMAND("SET_SAVED_ZERO_SCALE_CAL", setSavedChZeroScaleCalibration)

OperationResult setSavedChFullScaleCalibration(int ch, uint32_t value) {
  if (ch < 0 || ch >= NUM_ADC_CHANNELS)
    return OperationResult::Failure("Invalid channel index");
  if (value == 0 || value > kMaxCalibrationValue)
    return OperationResult::Failure("Invalid full scale calibration value");
  CalibrationData data;
  readCalibrationData(data);
  AdcCalibrationData* adcData = adcCalibration(data);
  if (!adcData) {
    return OperationResult::Failure("ADC calibration section is missing");
  }
  adcData->gain[ch] = value;
  adcData->calibrated = true;
  updateCalibrationData(data);
  return OperationResult::Success("Saved full scale calibration");
}
COMMAND("SET_SAVED_FULL_SCALE_CAL", setSavedChFullScaleCalibration)

OperationResult setChZeroScaleCalibration(int ch, uint32_t value) {
  auto r = ADCController::applyChZeroScaleCalibration(ch, value);
  if (!r.isSuccess()) return r;
  return setSavedChZeroScaleCalibration(ch, value);
}
COMMAND("SET_ZERO_SCALE_CAL", setChZeroScaleCalibration)

OperationResult setChFullScaleCalibration(int ch, uint32_t value) {
  auto r = ADCController::applyChFullScaleCalibration(ch, value);
  if (!r.isSuccess()) return r;
  return setSavedChFullScaleCalibration(ch, value);
}
COMMAND("SET_FULL_SCALE_CAL", setChFullScaleCalibration)

OperationResult resetToPreviousConversionTimesSerial() {
  if (!ADCController::resetToPreviousConversionTimes())
    return OperationResult::Failure("ADC reset failed");
  return OperationResult::Success("Reset to previous conversion times");
}
COMMAND("RESET_MAINTAIN", resetToPreviousConversionTimesSerial)

OperationResult setChopping(bool chop) {
  for (int b = 0; b < NUM_ADC_BOARDS; b++) chopEnabled[b] = chop;
  return OperationResult::Success();
}
COMMAND("SET_CHOP", setChopping)

OperationResult getChopping() {
  return chopEnabled[0] ? OperationResult::Success("true")
                        : OperationResult::Success("false");
}
COMMAND("GET_CHOP", getChopping)

}  // namespace

namespace ADCController {

void setup() {
  pinMode(adc_sync, OUTPUT);
  FastGpio::digitalWrite(adc_sync, false);
  for (int b = 0; b < NUM_ADC_BOARDS; b++) {
    pinMode(reset[b], OUTPUT);
    pinMode(drdy[b], INPUT);
    pinMode(adc_cs_pins[b], OUTPUT);
    FastGpio::digitalWrite(adc_cs_pins[b], true);
    FastGpio::digitalWrite(reset[b], true);
    FastGpio::digitalWrite(reset[b], false);
    delay(5);
    FastGpio::digitalWrite(reset[b], true);
    pinMode(adc_sync, OUTPUT);
    FastGpio::digitalWrite(adc_sync, false);
    writeRegister8(b, AdcRegister::kIo, kSyncEnabledIoRegister);
  }
}
ON_SETUP(setup)

OperationResult initialize() {
  if (!resetToPreviousConversionTimes())
    return OperationResult::Failure("ADC reset failed");
  for (int b = 0; b < NUM_ADC_BOARDS; b++) {
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      if (boardSetConversionTime(b, i, chopEnabled[b],
                                 calculateFilterWord(500, chopEnabled[b],
                                                     false),
                                 false) < 0)
        return OperationResult::Failure("ADC conversion-time setup failed");
    }
  }
  return OperationResult::Success();
}
ON_INITIALIZE(initialize)

bool resetToPreviousConversionTimes() {
  for (int b = 0; b < NUM_ADC_BOARDS; b++) {
    uint32_t zsCals[NUM_CHANNELS_PER_ADC_BOARD];
    uint32_t fsCals[NUM_CHANNELS_PER_ADC_BOARD];
    uint8_t conversionRegisters[NUM_CHANNELS_PER_ADC_BOARD];
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      bool success = false;
      conversionRegisters[i] = readRegister8(
          b, AdcRegister::channelConversionTime(i), &success);
      if (!success) return false;
    }
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      bool zeroRead = false;
      bool fullRead = false;
      zsCals[i] = readRegister24(
          b, AdcRegister::channelZeroScaleCal(i), &zeroRead);
      fsCals[i] = readRegister24(
          b, AdcRegister::channelFullScaleCal(i), &fullRead);
      if (!zeroRead || !fullRead) return false;
    }
    if (!boardReset(b)) return false;
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      if (!writeRegister24(b, AdcRegister::channelZeroScaleCal(i), zsCals[i]) ||
          !writeRegister24(b, AdcRegister::channelFullScaleCal(i), fsCals[i]))
        return false;
    }
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      if (!writeRegister8(b, AdcRegister::channelConversionTime(i),
                          conversionRegisters[i]))
        return false;
    }
  }
  return true;
}

bool isChannelIndexValid(int ch) {
  return ch >= 0 && ch < NUM_ADC_CHANNELS;
}

OperationResult readChannelVoltage(int ch) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index");
  const float voltage = getVoltage(ch);
  if (!std::isfinite(static_cast<double>(voltage)))
    return OperationResult::Failure("ADC read failed");
  return OperationResult::Success(String(voltage, 9));
}
COMMAND("GET_ADC", readChannelVoltage)

float getVoltage(int ch) {
  if (!isChannelIndexValid(ch)) return NAN;
  int b = boardForChannel(ch);
  int lc = localChannel(ch);
  byte data[2] = {
      static_cast<byte>(AdcRegister::kWrite | AdcRegister::mode(lc)),
      AdcRegister::kSingleConversionMode};
  FastGpio::digitalWrite(adc_sync, false);
  if (!comms[b].transferADC(data, 2)) return NAN;
  FastGpio::digitalWrite(adc_sync, true);
  if (!waitDataReady(b)) return NAN;
  byte readData[4] = {
      static_cast<byte>(AdcRegister::kRead | AdcRegister::channelData(lc)),
      0, 0, 0};
  if (!comms[b].transferADC(readData, 4)) return NAN;
  return conversionDataPacketToVoltage(readData);
}

double getVoltageData(int ch) {
  if (!isChannelIndexValid(ch)) return NAN;
  bool success = false;
  const uint32_t raw = readRegister24(
      boardForChannel(ch), AdcRegister::channelData(localChannel(ch)),
      &success);
  return success ? AdcRegister::toDouble(raw) : NAN;
}

bool startContinuousConversion(int ch) {
  if (!isChannelIndexValid(ch)) return false;
  int b = boardForChannel(ch);
  int lc = localChannel(ch);
  uint8_t setup[2];
  setup[0] = AdcRegister::kWrite | AdcRegister::channelSetup(lc);
  setup[1] = AdcRegister::kEnableContinuousConversion;
  if (!comms[b].transferADC(setup, 2)) return false;
  return selectContinuousConversionChannel(ch);
}

bool selectContinuousConversionChannel(int ch) {
  if (!isChannelIndexValid(ch)) return false;
  int b = boardForChannel(ch);
  int lc = localChannel(ch);
  uint8_t mode[2];
  mode[0] = AdcRegister::kWrite | AdcRegister::mode(lc);
  mode[1] = AdcRegister::kContinuousConversionMode;
  return comms[b].transferADC(mode, 2);
}

OperationResult idleMode(int ch) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index");
  if (!boardIdleMode(boardForChannel(ch), localChannel(ch)))
    return OperationResult::Failure("ADC write failed");
  return OperationResult::Success("Returned ADC " + String(ch) +
                                  " to idle mode");
}
COMMAND("IDLE_MODE", idleMode)

OperationResult setRDYFN(int ch) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index");
  if (!writeRegister8(boardForChannel(ch), AdcRegister::kIo,
                      kReadyFunctionIoRegister))
    return OperationResult::Failure("ADC write failed");
  return OperationResult::Success("Set RDYFN");
}
COMMAND("SET_RDYFN", setRDYFN)

OperationResult unsetRDYFN(int ch) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index");
  if (!writeRegister8(boardForChannel(ch), AdcRegister::kIo,
                      kSyncEnabledIoRegister))
    return OperationResult::Failure("ADC write failed");
  return OperationResult::Success("Unset RDYFN");
}
COMMAND("UNSET_RDYFN", unsetRDYFN)

int getDataReadyPin(int board_index) {
  if (board_index < 0 || board_index >= NUM_ADC_BOARDS) return NC;
  return drdy[board_index];
}

int getCsPin(int ch) {
  if (!isChannelIndexValid(ch)) return NC;
  return adc_cs_pins[boardForChannel(ch)];
}

bool buildConversionDataRead(int ch, byte packet[4]) {
  if (!isChannelIndexValid(ch)) return false;
  packet[0] = AdcRegister::kRead | AdcRegister::channelData(localChannel(ch));
  packet[1] = 0;
  packet[2] = 0;
  packet[3] = 0;
  return true;
}

double conversionDataPacketToVoltage(const byte packet[4]) {
  uint32_t raw = (static_cast<uint32_t>(packet[1]) << 16) |
                 (static_cast<uint32_t>(packet[2]) << 8) |
                 static_cast<uint32_t>(packet[3]);
  return AdcRegister::toDouble(raw);
}

OperationResult setConversionTime(int ch, float time_us) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index");
  int b = boardForChannel(ch);
  float sp = boardSetConversionTimeFloat(b, localChannel(ch), time_us,
                                         boardIsMoreThanOneChannelActive(b));
  if (sp == -1.0)
    return OperationResult::Failure(
        "The filter word you selected is not valid.");
  return OperationResult::Success(String(sp, 9));
}
COMMAND("CONVERT_TIME", setConversionTime)

float presetConversionTime(int ch, int time_us,
                           bool isMoreThanOneChannelActive) {
  return boardSetConversionTimeFloat(boardForChannel(ch), localChannel(ch),
                                     time_us, isMoreThanOneChannelActive);
}

float getConversionTimeFloat(int ch) {
  if (!isChannelIndexValid(ch)) return -1.0;
  int b = boardForChannel(ch);
  return boardGetConversionTime(b, localChannel(ch),
                                boardIsMoreThanOneChannelActive(b));
}

float getConversionTimeFloat(int ch, bool isMoreThanOneChannelActive) {
  if (!isChannelIndexValid(ch)) return -1.0;
  return boardGetConversionTime(boardForChannel(ch), localChannel(ch),
                                isMoreThanOneChannelActive);
}

OperationResult getChZeroScaleCalibration(int ch) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index");
  bool success = false;
  uint32_t val = readRegister24(
      boardForChannel(ch), AdcRegister::channelZeroScaleCal(localChannel(ch)),
      &success);
  if (!success) return OperationResult::Failure("ADC read failed");
  return OperationResult::Success(String(val));
}
COMMAND("GET_ZERO_SCALE_CAL", getChZeroScaleCalibration)

OperationResult getChFullScaleCalibration(int ch) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index");
  bool success = false;
  uint32_t val = readRegister24(
      boardForChannel(ch), AdcRegister::channelFullScaleCal(localChannel(ch)),
      &success);
  if (!success) return OperationResult::Failure("ADC read failed");
  return OperationResult::Success(String(val));
}
COMMAND("GET_FULL_SCALE_CAL", getChFullScaleCalibration)

OperationResult applyChZeroScaleCalibration(int ch, uint32_t value) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index");
  if (value > kMaxCalibrationValue)
    return OperationResult::Failure("Invalid zero scale calibration value");
  if (!writeRegister24(boardForChannel(ch),
                       AdcRegister::channelZeroScaleCal(localChannel(ch)),
                       value))
    return OperationResult::Failure("ADC write failed");
  return OperationResult::Success("Applied zero scale calibration");
}

OperationResult applyChFullScaleCalibration(int ch, uint32_t value) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index");
  if (value == 0 || value > kMaxCalibrationValue)
    return OperationResult::Failure("Invalid full scale calibration value");
  if (!writeRegister24(boardForChannel(ch),
                       AdcRegister::channelFullScaleCal(localChannel(ch)),
                       value))
    return OperationResult::Failure("ADC write failed");
  return OperationResult::Success("Applied full scale calibration");
}

OperationResult hardResetAllADCBoards() {
  for (int b = 0; b < NUM_ADC_BOARDS; b++) {
    FastGpio::digitalWrite(reset[b], true);
    FastGpio::digitalWrite(reset[b], false);
    delay(5);
    FastGpio::digitalWrite(reset[b], true);
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) boardIdleMode(b, i);
  }
  CalibrationData data;
  readCalibrationData(data);
  AdcCalibrationData* adcData = adcCalibration(data);
  if (adcData) adcData->calibrated = false;
  updateCalibrationData(data);
  return OperationResult::Success("All ADC boards have been hard reset");
}
COMMAND("HARD_RESET", hardResetAllADCBoards)

void applySavedCalibration() {
  while (!isCalibrationDataReady()) {
    delay(1);
  }

  CalibrationData calibration_data;
  readCalibrationData(calibration_data);
  AdcCalibrationData* adcData = adcCalibration(calibration_data);
  if (!adcData) return;

  if (!adcData->calibrated) {
    hardResetAllADCBoards();
    for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
      uint32_t zeroScaleCalibration =
          getChZeroScaleCalibration(i).getMessage().toInt();
      uint32_t fullScaleCalibration =
          getChFullScaleCalibration(i).getMessage().toInt();

      adcData->offset[i] = zeroScaleCalibration;
      adcData->gain[i] = fullScaleCalibration;
      adcData->calibrated = true;
    }
    updateCalibrationData(calibration_data);
    return;
  }

  for (int i = 0; i < NUM_ADC_CHANNELS; i++) {
    applyChZeroScaleCalibration(i, adcData->offset[i]);

    if (adcData->gain[i] != 0) {
      applyChFullScaleCalibration(i, adcData->gain[i]);
    }
  }
}
ON_SETUP_CALIBRATION(applySavedCalibration)

}  // namespace ADCController
