#include "Peripherals/ADC/ADCController.h"

#include "Calibration/Calibration.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "PeripheralCommsController.h"
#include "Utils/FastGpio.h"
#include "shared_memory.h"

namespace {

constexpr uint8_t kSyncEnabledIoRegister = 0b00010001;
constexpr uint8_t kReadyFunctionIoRegister = 0b00011001;

int boardForChannel(int ch) { return ch / NUM_CHANNELS_PER_ADC_BOARD; }
int localChannel(int ch) { return ch % NUM_CHANNELS_PER_ADC_BOARD; }

PeripheralCommsController comms[NUM_ADC_BOARDS] = {
    PeripheralCommsController(adc_cs_pins[0]),
    PeripheralCommsController(adc_cs_pins[1]),
};

bool chopEnabled[NUM_ADC_BOARDS] = {true, true};

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
    if (data.offset[i] > 0xFFFFFFu || data.gain[i] > 0xFFFFFFu) {
      return false;
    }
  }
  return true;
}
CALIBRATION_SECTION("ADC", AdcCalibrationData, setAdcCalibrationDefaults,
                    validateAdcCalibration)

uint8_t readRegister8(int board, uint8_t address) {
  byte data[2] = {static_cast<byte>(AdcRegister::kRead | address), 0};
  comms[board].transferADC(data, 2);
  return data[1];
}

uint32_t readRegister24(int board, uint8_t address,
                        bool noTransaction = false) {
  byte data[4] = {static_cast<byte>(AdcRegister::kRead | address), 0, 0, 0};
  if (noTransaction)
    comms[board].transferADCNoTransaction(data, 4);
  else
    comms[board].transferADC(data, 4);
  return (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) |
         static_cast<uint32_t>(data[3]);
}

void writeRegister8(int board, uint8_t address, uint8_t value) {
  byte data[2] = {static_cast<byte>(AdcRegister::kWrite | address), value};
  comms[board].transferADC(data, 2);
}

void writeRegister24(int board, uint8_t address, uint32_t value) {
  byte data[4] = {static_cast<byte>(AdcRegister::kWrite | address),
                  static_cast<byte>((value >> 16) & 0xFF),
                  static_cast<byte>((value >> 8) & 0xFF),
                  static_cast<byte>(value & 0xFF)};
  comms[board].transferADC(data, 4);
}

void waitDataReady(int board) {
  int count = 0;
  while (digitalRead(drdy[board]) == HIGH && count < 20000) {
    count++;
    delay(1);
  }
  FastGpio::digitalWrite(adc_sync, false);
}

void boardIdleMode(int board, int local_ch) {
  writeRegister8(board, AdcRegister::mode(local_ch),
                 AdcRegister::kIdleMode);
}

void boardReset(int board) {
  FastGpio::digitalWrite(reset[board], true);
  FastGpio::digitalWrite(reset[board], false);
  delay(5);
  FastGpio::digitalWrite(reset[board], true);
  for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) boardIdleMode(board, i);
  writeRegister8(board, AdcRegister::kIo, kSyncEnabledIoRegister);
  if (!isCalibrationDataReady()) return;
  CalibrationData data;
  readCalibrationData(data);
  const AdcCalibrationData* adcData = adcCalibration(data);
  if (!adcData || !adcData->calibrated) return;
  for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
    int global = board * NUM_CHANNELS_PER_ADC_BOARD + i;
    writeRegister24(board, AdcRegister::channelZeroScaleCal(i),
                    adcData->offset[global]);
    if (adcData->gain[global] != 0)
      writeRegister24(board, AdcRegister::channelFullScaleCal(i),
                      adcData->gain[global]);
  }
}

float calculateConversionTime(int board, byte b,
                              bool moreThanOneChannelActive) {
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

bool boardIsMoreThanOneChannelActive(int board) {
  const uint8_t status =
      readRegister8(board, AdcRegister::kAdcStatus) &
      ((1 << NUM_CHANNELS_PER_ADC_BOARD) - 1);
  return (status & (status - 1)) != 0;
}

float boardGetConversionTime(int board, int local_ch,
                             bool moreThanOneChannelActive) {
  return calculateConversionTime(
      board,
      readRegister8(board, AdcRegister::channelConversionTime(local_ch)),
      moreThanOneChannelActive);
}

float boardSetConversionTime(int board, int local_ch, bool chop, byte fw,
                             bool moreThanOneChannelActive) {
  if ((fw > 127) || (chop && fw < 2) || (!chop && fw < 3)) return -1;
  byte send = (chop ? 0x80 : 0x00) | fw;
  writeRegister8(board, AdcRegister::channelConversionTime(local_ch), send);
  float t = boardGetConversionTime(board, local_ch, moreThanOneChannelActive);
  delayMicroseconds(100);
  return t;
}

float boardSetConversionTimeFloat(int board, int local_ch, float time_us,
                                  bool moreThanOneChannelActive) {
  return boardSetConversionTime(
      board, local_ch, chopEnabled[board],
      calculateFilterWord(time_us, true, moreThanOneChannelActive),
      moreThanOneChannelActive);
}

// --- command handlers (registered via function registry) ---

OperationResult setConversionTimeFW(int adc_channel, int filter_word) {
  if (!ADCController::isChannelIndexValid(adc_channel))
    return OperationResult::Failure("Invalid channel index");
  int b = boardForChannel(adc_channel);
  float sp = boardSetConversionTime(b, localChannel(adc_channel), true,
                                    filter_word,
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
  float t = boardGetConversionTime(b, localChannel(adc_channel),
                                   boardIsMoreThanOneChannelActive(b));
  return OperationResult::Success(String(t, 9));
}
COMMAND("GET_CONVERT_TIME", getConversionTime)

OperationResult getRevisionRegister(int board_index) {
  if (board_index < 0 || board_index >= NUM_ADC_BOARDS)
    return OperationResult::Failure("Invalid board index");
  return OperationResult::Success(
      String(readRegister8(board_index, AdcRegister::kRevision)));
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

  byte setup_data[4];
  setup_data[0] = AdcRegister::kWrite | AdcRegister::channelSetup(lc);
  setup_data[1] = AdcRegister::kEnableContinuousConversion;
  setup_data[2] = AdcRegister::kWrite | AdcRegister::mode(lc);
  setup_data[3] = AdcRegister::kContinuousConversionMode;
  comms[b].transferADC(setup_data, 4);

  String result = "";
  for (uint32_t i = 0; i < num_samples; i++) {
    uint32_t raw = readRegister24(b, AdcRegister::channelData(lc));
    result += String(AdcRegister::toDouble(raw), 9) + ",";
    delayMicroseconds(frequency_us);
  }
  boardIdleMode(b, lc);
  result = result.substring(0, result.length() - 1);
  return OperationResult::Success(result);
}
COMMAND("CONTINUOUS_CONVERT_READ", continuousConvertRead)

OperationResult getChannelsActive() {
  String output = "";
  for (int b = 0; b < NUM_ADC_BOARDS; b++) {
    uint8_t status = readRegister8(b, AdcRegister::kAdcStatus);
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
  for (int b = 0; b < NUM_ADC_BOARDS; b++) boardReset(b);
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

OperationResult adcZeroScaleCal() {
  for (int b = 0; b < NUM_ADC_BOARDS; b++) {
    byte data[2] = {static_cast<byte>(AdcRegister::kWrite | AdcRegister::mode(0)),
                    AdcRegister::kZeroScaleSelfCalMode};
    FastGpio::digitalWrite(adc_sync, true);
    comms[b].transferADC(data, 2);
    waitDataReady(b);
  }
  return OperationResult::Success("CALIBRATION_FINISHED");
}
COMMAND("ADC_ZERO_SC_CAL", adcZeroScaleCal)

OperationResult adcChannelSystemZeroScaleCal() {
  for (int b = 0; b < NUM_ADC_BOARDS; b++) {
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      byte data[2] = {
          static_cast<byte>(AdcRegister::kWrite | AdcRegister::mode(i)),
          AdcRegister::kChannelZeroScaleSystemCalMode};
      FastGpio::digitalWrite(adc_sync, true);
      comms[b].transferADC(data, 2);
      waitDataReady(b);
      uint32_t cal = readRegister24(b, AdcRegister::channelZeroScaleCal(i));
      CalibrationData cd;
      readCalibrationData(cd);
      AdcCalibrationData* adcData = adcCalibration(cd);
      if (!adcData) {
        return OperationResult::Failure("ADC calibration section is missing");
      }
      adcData->offset[b * NUM_CHANNELS_PER_ADC_BOARD + i] = cal;
      updateCalibrationData(cd);
    }
  }
  return OperationResult::Success("CALIBRATION_FINISHED");
}
COMMAND("ADC_CH_ZERO_SC_CAL", adcChannelSystemZeroScaleCal)

OperationResult adcChannelSystemFullScaleCal() {
  for (int b = 0; b < NUM_ADC_BOARDS; b++) {
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      byte data[2] = {
          static_cast<byte>(AdcRegister::kWrite | AdcRegister::mode(i)),
          AdcRegister::kChannelFullScaleSystemCalMode};
      FastGpio::digitalWrite(adc_sync, true);
      comms[b].transferADC(data, 2);
      waitDataReady(b);
      uint32_t cal = readRegister24(b, AdcRegister::channelFullScaleCal(i));
      CalibrationData cd;
      readCalibrationData(cd);
      AdcCalibrationData* adcData = adcCalibration(cd);
      if (!adcData) {
        return OperationResult::Failure("ADC calibration section is missing");
      }
      adcData->gain[b * NUM_CHANNELS_PER_ADC_BOARD + i] = cal;
      updateCalibrationData(cd);
    }
  }
  return OperationResult::Success("CALIBRATION_FINISHED");
}
COMMAND("ADC_CH_FULL_SC_CAL", adcChannelSystemFullScaleCal)

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
  CalibrationData data;
  readCalibrationData(data);
  AdcCalibrationData* adcData = adcCalibration(data);
  if (!adcData) {
    return OperationResult::Failure("ADC calibration section is missing");
  }
  adcData->offset[ch] = value;
  updateCalibrationData(data);
  return OperationResult::Success("Saved zero scale calibration");
}
COMMAND("SET_SAVED_ZERO_SCALE_CAL", setSavedChZeroScaleCalibration)

OperationResult setSavedChFullScaleCalibration(int ch, uint32_t value) {
  if (ch < 0 || ch >= NUM_ADC_CHANNELS)
    return OperationResult::Failure("Invalid channel index");
  CalibrationData data;
  readCalibrationData(data);
  AdcCalibrationData* adcData = adcCalibration(data);
  if (!adcData) {
    return OperationResult::Failure("ADC calibration section is missing");
  }
  adcData->gain[ch] = value;
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
  ADCController::resetToPreviousConversionTimes();
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

void initialize() {
  for (int b = 0; b < NUM_ADC_BOARDS; b++) {
    boardReset(b);
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      boardSetConversionTime(b, i, chopEnabled[b],
                             calculateFilterWord(500, true, false), false);
    }
  }
}
ON_INITIALIZE(initialize)

void resetToPreviousConversionTimes() {
  for (int b = 0; b < NUM_ADC_BOARDS; b++) {
    uint32_t zsCals[NUM_CHANNELS_PER_ADC_BOARD];
    uint32_t fsCals[NUM_CHANNELS_PER_ADC_BOARD];
    float convTimes[NUM_CHANNELS_PER_ADC_BOARD];
    bool multi = boardIsMoreThanOneChannelActive(b);
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      convTimes[i] = boardGetConversionTime(b, i, multi);
    }
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      zsCals[i] = readRegister24(b, AdcRegister::channelZeroScaleCal(i));
      fsCals[i] = readRegister24(b, AdcRegister::channelFullScaleCal(i));
    }
    boardReset(b);
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      writeRegister24(b, AdcRegister::channelZeroScaleCal(i), zsCals[i]);
      writeRegister24(b, AdcRegister::channelFullScaleCal(i), fsCals[i]);
    }
    for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
      boardSetConversionTimeFloat(b, i, convTimes[i],
                                  boardIsMoreThanOneChannelActive(b));
    }
  }
}

bool isChannelIndexValid(int ch) {
  return ch >= 0 && ch < NUM_ADC_CHANNELS;
}

OperationResult readChannelVoltage(int ch) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index");
  return OperationResult::Success(String(getVoltage(ch), 9));
}
COMMAND("GET_ADC", readChannelVoltage)

float getVoltage(int ch) {
  int b = boardForChannel(ch);
  int lc = localChannel(ch);
  byte data[2] = {
      static_cast<byte>(AdcRegister::kWrite | AdcRegister::mode(lc)),
      AdcRegister::kSingleConversionMode};
  FastGpio::digitalWrite(adc_sync, false);
  comms[b].transferADC(data, 2);
  FastGpio::digitalWrite(adc_sync, true);
  waitDataReady(b);
  uint32_t raw = readRegister24(b, AdcRegister::channelData(lc));
  return AdcRegister::toDouble(raw);
}

double getVoltageData(int ch) {
  return AdcRegister::toDouble(
      readRegister24(boardForChannel(ch),
                     AdcRegister::channelData(localChannel(ch))));
}

double getVoltageDataNoTransaction(int ch) {
  return AdcRegister::toDouble(
      readRegister24(boardForChannel(ch),
                     AdcRegister::channelData(localChannel(ch)), true));
}

void startContinuousConversion(int ch) {
  int b = boardForChannel(ch);
  int lc = localChannel(ch);
  uint8_t data[4];
  data[0] = AdcRegister::kWrite | AdcRegister::channelSetup(lc);
  data[1] = AdcRegister::kEnableContinuousConversion;
  data[2] = AdcRegister::kWrite | AdcRegister::mode(lc);
  data[3] = AdcRegister::kContinuousConversionMode;
  comms[b].transferADC(data, 4);
}

OperationResult idleMode(int ch) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index");
  boardIdleMode(boardForChannel(ch), localChannel(ch));
  return OperationResult::Success("Returned ADC " + String(ch) +
                                  " to idle mode");
}
COMMAND("IDLE_MODE", idleMode)

OperationResult setRDYFN(int ch) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index");
  writeRegister8(boardForChannel(ch), AdcRegister::kIo,
                 kReadyFunctionIoRegister);
  return OperationResult::Success("Set RDYFN");
}
COMMAND("SET_RDYFN", setRDYFN)

OperationResult unsetRDYFN(int ch) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index");
  writeRegister8(boardForChannel(ch), AdcRegister::kIo,
                 kSyncEnabledIoRegister);
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
  uint32_t val = readRegister24(boardForChannel(ch),
                                AdcRegister::channelZeroScaleCal(localChannel(ch)));
  return OperationResult::Success(String(val));
}
COMMAND("GET_ZERO_SCALE_CAL", getChZeroScaleCalibration)

OperationResult getChFullScaleCalibration(int ch) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index");
  uint32_t val = readRegister24(boardForChannel(ch),
                                AdcRegister::channelFullScaleCal(localChannel(ch)));
  return OperationResult::Success(String(val));
}
COMMAND("GET_FULL_SCALE_CAL", getChFullScaleCalibration)

OperationResult applyChZeroScaleCalibration(int ch, uint32_t value) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index");
  writeRegister24(boardForChannel(ch),
                  AdcRegister::channelZeroScaleCal(localChannel(ch)), value);
  return OperationResult::Success("Applied zero scale calibration");
}

OperationResult applyChFullScaleCalibration(int ch, uint32_t value) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index");
  writeRegister24(boardForChannel(ch),
                  AdcRegister::channelFullScaleCal(localChannel(ch)), value);
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
