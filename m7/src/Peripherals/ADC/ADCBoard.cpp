#include "Peripherals/ADC/ADCBoard.h"

#include "Utils/FastGpio.h"

namespace {
constexpr uint8_t kSyncEnabledIoRegister = 0b00010001;
constexpr uint8_t kReadyFunctionIoRegister = 0b00011001;

uint8_t readCommand(uint8_t address) {
  return AdcRegister::kRead | address;
}

uint8_t writeCommand(uint8_t address) {
  return AdcRegister::kWrite | address;
}

uint32_t unpack24BitRegister(const byte data[4]) {
  return (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) |
         static_cast<uint32_t>(data[3]);
}
}  // namespace



void ADCBoard::waitDataReady() {
  int count = 0;
  while (digitalRead(data_ready_pin) == HIGH && count < 20000) {
    count = count + 1;
    delay(1);
  }
  FastGpio::digitalWrite(adc_sync, false);
}


uint8_t ADCBoard::readRegister8(uint8_t address) {
  byte data[2] = {readCommand(address), 0};
  commsController.transferADC(data, 2);
  return data[1];
}



uint32_t ADCBoard::readRegister24(uint8_t address, bool noTransaction) {
  byte data[4] = {readCommand(address), 0, 0, 0};
  if (noTransaction) {
    commsController.transferADCNoTransaction(data, 4);
  } else {
    commsController.transferADC(data, 4);
  }
  return unpack24BitRegister(data);
}



void ADCBoard::writeRegister8(uint8_t address, uint8_t value) {
  byte data[2] = {writeCommand(address), value};
  commsController.transferADC(data, 2);
}



void ADCBoard::writeRegister24(uint8_t address, uint32_t value) {
  byte data[4] = {writeCommand(address),
                  static_cast<byte>((value >> 16) & 0xFF),
                  static_cast<byte>((value >> 8) & 0xFF),
                  static_cast<byte>(value & 0xFF)};
  commsController.transferADC(data, 4);
}



ADCBoard::ADCBoard(int cs_pin, int data_ready_pin, int reset_pin, int board_idx)
    : cs_pin(cs_pin),
      data_ready_pin(data_ready_pin),
      reset_pin(reset_pin),
      board_idx(board_idx),
      commsController(cs_pin) {}



void ADCBoard::setup() {
  pinMode(reset_pin, OUTPUT);

  pinMode(data_ready_pin, INPUT);

  pinMode(cs_pin, OUTPUT);
  FastGpio::digitalWrite(cs_pin, true);

  // Resets ADC on startup.
  FastGpio::digitalWrite(reset_pin, true);
  FastGpio::digitalWrite(reset_pin, false);
  delay(5);
  FastGpio::digitalWrite(reset_pin, true);

  pinMode(adc_sync, OUTPUT);
  FastGpio::digitalWrite(adc_sync, false);

  writeRegister8(AdcRegister::kIo, kSyncEnabledIoRegister);
}



void ADCBoard::RDY_ISR() {
  setReadyFlag();
}



void ADCBoard::initialize() {
  reset();
  for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
    setConversionTime(i, 500);
  }
}



uint32_t ADCBoard::getZeroScaleCalibration(int adc_channel) {
  return readRegister24(AdcRegister::channelZeroScaleCal(adc_channel));
}



uint32_t ADCBoard::getFullScaleCalibration(int adc_channel) {
  return readRegister24(AdcRegister::channelFullScaleCal(adc_channel));
}



void ADCBoard::setZeroScaleCalibration(int adc_channel, uint32_t value) {
  writeRegister24(AdcRegister::channelZeroScaleCal(adc_channel), value);
}



void ADCBoard::setFullScaleCalibration(int adc_channel, uint32_t value) {
  writeRegister24(AdcRegister::channelFullScaleCal(adc_channel), value);
}



void ADCBoard::resetToPreviousConversionTimes() {
  uint32_t zeroScaleCalibrations[NUM_CHANNELS_PER_ADC_BOARD];
  uint32_t fullScaleCalibrations[NUM_CHANNELS_PER_ADC_BOARD];

  float conversion_times[NUM_CHANNELS_PER_ADC_BOARD];
  for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
    conversion_times[i] = getConversionTime(i);
  }

  for(int i = 0; i<NUM_CHANNELS_PER_ADC_BOARD; i++) {
    zeroScaleCalibrations[i] = getZeroScaleCalibration(i);
    fullScaleCalibrations[i] = getFullScaleCalibration(i);
  }

  reset();

  for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
    setZeroScaleCalibration(i, zeroScaleCalibrations[i]);
    setFullScaleCalibration(i, fullScaleCalibrations[i]);
  }
  for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
    setConversionTime(i, conversion_times[i]);
  }
}



int ADCBoard::getDataReadyPin() const { return data_ready_pin; }


int ADCBoard::getBoardIndex() const { return board_idx; }



int ADCBoard::getCsPin() const { return cs_pin; }



void ADCBoard::setReadyFlag() { data_ready = true; }


void ADCBoard::clearReadyFlag() { data_ready = false; }



double ADCBoard::readVoltage(int channel_index) {
  startSingleConversion(channel_index);
  waitDataReady();
  uint32_t data = getConversionData(channel_index);
  return AdcRegister::toDouble(data);
}



// return ADC status register, pg. 16
uint8_t ADCBoard::getADCStatus() {
  return readRegister8(AdcRegister::kAdcStatus);
}



void ADCBoard::setRDYFN() {
  writeRegister8(AdcRegister::kIo, kReadyFunctionIoRegister);
}



void ADCBoard::unsetRDYFN() {
  writeRegister8(AdcRegister::kIo, kSyncEnabledIoRegister);
}



void ADCBoard::channelSetup(int adc_channel, uint8_t flags) {
  writeRegister8(AdcRegister::channelSetup(adc_channel), flags);
}



// tells the ADC to start a single conversion on the passed channel
void ADCBoard::startSingleConversion(int adc_channel) {
  byte data[2];
  // setup communication register for writing operation to the mode register
  data[0] = writeCommand(AdcRegister::mode(adc_channel));
  // setup mode register
  data[1] = AdcRegister::kSingleConversionMode;
  FastGpio::digitalWrite(adc_sync, false);
  commsController.transferADC(data, 2);
  FastGpio::digitalWrite(adc_sync, true);

  // data is ready when _rdy goes low
}



// tells the ADC to start a continous conversion on the passed channel
void ADCBoard::startContinuousConversion(int adc_channel) {
  uint8_t data_array[4];

  // address the channel setup register and write to it
  data_array[0] = writeCommand(AdcRegister::channelSetup(adc_channel));
  data_array[1] = AdcRegister::kEnableContinuousConversion;

  // address the channel mode register and write to it
  data_array[2] = writeCommand(AdcRegister::mode(adc_channel));
  data_array[3] = AdcRegister::kContinuousConversionMode;

  // send off command
  commsController.transferADC(data_array, 4);

  // data is ready when _rdy goes low
}



uint8_t ADCBoard::getRevisionRegister() {
  return readRegister8(AdcRegister::kRevision);
}



void ADCBoard::setConversionTime(int adc_channel, int chop, int fw) {
  byte chop_byte = chop == 1 ? 0x80 : 0x00;
  byte send = chop_byte | static_cast<byte>(fw);

  writeRegister8(AdcRegister::channelConversionTime(adc_channel), send);
}



uint32_t ADCBoard::getConversionData(int adc_channel) {
  return readRegister24(AdcRegister::channelData(adc_channel));
}



uint32_t ADCBoard::getConversionDataNoTransaction(int adc_channel) {
  return readRegister24(AdcRegister::channelData(adc_channel), true);
}



std::vector<double> ADCBoard::continuousConvert(int channel_index, uint32_t period_us,
                                      uint32_t duration) {
  uint32_t num_samples = duration / period_us;
  std::vector<double> data(num_samples);
  startContinuousConversion(channel_index);
  for (uint32_t i = 0; i < num_samples; i++) {
    data[i] = AdcRegister::toDouble(getConversionData(channel_index));
    delayMicroseconds(period_us);
  }
  idleMode(channel_index);
  return data;
}



void ADCBoard::idleMode(int adc_channel) {
  writeRegister8(AdcRegister::mode(adc_channel),
                 AdcRegister::kIdleMode);
}



bool ADCBoard::isChannelActive(int adc_channel) {
  uint8_t status = getADCStatus();
  return (status & (1 << adc_channel)) != 0;
}



void ADCBoard::hardReset() {
  FastGpio::digitalWrite(reset_pin, true);
  FastGpio::digitalWrite(reset_pin, false);
  delay(5);
  FastGpio::digitalWrite(reset_pin, true);

  for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
    idleMode(i);
  }
}



void ADCBoard::restoreCalibrationFromFlash() {
  if (!isCalibrationDataReady()) {
    return;
  }

  CalibrationData data;
  readCalibrationData(data);

  if (!data.adcCalibrated) {
    return;
  }

  int boardIndex = getBoardIndex();

  for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
    setZeroScaleCalibration(i, data.adc_offset[NUM_CHANNELS_PER_ADC_BOARD * boardIndex + i]);

    uint32_t fullScale = data.adc_gain[NUM_CHANNELS_PER_ADC_BOARD * boardIndex + i];
    if (fullScale != 0) {
      setFullScaleCalibration(i, fullScale);
    }
  }
}



void ADCBoard::reset() {
  FastGpio::digitalWrite(reset_pin, true);
  FastGpio::digitalWrite(reset_pin, false);
  delay(5);
  FastGpio::digitalWrite(reset_pin, true);

  for (int i = 0; i < NUM_CHANNELS_PER_ADC_BOARD; i++) {
    idleMode(i);
  }

  writeRegister8(AdcRegister::kIo, kSyncEnabledIoRegister);

  restoreCalibrationFromFlash();
}



uint8_t ADCBoard::talkADC(byte command) {
  uint8_t comm = commsController.transferADC(command);
  return comm;
}



float ADCBoard::setConversionTime(int channel, float time_us) {
  return setConversionTimeFloat(channel, time_us,
                                isMoreThanOneChannelActive());
}



float ADCBoard::setConversionTimeFloat(int channel, float time_us,
                             bool moreThanOneChannelActive) {
  return setConversionTime(
      channel, chopEnabled,
      calculateFilterWord(time_us, true, moreThanOneChannelActive),
      moreThanOneChannelActive);
}



float ADCBoard::setConversionTime(int channel, bool chop, byte fw,
                        bool moreThanOneChannelActive) {
  if ((fw > 127) || (chop && fw < 2) || (!chop && fw < 3)) {
    return -1;
  }

  byte chop_byte = chop ? 0b10000000 : 0b00000000;
  byte send = chop_byte | fw;

  writeRegister8(AdcRegister::channelConversionTime(channel), send);

  // could've done the calculation with user-given values but it's good to check
  float time_us = getConversionTime(channel, moreThanOneChannelActive);

  delayMicroseconds(100);

  return time_us;
}



float ADCBoard::setConversionTimeFW(int channel, int filter_word) {
  return setConversionTime(channel, true, filter_word,
                          isMoreThanOneChannelActive());
}



float ADCBoard::getConversionTime(int channel) {
  return getConversionTime(channel, isMoreThanOneChannelActive());
}



float ADCBoard::getConversionTime(int channel, bool moreThanOneChannelActive) {
  return calculateConversionTime(
      readRegister8(AdcRegister::channelConversionTime(channel)),
      moreThanOneChannelActive);
}



float ADCBoard::calculateConversionTime(byte b, bool moreThanOneChannelActive) {
  // convert to actual conversion time
  byte received_fw = b & 0b01111111;
  bool received_chop = b & 0b10000000;
  if (received_chop) {
    if (moreThanOneChannelActive) {  // FW range is 2 to 127
      return (received_fw * 128.0 + 249.0) / 6.144;
    } else {
      return (received_fw * 128.0 + 248.0) / 6.144;
    }
  } else {  // FW range is 3 to 127
    if (moreThanOneChannelActive) {
      return (received_fw * 64.0 + 206.0) / 6.144;
    } else {
      return (received_fw * 64.0 + 207.0) / 6.144;
    }
  }

  return -1;
}



byte ADCBoard::calculateFilterWord(float time_us, bool chop,
                         bool moreThanOneChannelActive) {
  double raw;
  int minimumFilterWord;
  if (chop) {
    if (moreThanOneChannelActive) {
      raw = round((time_us * 6.144 - 249.0) / 128.0);
    } else {
      raw = round((time_us * 6.144 - 248.0) / 128.0);
    }
    minimumFilterWord = 2;
  } else {
    if (moreThanOneChannelActive) {
      raw = round((time_us * 6.144 - 206.0) / 64.0);
    } else {
      raw = round((time_us * 6.144 - 207.0) / 64.0);
    }
    minimumFilterWord = 3;
  }
  int out = static_cast<int>(raw);
  if (out < minimumFilterWord) return static_cast<byte>(minimumFilterWord);
  if (out > 127) return 127;
  return static_cast<byte>(out);
}



bool ADCBoard::isMoreThanOneChannelActive() {
  const uint8_t activeChannels =
      getADCStatus() & ((1 << NUM_CHANNELS_PER_ADC_BOARD) - 1);
  return (activeChannels & (activeChannels - 1)) != 0;
}



void ADCBoard::zeroScaleSelfCalibration() {
  byte data[2];
  data[0] = writeCommand(AdcRegister::mode(0));
  data[1] = AdcRegister::kZeroScaleSelfCalMode;
  FastGpio::digitalWrite(adc_sync, true);
  commsController.transferADC(data, 2);
  waitDataReady();
}



void ADCBoard::zeroScaleChannelSystemSelfCalibration(int channel) {
  byte data[2];
  data[0] = writeCommand(AdcRegister::mode(channel));
  data[1] = AdcRegister::kChannelZeroScaleSystemCalMode;
  FastGpio::digitalWrite(adc_sync, true);
  commsController.transferADC(data, 2);
  waitDataReady();

  // wait for the data ready flag to be set then store the calibration data in flash
  int boardIndex = getBoardIndex();

  uint32_t zeroScaleCalibration = getZeroScaleCalibration(channel);

  CalibrationData calibrationData;
  readCalibrationData(calibrationData);
  calibrationData.adc_offset[NUM_CHANNELS_PER_ADC_BOARD * boardIndex + channel] = zeroScaleCalibration;
  updateCalibrationData(calibrationData);

}



void ADCBoard::fullScaleChannelSystemSelfCalibration(int channel) {
  byte data[2];
  data[0] = writeCommand(AdcRegister::mode(channel));
  data[1] = AdcRegister::kChannelFullScaleSystemCalMode;
  FastGpio::digitalWrite(adc_sync, true);
  commsController.transferADC(data, 2);
  waitDataReady();

  // wait for the data ready flag to be set then store the calibration data in flash
  int boardIndex = getBoardIndex();

  uint32_t fullScaleCalibration = getFullScaleCalibration(channel);

  CalibrationData calibrationData;
  readCalibrationData(calibrationData);
  calibrationData.adc_gain[NUM_CHANNELS_PER_ADC_BOARD * boardIndex + channel] = fullScaleCalibration;
  updateCalibrationData(calibrationData);
}
