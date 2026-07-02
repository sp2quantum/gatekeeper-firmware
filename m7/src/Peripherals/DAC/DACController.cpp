#include "Peripherals/DAC/DACController.h"

#include <array>
#include <cmath>
#include <utility>

#include "Calibration/Calibration.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "PeripheralCommsController.h"
#include "Peripherals/ADC/ADCController.h"
#include "Utils/FastGpio.h"
#include "Utils/TimingUtil.h"
#include "shared_memory.h"

namespace {

constexpr byte kWriteAndUpdateDacCommand = 0x10;
constexpr byte kReadbackCommand = 0x90;
constexpr byte kWriteControlRegisterCommand = 0x20;
constexpr byte kUnclampDacFromGround = 0x02;
constexpr int kPositiveFullScaleCode = 524287;
constexpr int kNegativeFullScaleCode = 524288;
constexpr int kTwosComplementSpan = 1048576;
constexpr int kDataByteMask = 0xFF;
constexpr int kDataHighNibbleMask = 0x0F;

bool isFinite(float value) {
  return std::isfinite(static_cast<double>(value));
}

bool isValidCalibrationPair(float offset, float gain) {
  return isFinite(gain) && gain >= 0.5f && gain <= 1.5f &&
         isFinite(offset) && offset >= -1.0f && offset <= 1.0f;
}

template <size_t... Indices>
std::array<PeripheralCommsController, sizeof...(Indices)> makeDacComms(
    std::index_sequence<Indices...>) {
  return {{PeripheralCommsController(dac_cs_pins[Indices])...}};
}

auto comms = makeDacComms(std::make_index_sequence<NUM_DAC_CHANNELS>{});

float gain_error[NUM_DAC_CHANNELS];
float gain_error_inverse[NUM_DAC_CHANNELS];
float offset_error[NUM_DAC_CHANNELS];
float voltage_upper_bound[NUM_DAC_CHANNELS];
float voltage_lower_bound[NUM_DAC_CHANNELS];
float full_scale_val[NUM_DAC_CHANNELS];

struct DacCalibrationData {
  float gain[NUM_DAC_CALIBRATION_CHANNELS];
  float offset[NUM_DAC_CALIBRATION_CHANNELS];
};

const uint32_t kDacCalibrationId = CalibrationRegistry::sectionId("DAC");

DacCalibrationData* dacCalibration(CalibrationData& data) {
  return static_cast<DacCalibrationData*>(CalibrationRegistry::getSection(
      data, kDacCalibrationId, sizeof(DacCalibrationData)));
}

void setDacCalibrationDefaults(void* section) {
  auto& data = *static_cast<DacCalibrationData*>(section);
  for (int i = 0; i < NUM_DAC_CALIBRATION_CHANNELS; i++) {
    data.gain[i] = 1.0f;
    data.offset[i] = 0.0f;
  }
}

bool validateDacCalibration(const void* section) {
  const auto& data = *static_cast<const DacCalibrationData*>(section);
  for (int i = 0; i < NUM_DAC_CALIBRATION_CHANNELS; i++) {
    if (!isValidCalibrationPair(data.offset[i], data.gain[i])) {
      return false;
    }
  }
  return true;
}
CALIBRATION_SECTION("DAC", DacCalibrationData, setDacCalibrationDefaults,
                    validateDacCalibration)

void initChannelDefaults() {
  for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
    gain_error[i] = 1.0f;
    gain_error_inverse[i] = 1.0f;
    offset_error[i] = 0.0f;
    voltage_upper_bound[i] = 10.0f;
    voltage_lower_bound[i] = -10.0f;
    full_scale_val[i] = 10.0f;
  }
}

void voltageToBytes(int ch, float v, byte* DB1, byte* DB2, byte* DB3) {
  int decimal;
  if (v >= 0) {
    decimal = v * kPositiveFullScaleCode / full_scale_val[ch];
  } else {
    decimal =
        v * kNegativeFullScaleCode / full_scale_val[ch] + kTwosComplementSpan;
  }
  *DB1 = static_cast<byte>((decimal >> 16) | kWriteAndUpdateDacCommand);
  *DB2 = static_cast<byte>((decimal >> 8) & kDataByteMask);
  *DB3 = static_cast<byte>(decimal & kDataByteMask);
}

int threeByteToInt(byte DB1, byte DB2, byte DB3) {
  return (((DB1 & kDataHighNibbleMask) << 8 | DB2) << 8) | DB3;
}

float threeByteToVoltage(int ch, byte DB1, byte DB2, byte DB3) {
  const int decimal = threeByteToInt(DB1, DB2, DB3);
  if (decimal <= kPositiveFullScaleCode) {
    return decimal * full_scale_val[ch] / kPositiveFullScaleCode;
  }
  return -(kTwosComplementSpan - decimal) * full_scale_val[ch] /
         kNegativeFullScaleCode;
}

bool writeAndLatchPacket(int ch, const byte packet[3]) {
  byte buf[3] = {packet[0], packet[1], packet[2]};
  if (!comms[ch].transferDAC(buf, 3)) return false;
  FastGpio::pulseLowHigh(ldac);
  return true;
}

struct RampParams {
  int channel;
  double v0;
  double vf;
  double stepSize;
};

OperationResult setUpperLimit(int channel, float limit) {
  if (!DACController::isChannelIndexValid(channel))
    return OperationResult::Failure("Invalid channel index " + String(channel));
  if (!isFinite(limit))
    return OperationResult::Failure("Invalid voltage limit");
  if (limit < DACLimits::lower_voltage_limit[channel])
    return OperationResult::Failure("Upper limit must be >= lower limit");
  DACLimits::upper_voltage_limit[channel] = limit;
  return OperationResult::Success("CH" + String(channel) +
                                  " UPPER LIMIT SET TO " + String(limit, 6) +
                                  " V");
}
COMMAND("SET_UPPER_LIMIT", setUpperLimit)

OperationResult setLowerLimit(int channel, float limit) {
  if (!DACController::isChannelIndexValid(channel))
    return OperationResult::Failure("Invalid channel index " + String(channel));
  if (!isFinite(limit))
    return OperationResult::Failure("Invalid voltage limit");
  if (limit > DACLimits::upper_voltage_limit[channel])
    return OperationResult::Failure("Lower limit must be <= upper limit");
  DACLimits::lower_voltage_limit[channel] = limit;
  return OperationResult::Success("CH" + String(channel) +
                                  " LOWER LIMIT SET TO " + String(limit, 6) +
                                  " V");
}
COMMAND("SET_LOWER_LIMIT", setLowerLimit)

OperationResult getUpperLimit(int channel) {
  if (!DACController::isChannelIndexValid(channel))
    return OperationResult::Failure("Invalid channel index " + String(channel));
  return OperationResult::Success(
      String(DACLimits::upper_voltage_limit[channel], 6));
}
COMMAND("GET_UPPER_LIMIT", getUpperLimit)

OperationResult getLowerLimit(int channel) {
  if (!DACController::isChannelIndexValid(channel))
    return OperationResult::Failure("Invalid channel index " + String(channel));
  return OperationResult::Success(
      String(DACLimits::lower_voltage_limit[channel], 6));
}
COMMAND("GET_LOWER_LIMIT", getLowerLimit)

OperationResult toggleLdacTest() {
  DACController::toggleLdac();
  return OperationResult::Success("LDAC TOGGLED");
}
COMMAND("TOGGLE_LDAC", toggleLdacTest)

OperationResult setOSG(int channel_index, float off, float g) {
  if (!DACController::isChannelIndexValid(channel_index))
    return OperationResult::Failure("Invalid channel index " +
                                    String(channel_index));
  if (!isValidCalibrationPair(off, g))
    return OperationResult::Failure("Invalid calibration offset/gain");
  DACController::setCalibration(channel_index, off, g);
  return OperationResult::Success("OSG SET FOR DAC " + String(channel_index));
}
COMMAND("SET_OSG", setOSG)

OperationResult sendCode(int channel, int code) {
  if (!DACController::isChannelIndexValid(channel))
    return OperationResult::Failure("Invalid channel index " + String(channel));
  if (code < 0 || code > 1048575)
    return OperationResult::Failure("CODE OVERRANGE (0-1048575)");
  byte packet[3];
  packet[0] =
      static_cast<byte>((code >> 16) | kWriteAndUpdateDacCommand);
  packet[1] = static_cast<byte>((code >> 8) & kDataByteMask);
  packet[2] = static_cast<byte>(code & kDataByteMask);
  writeAndLatchPacket(channel, packet);
  return OperationResult::Success("DAC " + String(channel) +
                                  " CODE UPDATED TO " + String(code));
}
COMMAND("SET_DAC_CODE", sendCode)

OperationResult setFullScale(int channel, float fs) {
  if (!DACController::isChannelIndexValid(channel))
    return OperationResult::Failure("Invalid channel index " + String(channel));
  if (!isFinite(fs) || fs <= 0.0f)
    return OperationResult::Failure("Invalid full scale");
  full_scale_val[channel] = fs;
  voltage_upper_bound[channel] =
      fs * gain_error[channel] + offset_error[channel];
  voltage_lower_bound[channel] =
      -fs * gain_error[channel] + offset_error[channel];
  return OperationResult::Success("FULL_SCALE_UPDATED");
}
COMMAND("FULL_SCALE", setFullScale)

OperationResult getFullScale(int channel) {
  if (!DACController::isChannelIndexValid(channel))
    return OperationResult::Failure("Invalid channel index " + String(channel));
  return OperationResult::Success(String(full_scale_val[channel], 6));
}
COMMAND("GET_FULL_SCALE", getFullScale)

OperationResult inquiryOSG() {
  for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
    sendFloatResponseToGateway(&offset_error[i], 1);
  }
  for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
    sendFloatResponseToGateway(&gain_error[i], 1);
  }
  return OperationResult::Success("");
}
COMMAND("INQUIRY_OSG", inquiryOSG)

OperationResult runAutoRampN(int numDacs, int numSteps,
                             unsigned long settlingTime_us,
                             const int* dacChannels, const float* dacV0s,
                             const float* dacVfs) {
  if (numDacs < 1 || numDacs > NUM_DAC_CHANNELS || numSteps < 1 ||
      settlingTime_us < 1) {
    return OperationResult::Failure("Invalid ramp parameters.");
  }

  RampParams rampParams[NUM_DAC_CHANNELS] = {};
  int rampParamsCount = 0;

  for (int i = 0; i < numDacs; i++) {
    const int ch = dacChannels[i];
    const double v0 = dacV0s[i];
    const double vf = dacVfs[i];

    if (!DACController::isChannelIndexValid(ch))
      return OperationResult::Failure("Invalid channel index " + String(ch));

    if (v0 < voltage_lower_bound[ch] || v0 > voltage_upper_bound[ch] ||
        vf < voltage_lower_bound[ch] || vf > voltage_upper_bound[ch]) {
      return OperationResult::Failure("Voltage out of bounds for DAC " +
                                      String(ch));
    }

    double stepSize = numSteps > 1 ? (vf - v0) / (numSteps - 1) : 0.0;
    rampParams[rampParamsCount++] = {ch, v0, vf, stepSize};
  }

  int currentStep = 0;
  TimingUtil::setupTimerOnlyDac(settlingTime_us);

  double currentVoltages[NUM_DAC_CHANNELS] = {};
  for (int i = 0; i < rampParamsCount; i++) {
    currentVoltages[i] = rampParams[i].v0;
  }

  while (currentStep < numSteps) {
    if (isWorkerStopRequested()) break;
    if (TimingUtil::consumeDacFlag()) {
      for (int i = 0; i < rampParamsCount; i++) {
        const auto& param = rampParams[i];
        DACController::setVoltage(param.channel, currentVoltages[i]);
        currentVoltages[i] += param.stepSize;
      }
      currentStep++;
    }
  }

  TimingUtil::disableDacInterrupt();

  String output = "RAMPING ";
  for (int i = 0; i < rampParamsCount; i++) {
    const auto& param = rampParams[i];
    output += "DAC " + String(param.channel) + " FROM " + String(param.v0) +
              " TO " + String(param.vf) + "; ";
  }
  output += "IN " + String(numSteps) + " STEPS";

  if (isWorkerStopRequested()) {
    clearWorkerStopRequest();
    return OperationResult::Failure("RAMPING_STOPPED");
  }

  return OperationResult::Success(output);
}

OperationResult autoRamp1(int dacChannel, float v0, float vf, int numSteps,
                          u_long settlingTime_us) {
  const int ch[1] = {dacChannel};
  const float v0s[1] = {v0};
  const float vfs[1] = {vf};
  return runAutoRampN(1, numSteps, settlingTime_us, ch, v0s, vfs);
}
COMMAND("RAMP1", autoRamp1)

OperationResult autoRamp2(int dacChannel1, int dacChannel2, float vi1,
                          float vi2, float vf1, float vf2, int numSteps,
                          u_long settlingTime_us) {
  const int ch[2] = {dacChannel1, dacChannel2};
  const float v0s[2] = {vi1, vi2};
  const float vfs[2] = {vf1, vf2};
  return runAutoRampN(2, numSteps, settlingTime_us, ch, v0s, vfs);
}
COMMAND("RAMP2", autoRamp2)

OperationResult autoRampN(int numDacs, int numSteps,
                          unsigned long settlingTime_us,
                          List<int, 0>& dacChannels, List<float, 0>& dacV0s,
                          List<float, 0>& dacVfs) {
  return runAutoRampN(numDacs, numSteps, settlingTime_us, dacChannels.data(),
                      dacV0s.data(), dacVfs.data());
}
COMMAND("RAMP_N", autoRampN)

}  // namespace

float DACLimits::upper_voltage_limit[NUM_DAC_CHANNELS];
float DACLimits::lower_voltage_limit[NUM_DAC_CHANNELS];
bool DACLimits::limits_initialized = false;

namespace DACController {

void setup() {
  DACLimits::initializeLimits();
  initChannelDefaults();
  pinMode(ldac, OUTPUT);
  FastGpio::digitalWrite(ldac, true);
  for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
    pinMode(dac_cs_pins[i], OUTPUT);
    FastGpio::digitalWrite(dac_cs_pins[i], true);
  }
}
ON_SETUP(setup)

void initialize() {
  for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
    byte buf[3] = {kWriteControlRegisterCommand, 0, kUnclampDacFromGround};
    comms[i].transferDAC(buf, 3);
    setVoltage(i, 0.0);
  }
}
ON_INITIALIZE(initialize)

bool isChannelIndexValid(int ch) {
  return ch >= 0 && ch < NUM_DAC_CHANNELS;
}

OperationResult setVoltage(int ch, float voltage) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index " + String(ch));
  const float lowerBound = getLowerBound(ch);
  const float upperBound = getUpperBound(ch);
  if (voltage < lowerBound || voltage > upperBound)
    return OperationResult::Failure(
        "Voltage out of bounds for DAC " + String(ch) + " (" +
        String(voltage) + " V must be between " +
        String(lowerBound) + " and " + String(upperBound) + " V)");

  byte packet[3];
  voltageToBytes(ch, voltage * gain_error_inverse[ch] - offset_error[ch],
                 &packet[0], &packet[1], &packet[2]);
  if (!writeAndLatchPacket(ch, packet)) {
    return OperationResult::Failure("Voltage out of bounds for DAC " +
                                    String(ch));
  }
  float v = gain_error[ch] *
            (threeByteToVoltage(ch, packet[0], packet[1], packet[2]) +
             offset_error[ch]);
  return OperationResult::Success("DAC " + String(ch) + " UPDATED TO " +
                                  String(v, 6) + " V");
}
COMMAND("SET", setVoltage)

bool setVoltageNoTransactionNoLdac(int ch, float voltage) {
  if (!isChannelIndexValid(ch)) return false;
  if (voltage < voltage_lower_bound[ch] || voltage > voltage_upper_bound[ch])
    return false;
  byte buf[3];
  if (!encodeVoltagePacket(ch, voltage, buf)) return false;
  return writeVoltagePacketNoLdac(ch, buf);
}

bool encodeVoltagePacket(int ch, float voltage, byte packet[3]) {
  if (!isChannelIndexValid(ch)) return false;
  if (voltage < voltage_lower_bound[ch] || voltage > voltage_upper_bound[ch])
    return false;
  if (voltage > DACLimits::upper_voltage_limit[ch] ||
      voltage < DACLimits::lower_voltage_limit[ch])
    return false;
  voltageToBytes(ch, voltage * gain_error_inverse[ch] - offset_error[ch],
                 &packet[0], &packet[1], &packet[2]);
  return true;
}

bool writeVoltagePacketNoLdac(int ch, const byte packet[3]) {
  if (!isChannelIndexValid(ch)) return false;
  byte buf[3] = {packet[0], packet[1], packet[2]};
  return comms[ch].transferDACNoTransaction(buf, 3);
}

int getCsPin(int ch) {
  if (!isChannelIndexValid(ch)) return NC;
  return dac_cs_pins[ch];
}

void toggleLdac() { FastGpio::pulseLowHigh(ldac); }

OperationResult getVoltage(int ch) {
  if (!isChannelIndexValid(ch))
    return OperationResult::Failure("Invalid channel index " + String(ch));
  byte tx[3] = {kReadbackCommand, 0, 0};
  byte rx[3] = {0, 0, 0};
  if (!comms[ch].transferDACRead(tx, 3) || !comms[ch].transferDACRead(rx, 3))
    return OperationResult::Failure("DAC read failed");
  float v = gain_error[ch] *
            (threeByteToVoltage(ch, rx[0], rx[1], rx[2]) + offset_error[ch]);
  return OperationResult::Success(String(v, 6));
}
COMMAND("GET_DAC", getVoltage)

void applySavedCalibration() {
  while (!isCalibrationDataReady()) {
    delay(1);
  }

  CalibrationData calibration_data;
  readCalibrationData(calibration_data);
  const DacCalibrationData* dacData = dacCalibration(calibration_data);
  if (!dacData) return;
  for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
    applyCalibration(i, dacData->offset[i], dacData->gain[i]);
  }
}
ON_SETUP_CALIBRATION(applySavedCalibration)

void applyCalibration(int ch, float offset, float gain) {
  if (!isChannelIndexValid(ch)) return;
  if (!std::isfinite(static_cast<double>(offset))) offset = 0.0f;
  if (!std::isfinite(static_cast<double>(gain)) ||
      std::fabs(static_cast<double>(gain)) < 1e-6)
    gain = 1.0f;
  offset_error[ch] = offset;
  gain_error[ch] = gain;
  gain_error_inverse[ch] = 1.0f / gain;
  voltage_upper_bound[ch] =
      full_scale_val[ch] * gain_error[ch] + offset_error[ch];
  voltage_lower_bound[ch] =
      -full_scale_val[ch] * gain_error[ch] + offset_error[ch];
}

void setCalibration(int ch, float offset, float gain) {
  if (!isChannelIndexValid(ch)) return;
  applyCalibration(ch, offset, gain);
  CalibrationData data;
  readCalibrationData(data);
  DacCalibrationData* dacData = dacCalibration(data);
  if (!dacData) return;
  for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
    dacData->offset[i] = offset_error[i];
    dacData->gain[i] = gain_error[i];
  }
  updateCalibrationData(data);
}

OperationResult dacChannelCalibration() {
  CalibrationData calibrationData;
  readCalibrationData(calibrationData);
  DacCalibrationData* dacData = dacCalibration(calibrationData);
  if (!dacData) {
    return OperationResult::Failure("DAC calibration section is missing");
  }

  for (int i = 0; i < NUM_DAC_CHANNELS; i++) {
    initialize();
    applyCalibration(i, 0, 1);
    setVoltage(i, 0);
    delay(1);
    const float offsetError = ADCController::getVoltage(i);
    applyCalibration(i, offsetError, 1);
    const float voltSet = 9.0f;
    setVoltage(i, voltSet);
    delay(1);
    const float gainError =
        (ADCController::getVoltage(i) - offsetError) / voltSet;
    applyCalibration(i, offsetError, gainError);
    setVoltage(i, 0);
    dacData->offset[i] = offsetError;
    dacData->gain[i] = gainError;
  }
  updateCalibrationData(calibrationData);
  return OperationResult::Success("CALIBRATION_FINISHED");
}
COMMAND("DAC_CH_CAL", dacChannelCalibration)

float getLowerBound(int ch) {
  if (!isChannelIndexValid(ch)) return -1;
  return max(voltage_lower_bound[ch], DACLimits::lower_voltage_limit[ch]);
}

float getUpperBound(int ch) {
  if (!isChannelIndexValid(ch)) return -1;
  return min(voltage_upper_bound[ch], DACLimits::upper_voltage_limit[ch]);
}

}  // namespace DACController
