#include "shared_memory.h"

SharedMemory* shared_memory = nullptr;

bool initSharedMemory() {
  shared_memory = reinterpret_cast<SharedMemory*>(SHARED_MEMORY_ADDRESS);

  shared_memory->gateway_to_worker_char_buffer.read_index = 0;
  shared_memory->gateway_to_worker_char_buffer.write_index = 0;
  shared_memory->worker_to_gateway_char_buffer.read_index = 0;
  shared_memory->worker_to_gateway_char_buffer.write_index = 0;
  shared_memory->worker_to_gateway_float_buffer.read_index = 0;
  shared_memory->worker_to_gateway_float_buffer.write_index = 0;
  shared_memory->worker_to_gateway_voltage_buffer.read_index = 0;
  shared_memory->worker_to_gateway_voltage_buffer.write_index = 0;

  shared_memory->stop_requested = false;

  shared_memory->calibration_updated = false;
  shared_memory->worker_dma_ready = false;
  shared_memory->calibration_ready = false;

  return true;
}

void publishCalibrationData(const CalibrationData& data) {
  memcpy(&shared_memory->calibration_data, &data, sizeof(CalibrationData));
  __DMB();
  shared_memory->calibration_ready = true;
}

void updateCalibrationData(const CalibrationData& data) {
  memcpy(&shared_memory->calibration_data, &data, sizeof(CalibrationData));
  __DMB();
  shared_memory->calibration_updated = true;
}

void readCalibrationData(CalibrationData& data) {
  memcpy(&data, &shared_memory->calibration_data, sizeof(CalibrationData));
  __DMB();
}

bool isCalibrationDataUpdated() {
  __DMB();
  return shared_memory->calibration_updated;
}

void clearCalibrationDataUpdated() {
  shared_memory->calibration_updated = false;
}

bool isWorkerDmaReady() {
  __DMB();
  return shared_memory->worker_dma_ready;
}

bool isCalibrationDataReady() {
  __DMB();
  return shared_memory->calibration_ready;
}

void requestWorkerStop() {
  shared_memory->stop_requested = true;
  __DMB();
}
void clearWorkerStopRequest() {
  shared_memory->stop_requested = false;
  __DMB();
}
bool isWorkerStopRequested() {
  __DMB();
  return shared_memory->stop_requested;
}

// ---------------------------------------------------------------------------
//  Utilities to safely write/read a 32-bit length into the char buffer
//  (avoids unaligned 32-bit access and handles wraparound cleanly).
// ---------------------------------------------------------------------------
static void writeUint32ToCharBuffer(CharCircularBuffer* buffer,
                                    uint32_t& index,
                                    uint32_t value) {
  for (int i = 0; i < 4; i++) {
    buffer->buffer[index] = static_cast<char>((value >> (8 * i)) & 0xFF);
    index = (index + 1) % CHAR_BUFFER_SIZE; // advance by 1 byte
  }
}

// ---------------------------------------------------------------------------
//  Char buffer operations
// ---------------------------------------------------------------------------
static bool charBufferSend(CharCircularBuffer* buffer, const char* data,
                           size_t length) {
  constexpr size_t kMaxCharPayload =
      (CHAR_BUFFER_SIZE > 5) ? (CHAR_BUFFER_SIZE - 5) : 0;
  if (length > kMaxCharPayload) return false;

  // We need to store 4 bytes for 'length' + the actual payload
  uint32_t available_space =
      (buffer->read_index - buffer->write_index - 1 + CHAR_BUFFER_SIZE) %
      CHAR_BUFFER_SIZE;

  if (length + 4 > available_space) {
    return false;
  }

  uint32_t write_index = buffer->write_index;
  writeUint32ToCharBuffer(buffer, write_index, static_cast<uint32_t>(length));

  for (size_t i = 0; i < length; i++) {
    buffer->buffer[write_index] = data[i];
    write_index = (write_index + 1) % CHAR_BUFFER_SIZE;
  }

  // Ensure the payload is visible to the M4 before publishing the index.
  __DMB();
  buffer->write_index = write_index;
  return true;
}

static bool charBufferHasMessage(CharCircularBuffer* buffer) {
  return (buffer->read_index != buffer->write_index);
}

static bool floatBufferSend(FloatCircularBuffer* buffer, const float* data,
                            size_t length) {
  constexpr size_t kMaxFloatPayload =
      (FLOAT_BUFFER_SIZE > 2) ? (FLOAT_BUFFER_SIZE - 2) : 0;
  if (length > kMaxFloatPayload) return false;

  uint32_t available_space =
      (buffer->read_index - buffer->write_index - 1 + FLOAT_BUFFER_SIZE) %
      FLOAT_BUFFER_SIZE;

  if (length + 1 > available_space) return false;

  uint32_t write_index = buffer->write_index;
  buffer->buffer[write_index] = static_cast<float>(length);
  write_index = (write_index + 1) % FLOAT_BUFFER_SIZE;

  for (size_t i = 0; i < length; ++i) {
    buffer->buffer[write_index] = data[i];
    write_index = (write_index + 1) % FLOAT_BUFFER_SIZE;
  }

  // Ensure the payload is visible to the M4 before publishing the index.
  __DMB();
  buffer->write_index = write_index;
  return true;
}

// ---------------------------------------------------------------------------
//  Voltage buffer operations
// ---------------------------------------------------------------------------
static bool voltageBufferSend(VoltageCircularBuffer* buffer,
                              const double* data, size_t length) {
  if (length > MAX_MESSAGE_SIZE) return false;

  uint32_t available_space =
      (buffer->read_index - buffer->write_index - 1 + VOLTAGE_BUFFER_SIZE) %
      VOLTAGE_BUFFER_SIZE;

  if (length > available_space) return false;

  uint32_t write_index = buffer->write_index;
  for (size_t i = 0; i < length; ++i) {
    buffer->buffer[write_index] = data[i];
    write_index = (write_index + 1) % VOLTAGE_BUFFER_SIZE;
  }

  // Ensure the payload is visible to the M4 before publishing the index.
  __DMB();
  buffer->write_index = write_index;
  return true;
}

// ---------------------------------------------------------------------------
//  Worker command/response functions
// ---------------------------------------------------------------------------
bool sendTextToGateway(const char* data, size_t length) {
  const uint32_t start_ms = millis();
  while (!charBufferSend(&shared_memory->worker_to_gateway_char_buffer, data,
                         length)) {
    if (millis() - start_ms > 5000) return false;
    delay(1);
  }
  return true;
}
size_t receiveCommandBytesFromGateway(char* data, size_t capacity) {
  CharCircularBuffer* buffer = &shared_memory->gateway_to_worker_char_buffer;
  size_t count = 0;
  uint32_t read_index = buffer->read_index;
  const uint32_t write_index = buffer->write_index;
  __DMB();

  while (count < capacity && read_index != write_index) {
    data[count++] = buffer->buffer[read_index];
    read_index = (read_index + 1) % CHAR_BUFFER_SIZE;
  }

  if (count > 0) {
    __DMB();
    buffer->read_index = read_index;
  }

  return count;
}
bool hasCommandFromGateway() {
  return charBufferHasMessage(&shared_memory->gateway_to_worker_char_buffer);
}

bool sendFloatResponseToGateway(const float* data, size_t length) {
  return floatBufferSend(&shared_memory->worker_to_gateway_float_buffer, data,
                         length);
}

bool sendVoltageFrameToGateway(const double* data, size_t length) {
  return voltageBufferSend(&shared_memory->worker_to_gateway_voltage_buffer, data,
                           length);
}
