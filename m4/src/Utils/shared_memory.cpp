#include "shared_memory.h"

#include <cmath>

SharedMemory* shared_memory = nullptr;

bool initSharedMemory() {
  shared_memory = reinterpret_cast<SharedMemory*>(SHARED_MEMORY_ADDRESS);
  return true;
}

void requestWorkerStop() {
  shared_memory->stop_requested = true;
  __DMB();
}

static uint32_t readUint32FromCharBuffer(CharCircularBuffer* buffer,
                                         uint32_t& index) {
  uint32_t value = 0;
  for (int i = 0; i < 4; i++) {
    value |= (static_cast<unsigned char>(buffer->buffer[index]) << (8 * i));
    index = (index + 1) % CHAR_BUFFER_SIZE; // advance by 1 byte
  }
  return value;
}

static bool charBufferReceive(CharCircularBuffer* buffer, char* data,
                              size_t& length) {
  // If buffer empty, no messages
  if (buffer->read_index == buffer->write_index) {
    length = 0;
    return false;
  }
  __DMB();

  // Read the 4-byte length
  uint32_t read_index = buffer->read_index;
  uint32_t msg_length = readUint32FromCharBuffer(buffer, read_index);
  constexpr uint32_t kMaxPayload = CHAR_BUFFER_SIZE - 5;
  if (msg_length > kMaxPayload) {
    buffer->read_index = buffer->write_index;
    length = 0;
    return false;
  }

  // Check if caller's buffer is large enough
  if (msg_length > length) {
    // Let the caller know the required size
    length = msg_length;
    return false;
  }

  // Copy the message into 'data'
  for (size_t i = 0; i < msg_length; i++) {
    data[i] = buffer->buffer[read_index];
    read_index = (read_index + 1) % CHAR_BUFFER_SIZE;
  }

  // Publish the consumed bytes only after all payload reads complete.
  __DMB();
  buffer->read_index = read_index;

  // Tell caller how many bytes we read
  length = msg_length;
  return true;
}

static bool charBufferHasMessage(CharCircularBuffer* buffer) {
  return (buffer->read_index != buffer->write_index);
}

// ---------------------------------------------------------------------------
//  Gateway command stream functions
// ---------------------------------------------------------------------------
bool sendCommandBytesToWorker(const char* data, size_t length) {
  CharCircularBuffer* buffer = &shared_memory->gateway_to_worker_char_buffer;
  uint32_t last_progress_ms = millis();

  for (size_t i = 0; i < length; i++) {
    while (((buffer->write_index + 1) % CHAR_BUFFER_SIZE) ==
           buffer->read_index) {
      if (millis() - last_progress_ms > 5000) {
        return false;
      }
      delay(1);
    }

    const uint32_t write_index = buffer->write_index;
    buffer->buffer[write_index] = data[i];
    __DMB();
    buffer->write_index = (write_index + 1) % CHAR_BUFFER_SIZE;
    last_progress_ms = millis();
  }

  return true;
}
bool receiveTextFromWorker(char* data, size_t& length) {
  return charBufferReceive(&shared_memory->worker_to_gateway_char_buffer, data,
                           length);
}
bool hasTextFromWorker() {
  return charBufferHasMessage(&shared_memory->worker_to_gateway_char_buffer);
}

static bool floatBufferReceive(FloatCircularBuffer* buffer, float* data,
                               size_t& length) {
  if (buffer->read_index == buffer->write_index) {
    length = 0;
    return false;
  }
  __DMB();

  uint32_t read_index = buffer->read_index;
  float msg_length_f = buffer->buffer[read_index];
  constexpr uint32_t kMaxPayload = FLOAT_BUFFER_SIZE - 2;
  if (!std::isfinite(static_cast<double>(msg_length_f)) || msg_length_f < 0 ||
      msg_length_f != std::trunc(msg_length_f) ||
      msg_length_f > static_cast<float>(kMaxPayload)) {
    buffer->read_index = buffer->write_index;
    length = 0;
    return false;
  }
  uint32_t msg_length = static_cast<uint32_t>(msg_length_f);
  read_index = (read_index + 1) % FLOAT_BUFFER_SIZE;

  if (msg_length > length) {
    length = msg_length;
    return false;
  }

  for (size_t i = 0; i < msg_length; i++) {
    data[i] = buffer->buffer[read_index];
    read_index = (read_index + 1) % FLOAT_BUFFER_SIZE;
  }

  __DMB();
  buffer->read_index = read_index;
  length = msg_length;
  return true;
}

bool receiveFloatResponseFromWorker(float* data, size_t& length) {
  return floatBufferReceive(&shared_memory->worker_to_gateway_float_buffer, data,
                            length);
}
bool hasFloatResponseFromWorker() {
  return shared_memory->worker_to_gateway_float_buffer.read_index !=
         shared_memory->worker_to_gateway_float_buffer.write_index;
}

static bool voltageBufferReceive(VoltageCircularBuffer* buffer,
                                 double* data, size_t& length) {
  if (buffer->read_index == buffer->write_index) {
    length = 0;
    return false;
  }
  const uint32_t read_index_start = buffer->read_index;
  const uint32_t write_index = buffer->write_index;
  __DMB();

  size_t available =
      (write_index - read_index_start + VOLTAGE_BUFFER_SIZE) %
      VOLTAGE_BUFFER_SIZE;
  size_t to_read = (length < available) ? length : available;

  uint32_t read_index = read_index_start;
  for (size_t i = 0; i < to_read; ++i) {
    data[i] = buffer->buffer[read_index];
    read_index = (read_index + 1) % VOLTAGE_BUFFER_SIZE;
  }

  __DMB();
  buffer->read_index = read_index;
  length = to_read;
  return true;
}

bool receiveVoltageFrameFromWorker(double* data, size_t& length) {
  return voltageBufferReceive(&shared_memory->worker_to_gateway_voltage_buffer,
                              data, length);
}
bool hasVoltageFrameFromWorker() {
  return shared_memory->worker_to_gateway_voltage_buffer.read_index !=
         shared_memory->worker_to_gateway_voltage_buffer.write_index;
}
