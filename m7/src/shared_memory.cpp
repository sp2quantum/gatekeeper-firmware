#include "shared_memory.h"
#include <vector>

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

void requestWorkerStop() { shared_memory->stop_requested = true; }
void clearWorkerStopRequest() { shared_memory->stop_requested = false; }
bool isWorkerStopRequested() { return shared_memory->stop_requested; }

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

  for (size_t i = 0; i < length; ++i) {
    buffer->buffer[buffer->write_index] = data[i];
    buffer->write_index = (buffer->write_index + 1) % VOLTAGE_BUFFER_SIZE;
  }

  return true;
}

// ---------------------------------------------------------------------------
//  Worker command/response functions
// ---------------------------------------------------------------------------
bool sendTextToGateway(const char* data, size_t length) {
  // The underlying ring buffer is only CHAR_BUFFER_SIZE bytes and stores
  // an additional 4-byte length prefix per frame (see charBufferSend()).
  // That means the *true* maximum encoded frame size is (CHAR_BUFFER_SIZE - 5).
  //
  // Large commands (e.g. AWG with many points) can exceed this, so we fragment
  // them into multiple frames and let the M4 side reassemble.

  constexpr size_t kCharFrameMaxPayload =
      (CHAR_BUFFER_SIZE > 5) ? (CHAR_BUFFER_SIZE - 5) : 0;
  constexpr size_t kNormalFrameOverhead = 1;

  auto write_le16 = [](uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  };
  auto write_le32 = [](uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v & 0xFF);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
  };

  auto send_blocking = [&](const char* frame, size_t frame_len) -> bool {
    // Block until there's space; this prevents dropping long commands.
    // If the M4 stops draining the ring buffer, this would block here.
    uint32_t start_ms = millis();
    while (!charBufferSend(&shared_memory->worker_to_gateway_char_buffer, frame,
                           frame_len)) {
      // Avoid hard-deadlocking the M7 if the M4 is busy/crashed and not draining.
      if (millis() - start_ms > 5000) {
        return false;
      }
      delay(1);
    }
    return true;
  };

  // Fast path: fits in a single ring-buffer frame.
  if (kCharFrameMaxPayload <= kNormalFrameOverhead) {
    return false;
  }
  if (length <= (kCharFrameMaxPayload - kNormalFrameOverhead)) {
    static uint8_t frame[kCharFrameMaxPayload];
    frame[0] = CHAR_FRAME_TYPE_NORMAL;
    memcpy(&frame[1], data, length);
    return send_blocking(reinterpret_cast<const char*>(frame), length + 1);
  }

  // Fragmented path: fixed header + chunked payload.
  if (kCharFrameMaxPayload <= CHAR_FRAGMENT_HEADER_SIZE) {
    // Should never happen with current sizes.
    return false;
  }

  const size_t max_chunk = kCharFrameMaxPayload - CHAR_FRAGMENT_HEADER_SIZE;
  uint16_t seq = 0;
  size_t offset = 0;

  while (offset < length) {
    const size_t remaining = length - offset;
    const size_t chunk = (remaining < max_chunk) ? remaining : max_chunk;

    const bool is_first = (offset == 0);
    const bool is_last = (offset + chunk == length);

    static uint8_t frame[kCharFrameMaxPayload];
    frame[0] = CHAR_FRAME_TYPE_FRAGMENT;
    // Flags
    frame[1] = static_cast<uint8_t>((is_first ? 0x01 : 0x00) |
                                    (is_last ? 0x02 : 0x00));
    // Version
    frame[2] = CHAR_FRAGMENT_VERSION;
    // Sequence
    write_le16(&frame[3], seq);
    // Total length of the full (reassembled) message
    write_le32(&frame[5], static_cast<uint32_t>(length));
    // Payload bytes
    memcpy(&frame[CHAR_FRAGMENT_HEADER_SIZE], data + offset, chunk);

    if (!send_blocking(reinterpret_cast<const char*>(frame),
                       CHAR_FRAGMENT_HEADER_SIZE + chunk)) {
      return false;
    }

    offset += chunk;
    seq++;
  }

  return true;
}
size_t receiveCommandBytesFromGateway(char* data, size_t capacity) {
  CharCircularBuffer* buffer = &shared_memory->gateway_to_worker_char_buffer;
  size_t count = 0;
  uint32_t read_index = buffer->read_index;

  while (count < capacity && read_index != buffer->write_index) {
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
