#include "shared_memory.h"
#include <vector>

SharedMemory* shared_memory = nullptr;

bool initSharedMemory() {
  shared_memory = reinterpret_cast<SharedMemory*>(SHARED_MEMORY_ADDRESS);
  return true;
}

void requestWorkerStop() { shared_memory->stop_requested = true; }

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

  // Read the 4-byte length
  uint32_t read_index = buffer->read_index;
  uint32_t msg_length = readUint32FromCharBuffer(buffer, read_index);

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

  // Update read_index
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
  struct CharReassemblyState {
    bool assembling = false;
    uint16_t next_seq = 0;
    uint32_t expected_total = 0;
    std::vector<char> assembled;
  };
  static CharReassemblyState state;

  const size_t output_capacity = length;
  static char frame[CHAR_BUFFER_SIZE];
  size_t frame_length = sizeof(frame);
  if (!charBufferReceive(&shared_memory->worker_to_gateway_char_buffer, frame,
                         frame_length)) {
    return false;
  }

  auto read_le16 = [](const uint8_t* p) -> uint16_t {
    return static_cast<uint16_t>(p[0]) |
           (static_cast<uint16_t>(p[1]) << 8);
  };
  auto read_le32 = [](const uint8_t* p) -> uint32_t {
    return static_cast<uint32_t>(p[0]) |
           (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) |
           (static_cast<uint32_t>(p[3]) << 24);
  };

  const uint8_t* u8 = reinterpret_cast<const uint8_t*>(frame);
  if (frame_length == 0) {
    length = 0;
    return false;
  }

  const uint8_t frame_type = u8[0];
  if (frame_type == CHAR_FRAME_TYPE_NORMAL) {
    const size_t payload_length = frame_length - 1;
    if (payload_length > output_capacity) {
      length = 0;
      return false;
    }

    state = CharReassemblyState{};
    memcpy(data, frame + 1, payload_length);
    length = payload_length;
    return true;
  }

  if (frame_type != CHAR_FRAME_TYPE_FRAGMENT ||
      frame_length < CHAR_FRAGMENT_HEADER_SIZE) {
    state = CharReassemblyState{};
    length = 0;
    return false;
  }

  const uint8_t flags = u8[1];
  const uint8_t version = u8[2];
  const uint16_t seq = read_le16(&u8[3]);
  const uint32_t total_len = read_le32(&u8[5]);
  const bool is_first = (flags & 0x01) != 0;
  const bool is_last = (flags & 0x02) != 0;

  if (version != CHAR_FRAGMENT_VERSION || total_len == 0) {
    state = CharReassemblyState{};
    length = 0;
    return false;
  }

  if (is_first) {
    state.assembling = true;
    state.next_seq = 0;
    state.expected_total = total_len;
    state.assembled.clear();
    state.assembled.reserve(total_len);
  }

  if (!state.assembling || seq != state.next_seq ||
      total_len != state.expected_total) {
    state = CharReassemblyState{};
    length = 0;
    return false;
  }

  const size_t payload_len =
      (frame_length > CHAR_FRAGMENT_HEADER_SIZE)
          ? (frame_length - CHAR_FRAGMENT_HEADER_SIZE)
          : 0;
  const size_t already = state.assembled.size();
  if (payload_len > 0 && already < state.expected_total) {
    const size_t remaining = state.expected_total - already;
    const size_t to_append =
        (payload_len < remaining) ? payload_len : remaining;
    state.assembled.insert(state.assembled.end(),
                           frame + CHAR_FRAGMENT_HEADER_SIZE,
                           frame + CHAR_FRAGMENT_HEADER_SIZE + to_append);
  }

  state.next_seq++;

  if (!(is_last && state.assembled.size() >= state.expected_total)) {
    length = 0;
    return false;
  }

  if (state.expected_total > output_capacity) {
    state = CharReassemblyState{};
    length = 0;
    return false;
  }

  memcpy(data, state.assembled.data(), state.expected_total);
  length = state.expected_total;
  state = CharReassemblyState{};
  return true;
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

  uint32_t read_index = buffer->read_index;
  float msg_length_f = buffer->buffer[read_index];
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

  size_t available =
      (buffer->write_index - buffer->read_index + VOLTAGE_BUFFER_SIZE) %
      VOLTAGE_BUFFER_SIZE;
  size_t to_read = (length < available) ? length : available;

  for (size_t i = 0; i < to_read; ++i) {
    data[i] = buffer->buffer[buffer->read_index];
    buffer->read_index = (buffer->read_index + 1) % VOLTAGE_BUFFER_SIZE;
  }

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
