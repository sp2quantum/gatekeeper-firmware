#include <Arduino.h>

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "mbed/drivers/usb/include/usb/USBCDC.h"

#include "Utils/shared_memory.h"

typedef union {
  float floatingPoint;
  byte binary[4];
} binaryFloat;

namespace {

constexpr uint8_t kUsbStringDescriptor = 0x03;
volatile bool bootloader_touch_requested = false;

constexpr uint8_t kManufacturerDescriptor[] = {
    24, kUsbStringDescriptor, 's', 0, 'p', 0, '2', 0, ' ', 0, 'q', 0,
    'u', 0, 'a', 0, 'n', 0, 't', 0, 'u', 0, 'm', 0};

constexpr uint8_t kProductDescriptor[] = {
    22, kUsbStringDescriptor, 'G', 0, 'a', 0, 't', 0, 'e', 0, 'K', 0,
    'e', 0, 'e', 0, 'p', 0, 'e', 0, 'r', 0};

static_assert(sizeof(kManufacturerDescriptor) == kManufacturerDescriptor[0],
              "USB manufacturer string descriptor length is wrong.");
static_assert(sizeof(kProductDescriptor) == kProductDescriptor[0],
              "USB product string descriptor length is wrong.");

constexpr size_t kSerialMarkerLength = 17;
constexpr size_t kDacSerialFieldLength = 12;

__attribute__((section(".serial_number"), used))
static const char kDacSerialNumber[kSerialMarkerLength + kDacSerialFieldLength] =
    "__SERIAL_NUMBER__DA_2025_ABC";

static const uint8_t* dacSerialDescriptor() {
  static uint8_t descriptor[2 + kDacSerialFieldLength * 2];
  static bool initialized = false;

  if (!initialized) {
    const char* serial = kDacSerialNumber + kSerialMarkerLength;
    size_t serial_length = 0;
    while (serial_length < kDacSerialFieldLength &&
           serial[serial_length] != '\0') {
      serial_length++;
    }

    descriptor[0] = 2 + serial_length * 2;
    descriptor[1] = kUsbStringDescriptor;

    for (size_t i = 0; i < serial_length; ++i) {
      descriptor[2 + i * 2] = static_cast<uint8_t>(serial[i]);
      descriptor[3 + i * 2] = 0;
    }

    initialized = true;
  }

  return descriptor;
}

static void resetToBootloaderDfu() {
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  RTC_HandleTypeDef rtc_handle = {};
  rtc_handle.Instance = RTC;
  HAL_RTCEx_BKUPWrite(&rtc_handle, RTC_BKP_DR0, 0xDF59);
  NVIC_SystemReset();
}

class GateKeeperUSBCDC : public USBCDC {
 public:
  using USBCDC::USBCDC;

 protected:
  const uint8_t* string_imanufacturer_desc() override {
    return kManufacturerDescriptor;
  }

  const uint8_t* string_iproduct_desc() override {
    return kProductDescriptor;
  }

  const uint8_t* string_iserial_desc() override {
    return dacSerialDescriptor();
  }

  void line_coding_changed(int baud, int bits, int parity, int stop) override {
    (void)bits;
    (void)parity;
    (void)stop;
    if (baud == 1200) {
      bootloader_touch_requested = true;
    }
  }
};

}  // namespace

static GateKeeperUSBCDC usb_cdc(false, 0x2341, 0x0266, 0x0101);

static void usbWrite(const uint8_t* data, uint32_t size) {
  if (!usb_cdc.ready()) {
    return;
  }
  usb_cdc.send(const_cast<uint8_t*>(data), size);
}

static void usbPrint(const char* data) {
  usbWrite(reinterpret_cast<const uint8_t*>(data), strlen(data));
}

static char lowerAscii(char c) {
  if (c >= 'A' && c <= 'Z') {
    return c + ('a' - 'A');
  }
  return c;
}

static bool normalizedStopMatch(const char* data, size_t length,
                                bool allowPrefix) {
  size_t start = 0;
  while (start < length && (data[start] == ' ' || data[start] == '\t')) {
    start++;
  }

  size_t end = length;
  while (end > start &&
         (data[end - 1] == ' ' || data[end - 1] == '\t' ||
          data[end - 1] == '\r' || data[end - 1] == '\n')) {
    end--;
  }

  const char stop[] = "stop";
  const size_t normalizedLength = end - start;
  if (normalizedLength > sizeof(stop) - 1) {
    return false;
  }
  for (size_t i = 0; i < normalizedLength; i++) {
    if (lowerAscii(data[start + i]) != stop[i]) {
      return false;
    }
  }
  return allowPrefix || normalizedLength == sizeof(stop) - 1;
}

static bool streamHostBytesToWorker(const char* data, size_t length) {
  if (length == 0) {
    return true;
  }
  if (!sendCommandBytesToWorker(data, length)) {
    usbPrint("FAILURE: Gateway command stream full\r\n");
    return false;
  }
  return true;
}

static void processHostByte(char c) {
  constexpr size_t kStopDetectionBufferSize = 16;
  static char pending[kStopDetectionBufferSize];
  static size_t pendingLength = 0;
  static bool streamingCommand = false;
  static bool droppingUntilNewline = false;

  if (c == '\r') {
    return;
  }

  if (droppingUntilNewline) {
    if (c == '\n') {
      droppingUntilNewline = false;
    }
    return;
  }

  if (streamingCommand) {
    if (!streamHostBytesToWorker(&c, 1)) {
      streamingCommand = false;
      droppingUntilNewline = true;
      return;
    }
    if (c == '\n') {
      streamingCommand = false;
    }
    return;
  }

  if (pendingLength < sizeof(pending)) {
    pending[pendingLength++] = c;
  } else {
    if (!streamHostBytesToWorker(pending, pendingLength) ||
        !streamHostBytesToWorker(&c, 1)) {
      pendingLength = 0;
      droppingUntilNewline = true;
      return;
    }
    pendingLength = 0;
    streamingCommand = c != '\n';
    return;
  }

  if (c == '\n') {
    if (normalizedStopMatch(pending, pendingLength, false)) {
      requestWorkerStop();
    } else if (!streamHostBytesToWorker(pending, pendingLength)) {
      droppingUntilNewline = true;
    }
    pendingLength = 0;
    return;
  }

  if (!normalizedStopMatch(pending, pendingLength, true)) {
    if (!streamHostBytesToWorker(pending, pendingLength)) {
      pendingLength = 0;
      droppingUntilNewline = true;
      return;
    }
    pendingLength = 0;
    streamingCommand = true;
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  initSharedMemory();

  usb_cdc.init();
  usb_cdc.connect();
}

void loop() {
  if (bootloader_touch_requested) {
    usb_cdc.disconnect();
    delay(250);
    resetToBootloaderDfu();
  }

  uint8_t input[64];
  uint32_t actual = 0;
  usb_cdc.receive_nb(input, sizeof(input), &actual);
  for (uint32_t i = 0; i < actual; ++i) {
    processHostByte(static_cast<char>(input[i]));
  }

  if (hasTextFromWorker()) {
    static char response[4096];
    size_t size = sizeof(response);
    if (receiveTextFromWorker(response, size)) {
      if (size > 0) {
        size--;
      }
      usbWrite(reinterpret_cast<const uint8_t*>(response), size);
      usbPrint("\r\n");
    }
  }

  if (hasFloatResponseFromWorker()) {
    static float response[FLOAT_BUFFER_SIZE];
    size_t size = FLOAT_BUFFER_SIZE;
    if (receiveFloatResponseFromWorker(response, size)) {
      for (size_t i = 0; i < size; ++i) {
        char value[24];
        snprintf(value, sizeof(value), "%.8f ", response[i]);
        usbPrint(value);
      }
      usbPrint("\r\n");
    }
  }

  if (hasVoltageFrameFromWorker()) {
    static double response[VOLTAGE_BUFFER_SIZE];
    size_t size = VOLTAGE_BUFFER_SIZE;
    if (receiveVoltageFrameFromWorker(response, size)) {
      for (size_t i = 0; i < size; ++i) {
        binaryFloat send;
        send.floatingPoint = static_cast<float>(response[i]);
        usbWrite(send.binary, 4);
      }
    }
  }
}
