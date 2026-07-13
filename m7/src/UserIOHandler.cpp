#include "UserIOHandler.h"

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>

#include "FunctionRegistry/FunctionRegistry.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "shared_memory.h"

#ifndef FIRMWARE_VERSION_HASH
#define FIRMWARE_VERSION_HASH UNKNOWN
#endif

#ifndef STRINGIZE
#define STRINGIZE(x) #x
#define STRINGIZE_VALUE_OF(x) STRINGIZE(x)
#endif

namespace {
constexpr size_t COMMAND_NAME_CAPACITY = 64;
constexpr size_t NUMBER_TOKEN_CAPACITY = 64;
constexpr size_t COMMAND_READ_CHUNK_SIZE = 128;
// Dynamic-list commands temporarily hold both the parsed float stream and
// typed list storage; AWG commands also pre-encode three bytes per value.
// Keep the limit below the M7 heap's worst-case capacity-growth threshold.
constexpr size_t MAX_COMMAND_ARGS = 16000;

struct StreamingCommandParser {
  char command[COMMAND_NAME_CAPACITY];
  char token[NUMBER_TOKEN_CAPACITY];
  size_t commandLength;
  size_t tokenLength;
  bool parsingArgs;
  const char* error;
  std::vector<float> args;
};

StreamingCommandParser parser = {};

void resetParser() {
  parser.commandLength = 0;
  parser.tokenLength = 0;
  parser.parsingArgs = false;
  parser.error = nullptr;
  parser.args.clear();
}

void sendText(const char* message) {
  sendTextToGateway(message, strlen(message) + 1);
}

bool finishToken() {
  parser.token[parser.tokenLength] = '\0';

  char* tokenStart = parser.token;
  while (*tokenStart == ' ' || *tokenStart == '\t') {
    tokenStart++;
  }

  char* tokenEnd = parser.token + parser.tokenLength;
  while (tokenEnd > tokenStart &&
         (tokenEnd[-1] == ' ' || tokenEnd[-1] == '\t')) {
    tokenEnd--;
  }
  *tokenEnd = '\0';

  if (*tokenStart == '\0') {
    parser.error = "FAILURE: Invalid arguments";
    return false;
  }

  char* endPtr = nullptr;
  const double value = strtod(tokenStart, &endPtr);
  if (endPtr == tokenStart || !std::isfinite(value)) {
    parser.error = "FAILURE: Invalid arguments";
    return false;
  }
  while (*endPtr == ' ' || *endPtr == '\t') {
    endPtr++;
  }
  if (*endPtr != '\0') {
    parser.error = "FAILURE: Invalid arguments";
    return false;
  }

  if (parser.args.size() >= MAX_COMMAND_ARGS) {
    parser.error = "FAILURE: Command has too many arguments";
    return false;
  }
  const float parsedValue = static_cast<float>(value);
  if (!std::isfinite(parsedValue)) {
    parser.error = "FAILURE: Numeric argument out of range";
    return false;
  }
  parser.args.push_back(parsedValue);
  parser.tokenLength = 0;
  return true;
}

void executeParsedCommand() {
  parser.command[parser.commandLength] = '\0';
  String command(parser.command);
  command.trim();
  if (command.length() == 0) {
    resetParser();
    return;
  }

  OperationResult result = OperationResult::Failure("Something went wrong!");
  FunctionRegistry::ExecuteResult executeResult =
      FunctionRegistry::execute(command, parser.args, result);

  switch (executeResult) {
    case FunctionRegistry::ExecuteResult::Success:
      if (result.hasMessage()) {
        const String message = result.getMessage();
        sendTextToGateway(message.c_str(), message.length() + 1);
      }
      break;
    case FunctionRegistry::ExecuteResult::ArgumentError:
      sendText("FAILURE: Argument error");
      break;
    case FunctionRegistry::ExecuteResult::FunctionNotFound:
      sendText("FAILURE: Function not found");
      break;
  }

  resetParser();
}

void finishCommand() {
  if (parser.error) {
    sendText(parser.error);
    resetParser();
    return;
  }
  if (parser.parsingArgs && parser.tokenLength > 0 && !finishToken()) {
    sendText(parser.error);
    resetParser();
    return;
  }
  if (parser.error) {
    sendText(parser.error);
    resetParser();
    return;
  }
  executeParsedCommand();
}

void consumeCommandByte(char c) {
  if (c == '\r') {
    return;
  }
  if (c == '\n') {
    finishCommand();
    return;
  }
  if (parser.error) {
    return;
  }
  if (!parser.parsingArgs) {
    if (c == ',') {
      parser.parsingArgs = true;
      return;
    }
    if (parser.commandLength >= sizeof(parser.command) - 1) {
      parser.error = "FAILURE: Command name too long";
      return;
    }
    parser.command[parser.commandLength++] = c;
    return;
  }
  if (c == ',') {
    finishToken();
    return;
  }
  if (parser.tokenLength >= sizeof(parser.token) - 1) {
    parser.error = "FAILURE: Numeric argument too long";
    return;
  }
  parser.token[parser.tokenLength++] = c;
}
}  // namespace

namespace UserIOHandler {

__attribute__((section(".serial_number")))
const char serial_number[29] = "__SERIAL_NUMBER__DA_2025_ABC";

OperationResult getFirmwareVersion() {
#ifdef FIRMWARE_VERSION_TAG
  return OperationResult::Success(STRINGIZE_VALUE_OF(FIRMWARE_VERSION_TAG));
#endif
  return OperationResult::Success(String("Commit Hash: ") +
                                  STRINGIZE_VALUE_OF(FIRMWARE_VERSION_HASH));
}
COMMAND("GET_FIRMWARE_VERSION", getFirmwareVersion)

OperationResult nop() {
  return OperationResult::Success("NOP");
}
COMMAND("NOP", nop)

OperationResult getEnvironment() {
  return OperationResult::Success("GATEKEEPER");
}
COMMAND("GET_ENVIRONMENT", getEnvironment)

OperationResult id() {
  return OperationResult::Success("GateKeeper 1.0");
}
COMMAND("*IDN?", id)

OperationResult rdy() {
  return OperationResult::Success("READY");
}
COMMAND("*RDY?", rdy)

OperationResult serialNumber() {
  return OperationResult::Success(serial_number + 17);
}
COMMAND("SERIAL_NUMBER", serialNumber)

void handleUserIO() {
  char bytes[COMMAND_READ_CHUNK_SIZE];
  while (hasCommandFromGateway()) {
    const size_t count =
        receiveCommandBytesFromGateway(bytes, sizeof(bytes));
    if (count == 0) {
      break;
    }
    for (size_t i = 0; i < count; i++) {
      consumeCommandByte(bytes[i]);
    }
  }
}

}  // namespace UserIOHandler
