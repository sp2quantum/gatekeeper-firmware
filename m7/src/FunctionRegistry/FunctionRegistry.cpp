#include "FunctionRegistry/FunctionRegistry.h"

#include <algorithm>

namespace FunctionRegistry {
namespace {

struct FunctionEntry {
  String name;
  CommandCallback func;
  int argCount;
};

std::vector<FunctionEntry>& getFunctions() {
  static std::vector<FunctionEntry> functions;
  return functions;
}

}  // namespace

void registerFunction(const String& name, CommandCallback func, int argCount) {
  String upperName = name;
  upperName.toUpperCase();
  getFunctions().push_back({upperName, func, argCount});
}

ExecuteResult execute(const String& name, const std::vector<float>& args,
                      OperationResult& result) {
  auto& functions = getFunctions();
  String upperName = name;
  upperName.toUpperCase();
  if (upperName == "PRINT_FUNCTIONS") {
    String message = "Available functions, args: \nPRINT_FUNCTIONS, 0\n";
    for (const auto& entry : functions) {
      message += String(entry.name + ", " + entry.argCount + "\n");
    }
    message = message.substring(0, message.length() - 1);
    result = OperationResult::Success(message);
    return ExecuteResult::Success;
  }
  for (const auto& entry : functions) {
    if (entry.name == upperName) {
      if (entry.argCount >= 0 &&
          static_cast<int>(entry.argCount) != static_cast<int>(args.size())) {
        result = OperationResult::Failure("Argument count mismatch");
        return ExecuteResult::ArgumentError;
      }
      result = entry.func(args);
      return ExecuteResult::Success;
    }
  }
  result = OperationResult::Failure("Function not found");
  return ExecuteResult::FunctionNotFound;
}

}  // namespace FunctionRegistry

namespace SetupRegistry {
struct SetupEntry {
  int priority;
  SetupCallback callback;
};

static std::vector<SetupEntry>& getCallbacks() {
  static std::vector<SetupEntry> callbacks;
  return callbacks;
}

void registerCallback(SetupCallback cb, int priority) {
  getCallbacks().push_back({priority, cb});
}

void runAll() {
  auto& callbacks = getCallbacks();
  std::stable_sort(callbacks.begin(), callbacks.end(),
                   [](const SetupEntry& a, const SetupEntry& b) {
                     return a.priority < b.priority;
                   });
  for (const auto& entry : callbacks) entry.callback();
}
}

namespace InitRegistry {
static std::vector<InitCallback>& getCallbacks() {
  static std::vector<InitCallback> callbacks;
  return callbacks;
}
void registerCallback(InitCallback cb) { getCallbacks().push_back(cb); }
void runAll() {
  for (auto cb : getCallbacks()) cb();
}
}
