#pragma once

#include <Arduino.h>

#include <functional>
#include <vector>

#include "Utils/OperationResult.h"

namespace FunctionRegistry {

enum class ExecuteResult { Success, FunctionNotFound, ArgumentError };

using CommandCallback =
    std::function<OperationResult(const std::vector<float>&)>;

void registerFunction(const String& name, CommandCallback func, int argCount);

ExecuteResult execute(const String& name, const std::vector<float>& args,
                      OperationResult& result);

}  // namespace FunctionRegistry

namespace SetupRegistry {
using SetupCallback = void (*)();

namespace Priority {
// Lower priority callbacks run first. Keep platform/shared infrastructure
// before peripheral setup, and saved calibration restore after hardware setup.
constexpr int Platform = 0;
constexpr int Peripheral = 100;
constexpr int Calibration = 200;
}  // namespace Priority

void registerCallback(SetupCallback cb,
                      int priority = Priority::Peripheral);
void runAll();
}

namespace InitRegistry {
using InitCallback = void (*)();
void registerCallback(InitCallback cb);
void runAll();
}
