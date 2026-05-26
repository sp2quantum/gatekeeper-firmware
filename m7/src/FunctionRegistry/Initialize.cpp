#include "FunctionRegistry/FunctionRegistry.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Utils/OperationResult.h"

namespace {

OperationResult initialize() {
  InitRegistry::runAll();
  return OperationResult::Success("INITIALIZATION COMPLETE");
}
COMMAND("INITIALIZE", initialize)
COMMAND("INIT", initialize)
COMMAND("INNIT", initialize)

}  // namespace
