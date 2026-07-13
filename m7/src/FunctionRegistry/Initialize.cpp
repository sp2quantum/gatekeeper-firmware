#include "FunctionRegistry/FunctionRegistry.h"
#include "FunctionRegistry/FunctionRegistryHelpers.h"
#include "Utils/OperationResult.h"

namespace {

OperationResult initialize() {
  OperationResult result = InitRegistry::runAll();
  if (!result.isSuccess()) return result;
  return OperationResult::Success("INITIALIZATION COMPLETE");
}
COMMAND("INITIALIZE", initialize)
COMMAND("INIT", initialize)
COMMAND("INNIT", initialize)

}  // namespace
