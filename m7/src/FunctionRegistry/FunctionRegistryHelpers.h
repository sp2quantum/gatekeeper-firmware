#pragma once

#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "Calibration/Calibration.h"
#include "FunctionRegistry/FunctionRegistryArgumentParser.h"
#include "FunctionRegistry/FunctionRegistry.h"
#include "Utils/OperationResult.h"

namespace FunctionRegistryDetail {

template <typename T>
struct FunctionTraits;

template <typename ReturnType, typename... Args>
struct FunctionTraits<ReturnType (*)(Args...)> {
  using return_type = ReturnType;
  using args_tuple = std::tuple<Args...>;
  static constexpr size_t arity = sizeof...(Args);
};

template <typename ClassType, typename ReturnType, typename... Args>
struct FunctionTraits<ReturnType (ClassType::*)(Args...)> {
  using return_type = ReturnType;
  using args_tuple = std::tuple<Args...>;
  static constexpr size_t arity = sizeof...(Args);
};

template <typename ClassType, typename ReturnType, typename... Args>
struct FunctionTraits<ReturnType (ClassType::*)(Args...) const> {
  using return_type = ReturnType;
  using args_tuple = std::tuple<Args...>;
  static constexpr size_t arity = sizeof...(Args);
};

template <typename Functor>
struct FunctionTraits : public FunctionTraits<decltype(&Functor::operator())> {
};

template <typename Tuple, size_t... Is>
auto decayTupleHelper(std::index_sequence<Is...>)
    -> std::tuple<typename std::decay<
        typename std::tuple_element<Is, Tuple>::type>::type...>;

template <typename Tuple>
struct DecayTuple {
  using type = decltype(decayTupleHelper<Tuple>(
      std::make_index_sequence<std::tuple_size<Tuple>::value>{}));
};

template <typename Function, typename Tuple, size_t... Is>
auto callFunctionHelper(Function function, Tuple& parsed,
                        std::index_sequence<Is...>)
    -> decltype(function(std::get<Is>(parsed)...)) {
  return function(std::get<Is>(parsed)...);
}

}  // namespace FunctionRegistryDetail

template <typename Function>
void registerFunction(Function function, const String& commandName) {
  using Traits = FunctionRegistryDetail::FunctionTraits<Function>;
  using ArgsTuple = typename Traits::args_tuple;
  using ParsedTuple =
      typename FunctionRegistryDetail::DecayTuple<ArgsTuple>::type;
  constexpr bool hasDynamicArguments =
      FunctionRegistryParsing::HasAnyListInTuple<ParsedTuple>::value;
  constexpr int argCount =
      hasDynamicArguments ? -1 : static_cast<int>(Traits::arity);

  auto wrapper = [function](const std::vector<float>& args) -> OperationResult {
    ParsedTuple parsed;
    OperationResult parseResult =
        FunctionRegistryParsing::parseTuple(args, parsed);
    if (!parseResult.isSuccess()) {
      return parseResult;
    }

    return FunctionRegistryDetail::callFunctionHelper<Function, ParsedTuple>(
        function, parsed, std::make_index_sequence<Traits::arity>{});
  };

  FunctionRegistry::registerFunction(commandName, wrapper, argCount);
}

#define _REG_CAT2(a, b) a##b
#define _REG_CAT(a, b) _REG_CAT2(a, b)

#define COMMAND(name, func) \
  namespace { auto _REG_CAT(_cmd_, __COUNTER__) = \
      (registerFunction(func, name), 0); }

#define ON_SETUP(func) \
  _ON_SETUP_WITH_PRIORITY(SetupRegistry::Priority::Peripheral, func)

#define ON_SETUP_PLATFORM(func) \
  _ON_SETUP_WITH_PRIORITY(SetupRegistry::Priority::Platform, func)

#define ON_SETUP_CALIBRATION(func) \
  _ON_SETUP_WITH_PRIORITY(SetupRegistry::Priority::Calibration, func)

#define _ON_SETUP_WITH_PRIORITY(priority, func) \
  namespace { auto _REG_CAT(_setup_, __COUNTER__) = \
      (SetupRegistry::registerCallback(func, priority), 0); }

#define ON_INITIALIZE(func) \
  namespace { auto _REG_CAT(_init_, __COUNTER__) = \
      (InitRegistry::registerCallback(func), 0); }
