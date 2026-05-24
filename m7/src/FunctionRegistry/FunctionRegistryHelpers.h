#pragma once

#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "FunctionRegistry/FunctionRegistryArgumentParser.h"
#include "FunctionRegistry/FunctionRegistry.h"
#include "Peripherals/OperationResult.h"

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
