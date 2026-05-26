#pragma once

#include <Arduino.h>

#include <tuple>
#include <type_traits>
#include <vector>

#include "Utils/OperationResult.h"

namespace FunctionRegistryParsing {

class ArgReader {
 public:
  explicit ArgReader(const std::vector<float>& args) : args_(args), index_(0) {}

  size_t index() const { return index_; }
  size_t remaining() const {
    return index_ <= args_.size() ? args_.size() - index_ : 0u;
  }

  template <typename T>
  typename std::enable_if<std::is_arithmetic<T>::value,
                          OperationResult>::type
  read(T& value) {
    if (remaining() == 0) {
      return OperationResult::Failure("Argument count mismatch");
    }
    value = static_cast<T>(args_[index_++]);
    return OperationResult::Success();
  }

 private:
  const std::vector<float>& args_;
  size_t index_;
};

constexpr size_t NoListMultiplier = static_cast<size_t>(-1);

// A command argument list whose length is controlled by an earlier scalar
// argument. For example:
//   foo(int numDacs, List<int, 0>& channels,
//       List<float, 0>& voltages)
// parses numDacs channel values followed by numDacs voltage values.
//
// For matrix-like payloads, pass one optional multiplier argument index:
// List<float, 0, 2> reads arg0 * arg2 values.
template <typename T, size_t LengthArgIndex,
          size_t MultiplierArgIndex = NoListMultiplier>
class List {
  static_assert(std::is_arithmetic<T>::value,
                "List only supports primitive arithmetic values");

 public:
  using value_type = T;
  static constexpr size_t length_arg_index = LengthArgIndex;
  static constexpr size_t multiplier_arg_index = MultiplierArgIndex;
  static constexpr bool has_multiplier =
      MultiplierArgIndex != NoListMultiplier;

  OperationResult read(ArgReader& reader, size_t length) {
    if (length > reader.remaining()) {
      return OperationResult::Failure("Argument count mismatch");
    }
    values_.resize(length);
    for (size_t i = 0; i < length; ++i) {
      OperationResult result = reader.read(values_[i]);
      if (!result.isSuccess()) {
        return result;
      }
    }
    return OperationResult::Success();
  }

  T* data() { return values_.data(); }
  const T* data() const { return values_.data(); }
  size_t size() const { return values_.size(); }
  T& operator[](size_t index) { return values_[index]; }
  const T& operator[](size_t index) const { return values_[index]; }
  const std::vector<T>& values() const { return values_; }
  std::vector<T>& values() { return values_; }

 private:
  std::vector<T> values_;
};

template <typename T>
struct IsList : std::false_type {};

template <typename T, size_t LengthArgIndex, size_t MultiplierArgIndex>
struct IsList<List<T, LengthArgIndex, MultiplierArgIndex> >
    : std::true_type {};

inline OperationResult multiplyListLength(size_t& length,
                                          size_t factor,
                                          size_t maxLength) {
  if (factor != 0 && length > maxLength / factor) {
    return OperationResult::Failure("Argument count mismatch");
  }
  length *= factor;
  return OperationResult::Success();
}

template <typename Tuple, size_t Index>
OperationResult tupleLengthValue(const Tuple& parsed, size_t& value) {
  const int signedValue = static_cast<int>(std::get<Index>(parsed));
  if (signedValue < 0) {
    return OperationResult::Failure("Invalid list length");
  }
  value = static_cast<size_t>(signedValue);
  return OperationResult::Success();
}

template <typename Tuple, size_t Index>
OperationResult multiplyListLengthByTupleValue(const Tuple& parsed,
                                               size_t maxLength,
                                               size_t& length) {
  size_t multiplier = 0;
  OperationResult result = tupleLengthValue<Tuple, Index>(parsed, multiplier);
  if (!result.isSuccess()) {
    return result;
  }
  return multiplyListLength(length, multiplier, maxLength);
}

template <typename Tuple, typename ListType, bool HasMultiplier>
struct ListMultiplier;

template <typename Tuple, typename ListType>
struct ListMultiplier<Tuple, ListType, false> {
  static OperationResult apply(const Tuple& parsed, size_t maxLength,
                               size_t& length) {
    (void)parsed;
    (void)maxLength;
    (void)length;
    return OperationResult::Success();
  }
};

template <typename Tuple, typename ListType>
struct ListMultiplier<Tuple, ListType, true> {
  static OperationResult apply(const Tuple& parsed, size_t maxLength,
                               size_t& length) {
    return multiplyListLengthByTupleValue<Tuple,
                                          ListType::multiplier_arg_index>(
        parsed, maxLength, length);
  }
};

template <typename Tuple, typename ListType>
OperationResult listLengthFor(const Tuple& parsed, size_t maxLength,
                              size_t& length) {
  OperationResult result =
      tupleLengthValue<Tuple, ListType::length_arg_index>(parsed, length);
  if (!result.isSuccess()) {
    return result;
  }
  return ListMultiplier<Tuple, ListType, ListType::has_multiplier>::apply(
      parsed, maxLength, length);
}

template <typename Tuple, size_t Index>
struct HasAnyList {
  using Current = typename std::tuple_element<Index, Tuple>::type;
  static constexpr bool value =
      IsList<Current>::value || HasAnyList<Tuple, Index - 1>::value;
};

template <typename Tuple>
struct HasAnyList<Tuple, 0> {
  using Current = typename std::tuple_element<0, Tuple>::type;
  static constexpr bool value = IsList<Current>::value;
};

template <typename Tuple, size_t Count>
struct HasAnyListInTupleImpl {
  static constexpr bool value = HasAnyList<Tuple, Count - 1>::value;
};

template <typename Tuple>
struct HasAnyListInTupleImpl<Tuple, 0> {
  static constexpr bool value = false;
};

template <typename Tuple>
struct HasAnyListInTuple {
  static constexpr bool value =
      HasAnyListInTupleImpl<Tuple, std::tuple_size<Tuple>::value>::value;
};

template <typename Tuple, size_t Index>
typename std::enable_if<
    !IsList<typename std::tuple_element<Index, Tuple>::type>::value,
    OperationResult>::type
readTupleArgument(ArgReader& reader, Tuple& parsed) {
  return reader.read(std::get<Index>(parsed));
}

template <typename Tuple, size_t Index>
typename std::enable_if<
    IsList<typename std::tuple_element<Index, Tuple>::type>::value,
    OperationResult>::type
readTupleArgument(ArgReader& reader, Tuple& parsed) {
  using ListType = typename std::tuple_element<Index, Tuple>::type;
  static_assert(ListType::length_arg_index < Index,
                "List length argument must appear before the List argument");
  static_assert(!ListType::has_multiplier ||
                    ListType::multiplier_arg_index < Index,
                "List multiplier argument must appear before the List "
                "argument");
  size_t length = 0;
  OperationResult lengthResult =
      listLengthFor<Tuple, ListType>(parsed, reader.remaining(), length);
  if (!lengthResult.isSuccess()) {
    return lengthResult;
  }
  return std::get<Index>(parsed).read(reader, length);
}

template <typename Tuple, size_t Index>
typename std::enable_if<Index == std::tuple_size<Tuple>::value,
                        OperationResult>::type
readTupleArguments(ArgReader&, Tuple&) {
  return OperationResult::Success();
}

template <typename Tuple, size_t Index>
typename std::enable_if<Index < std::tuple_size<Tuple>::value,
                        OperationResult>::type
readTupleArguments(ArgReader& reader, Tuple& parsed) {
  OperationResult result = readTupleArgument<Tuple, Index>(reader, parsed);
  if (!result.isSuccess()) {
    return result;
  }
  return readTupleArguments<Tuple, Index + 1>(reader, parsed);
}

template <typename Tuple>
OperationResult parseTuple(const std::vector<float>& args, Tuple& parsed) {
  ArgReader reader(args);
  OperationResult result = readTupleArguments<Tuple, 0>(reader, parsed);
  if (!result.isSuccess()) {
    return result;
  }
  if (reader.remaining() != 0) {
    return OperationResult::Failure("Argument count mismatch");
  }
  return OperationResult::Success();
}

}  // namespace FunctionRegistryParsing
