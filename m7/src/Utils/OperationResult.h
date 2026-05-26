#pragma once

#include <Arduino.h>

class OperationResult {
 public:
  enum class Status { Success, Failure };

  static OperationResult Success(const String& message) {
    return OperationResult(Status::Success, message);
  }

  static OperationResult Success() {
    return OperationResult(Status::Success, "");
  }

  static OperationResult Failure(const String& message) {
    return OperationResult(Status::Failure, String("FAILURE: ") + message);
  }

  static OperationResult Failure() {
    return OperationResult(Status::Failure, "FAILURE: Something went wrong!");
  }

  bool isSuccess() const { return status_ == Status::Success; }
  bool hasMessage() const { return message_.length() > 0; }
  const String& getMessage() const { return message_; }

 private:
  Status status_;
  String message_;

  OperationResult(Status status, const String& message)
      : status_(status), message_(message) {}
};
