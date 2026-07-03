#pragma once

#include <Arduino.h>

#include "Config.h"
#include "Utils/OperationResult.h"

struct BoardUsage {
  uint8_t numBoards = 0;
  uint8_t idx[NUM_ADC_BOARDS] = {};
};

class RampContext {
 public:
  RampContext();
  ~RampContext();

  RampContext(const RampContext&) = delete;
  RampContext& operator=(const RampContext&) = delete;

  OperationResult beginDacAndAdc(int* adcChannels, int numAdcChannels);
  void beginDacOnly();

  AdcBoardMask adcMask() const { return adcMask_; }
  bool stopped() const;

  OperationResult finish(OperationResult rampResult,
                         bool checkTiming = true,
                         bool checkAdcMissteps = true);

 private:
  int* adcChannels_ = nullptr;
  int numAdcChannels_ = 0;
  BoardUsage boardUsage_;
  AdcBoardMask adcMask_ = 0;
  bool begun_ = false;
  bool finished_ = false;
  bool hasAdc_ = false;

  void setupAdcHardware(int* adcChannels, int numAdcChannels);
  void cleanup();
};
