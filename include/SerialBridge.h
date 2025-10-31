#pragma once
#include <Arduino.h>
#include "PIDController.h"
#include "Types.h"
#include "Config.h"

struct SerialCmdResult {
  bool ok;
  String msg;
};

class SerialBridge {
public:
  SerialBridge();
  void begin(unsigned long baud);
  // call frequently in loop to parse incoming lines
  // returns true if a new PID was requested (and fills paramsOut)
  bool poll(PIDParams &paramsOut);
  // when asked, prints current PID values
  void printHelp();
  void printCurrent(PIDController &pid);
  // If host requested GET PID, main can call this to consume that request (returns true once)
  bool consumeGetPidRequest();
private:
  String _buffer;
  volatile bool _getPidRequested = false;
};
