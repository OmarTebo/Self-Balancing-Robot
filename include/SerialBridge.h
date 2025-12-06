#pragma once
#include <Arduino.h>
#include "PIDController.h"
#include "Types.h"
#include "Config.h"

// Forward declarations
class IMU;

// Forward declarations
class BotController;

class SerialBridge {
public:
  SerialBridge();
  void begin(unsigned long baud);
  // call frequently in loop to parse incoming lines
  // returns true if a new PID was requested (and fills paramsOut)
  // imu: optional pointer to IMU for calibration commands (nullptr to skip calibration)
  // controller: optional pointer to BotController for test mode commands (nullptr to skip)
  bool poll(PIDParams &paramsOut, IMU *imu = nullptr, BotController *controller = nullptr);
  // when asked, prints current PID values
  void printHelp();
  void printCurrent(PIDController &pid);
  // If host requested GET PID, main can call this to consume that request (returns true once)
  bool consumeGetPidRequest();
  
private:
  String _buffer;
  volatile bool _getPidRequested = false;
};
