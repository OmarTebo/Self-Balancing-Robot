#pragma once
#include "PIDController.h"
#include "MotorDriver.h"
#include "Config.h"
#include "IMU.h"
#include "Types.h"
#include "BLEHandler.h"
#include <Arduino.h>

class BotController {
public:
  BotController();
  void begin();
  // call every control tick
  void update(float dt);
  void requestPidParams(const PIDParams &p); // request atomic apply
  // print current pid to Serial
  void printCurrentPid();

  MotorDriver pitchMotor;
  MotorDriver rollMotor;
  // setpoint in degrees
  float targetPitch = 0.0f;
  float targetRoll  = 0.0f;

private:
  BLEHandler ble;
  portMUX_TYPE mux;
  volatile bool pendingPid;
  PIDParams pendingParams;
  float stepsPerDegree;
  PIDController pitchPid;
  void applyPendingPid();
  IMU imu;
};
