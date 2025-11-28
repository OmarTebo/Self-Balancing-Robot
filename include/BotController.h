#pragma once
#include "PIDController.h"
#include "MotorDriver.h"
#include "Config.h"
#include "IMU.h"
#include "Types.h"
#include "BLEHandler.h"
#include <MD_MAX72xx.h>
#include <SPI.h>

class BotController {
public:
  BotController();
  void begin();
  // call every control tick
  void update(float dt);
  void requestPidParams(const PIDParams &p); // request atomic apply
  // print current pid to Serial
  void printCurrentPid();

  MotorDriver leftMotor;  // was pitchMotor
  MotorDriver rightMotor; // was rollMotor

  // setpoint in degrees
  float targetPitch = 0.0f;
  float targetRoll = 0.0f;

private:
  void loadStoredPid();
  void savePidToStorage(float kp, float ki, float kd);
  void updateDisplay();
  void displayBitmap(uint8_t device, const uint8_t *bitmap);
  BLEHandler ble;
  portMUX_TYPE mux;
  volatile bool pendingPid;
  PIDParams pendingParams;
  float stepsPerDegree;
  PIDController pitchPid;
  void applyPendingPid();
  IMU imu;
  MD_MAX72XX displayMatrix;
  static const uint8_t PROGMEM smileyBitmap[8];
  static const uint8_t PROGMEM sadBitmap[8];
  unsigned long lastDisplayUpdateMs;
};


