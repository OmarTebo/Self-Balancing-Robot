#pragma once
#include <Arduino.h>
#include "Config.h"

class IMU {
public:
  IMU();
  bool begin();
  // update internal sensor fusion if needed; dt optional (some drivers do their own timing)
  void update(float dt);
  // returns angles in degrees
  float getPitch(); // -180..180
  float getRoll();
  float getYaw();

  // optionally expose last update time for health check
  unsigned long lastUpdateMillis();

  // I2C recovery helper (public so caller can invoke)
  static void i2cBusRecover(int sdaPin, int sclPin);

private:
  float pitch, roll, yaw;
  unsigned long lastMillis;
  // for MPU fallback
  void beginMPU();
  void beginBNO();
};
