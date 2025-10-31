#pragma once
#include <Arduino.h>
#include "MPU6050_Processing.h"

struct Angles { float pitch, roll, yaw; };

class MPU6050_Fusion {
public:
  virtual ~MPU6050_Fusion() {}
  virtual void begin(float updateHz = MPU_KALMAN_DEFAULT_UPDATE_HZ) = 0;
  virtual void reset() = 0;
  virtual void update(const ProcessedSample &s, float dt) = 0;
  virtual void getAngles(Angles &out) = 0;
};
