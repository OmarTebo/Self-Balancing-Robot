#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "MPU6050_Driver.h"
#include "MPU6050_Processing.h"
#include "MPU6050_Fusion.h"

class MPU6050 {
public:
  MPU6050(TwoWire &w = Wire, uint8_t addr = MPU6050_DEFAULT_ADDR);
  ~MPU6050();

  bool begin(bool doInit = true);
  void attachFilter(MPU6050_Fusion *filter);
  void calibrateGyro(uint16_t samples = 512);
  void update(float dt = -1.0f);
  float getPitch() const;
  float getRoll()  const;
  float getYaw()   const;
  void getAngles(Angles &out) const;
  void setAccelScale(float lsb_per_g);
  void setGyroScale(float lsb_per_degps);

private:
  MPU6050_Driver _drv;
  MPU6050_Processing _proc;
  MPU6050_Fusion *_filter;
  ProcessedSample _lastProcessed;
  Angles _cachedAngles;
  unsigned long _lastUpdateUs;
  bool _hasNewSample;
};
