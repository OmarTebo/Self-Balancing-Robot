#pragma once
#include <Arduino.h>
#include "MPU6050_Driver.h"

struct ProcessedSample {
  float ax_g, ay_g, az_g;
  float gx_rads, gy_rads, gz_rads;
  unsigned long t_us;
};

class MPU6050_Processing {
public:
  MPU6050_Processing();
  void setScales(float accelScale, float gyroScaleDegPerSec);
  void setOffsets(float accelX=0, float accelY=0, float accelZ=0, float gyroX=0, float gyroY=0, float gyroZ=0);
  void process(const RawSample &in, ProcessedSample &out);
private:
  float _accelScale;
  float _gyroScale;
  float _accelOffX, _accelOffY, _accelOffZ;
  float _gyroOffX, _gyroOffY, _gyroOffZ;
};
