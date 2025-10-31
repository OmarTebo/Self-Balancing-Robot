#include "MPU6050_Processing.h"
#include <math.h>

MPU6050_Processing::MPU6050_Processing() {
  _accelScale = 16384.0f;
  _gyroScale = 131.0f;
  _accelOffX = _accelOffY = _accelOffZ = 0.0f;
  _gyroOffX = _gyroOffY = _gyroOffZ = 0.0f;
}

void MPU6050_Processing::setScales(float accelScale, float gyroScaleDegPerSec) {
  _accelScale = accelScale;
  _gyroScale = gyroScaleDegPerSec;
}

void MPU6050_Processing::setOffsets(float ax, float ay, float az, float gx, float gy, float gz) {
  _accelOffX = ax; _accelOffY = ay; _accelOffZ = az;
  _gyroOffX = gx;  _gyroOffY = gy;  _gyroOffZ = gz;
}

void MPU6050_Processing::process(const RawSample &in, ProcessedSample &out) {
  out.t_us = in.t_us;
  out.ax_g = ((float)in.ax - _accelOffX) / _accelScale;
  out.ay_g = ((float)in.ay - _accelOffY) / _accelScale;
  out.az_g = ((float)in.az - _accelOffZ) / _accelScale;
  const float deg2rad = 3.14159265358979323846f / 180.0f;
  out.gx_rads = (((float)in.gx - _gyroOffX) / _gyroScale) * deg2rad;
  out.gy_rads = (((float)in.gy - _gyroOffY) / _gyroScale) * deg2rad;
  out.gz_rads = (((float)in.gz - _gyroOffZ) / _gyroScale) * deg2rad;
}
