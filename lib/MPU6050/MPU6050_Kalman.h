#pragma once
#include "MPU6050_Fusion.h"

class Kalman1D {
public:
  Kalman1D();
  void init(float q_angle, float q_bias, float r_measure);
  void reset(float angle0 = 0.0f);
  float update(float newRate, float newAngle, float dt);
private:
  float angle;
  float bias;
  float P[2][2];
  float Q_angle, Q_bias, R_measure;
};

class MPU6050_Kalman : public MPU6050_Fusion {
public:
  MPU6050_Kalman();
  void begin(float updateHz = MPU_KALMAN_DEFAULT_UPDATE_HZ) override;
  void reset() override;
  void update(const ProcessedSample &s, float dt) override;
  void getAngles(Angles &out) override;
  void setQ(float q_angle, float q_bias);
  void setR(float r);
private:
  Kalman1D kalmanRoll;
  Kalman1D kalmanPitch;
  Angles angles;
};
