#include "MPU6050_Kalman.h"
#include <math.h>

Kalman1D::Kalman1D() {
  angle = 0; bias = 0;
  P[0][0]=P[0][1]=P[1][0]=P[1][1]=0;
  Q_angle = MPU_KALMAN_DEFAULT_Q_ANGLE;
  Q_bias = MPU_KALMAN_DEFAULT_Q_BIAS;
  R_measure = MPU_KALMAN_DEFAULT_R_MEASURE;
}

void Kalman1D::init(float q_angle, float q_bias, float r_measure) {
  Q_angle = q_angle; Q_bias = q_bias; R_measure = r_measure;
}

void Kalman1D::reset(float angle0) {
  angle = angle0; bias = 0;
  P[0][0]=P[0][1]=P[1][0]=P[1][1]=0;
}

float Kalman1D::update(float newRate, float newAngle, float dt) {
  angle += dt * (newRate - bias);
  P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float S = P[0][0] + R_measure;
  float K0 = P[0][0] / S;
  float K1 = P[1][0] / S;

  float y = newAngle - angle;
  angle += K0 * y;
  bias  += K1 * y;

  float P00 = P[0][0], P01 = P[0][1], P10 = P[1][0], P11 = P[1][1];
  P[0][0] = P00 - K0 * P00;
  P[0][1] = P01 - K0 * P01;
  P[1][0] = P10 - K1 * P00;
  P[1][1] = P11 - K1 * P01;

  return angle;
}

MPU6050_Kalman::MPU6050_Kalman() {}

void MPU6050_Kalman::begin(float updateHz) {
  if (updateHz <= 0) updateHz = MPU_KALMAN_DEFAULT_UPDATE_HZ;
  if (updateHz > MPU_MAX_UPDATE_HZ) updateHz = MPU_MAX_UPDATE_HZ;
  kalmanRoll.init(MPU_KALMAN_DEFAULT_Q_ANGLE, MPU_KALMAN_DEFAULT_Q_BIAS, MPU_KALMAN_DEFAULT_R_MEASURE);
  kalmanPitch.init(MPU_KALMAN_DEFAULT_Q_ANGLE, MPU_KALMAN_DEFAULT_Q_BIAS, MPU_KALMAN_DEFAULT_R_MEASURE);
  reset();
}

void MPU6050_Kalman::reset() {
  kalmanRoll.reset(0.0f);
  kalmanPitch.reset(0.0f);
  angles.pitch = angles.roll = angles.yaw = 0.0f;
}

void MPU6050_Kalman::update(const ProcessedSample &s, float dt) {
  float rollAcc  = atan2f(s.ay_g, s.az_g) * 180.0f / M_PI;
  float pitchAcc = atan2f(-s.ax_g, sqrtf(s.ay_g*s.ay_g + s.az_g*s.az_g)) * 180.0f / M_PI;

  float gx_dps = s.gx_rads * 180.0f / M_PI;
  float gy_dps = s.gy_rads * 180.0f / M_PI;

  float rollEst = kalmanRoll.update(gx_dps, rollAcc, dt);
  float pitchEst = kalmanPitch.update(gy_dps, pitchAcc, dt);

  angles.roll = rollEst;
  angles.pitch = pitchEst;
}

void MPU6050_Kalman::getAngles(Angles &out) { out = angles; }

void MPU6050_Kalman::setQ(float q_angle, float q_bias) {
  kalmanRoll.init(q_angle, q_bias, MPU_KALMAN_DEFAULT_R_MEASURE);
  kalmanPitch.init(q_angle, q_bias, MPU_KALMAN_DEFAULT_R_MEASURE);
}

void MPU6050_Kalman::setR(float r) {
  kalmanRoll.init(MPU_KALMAN_DEFAULT_Q_ANGLE, MPU_KALMAN_DEFAULT_Q_BIAS, r);
  kalmanPitch.init(MPU_KALMAN_DEFAULT_Q_ANGLE, MPU_KALMAN_DEFAULT_Q_BIAS, r);
}
