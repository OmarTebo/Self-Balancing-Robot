#include "PIDController.h"
#include "Config.h"

PIDController::PIDController() {
  kp = ki = kd = 0.0f;
  integral = prevError = derivFiltered = 0.0f;
  d_alpha = 0.6f; // derivative filter
  outMin = PID_OUTPUT_MIN_F;
  outMax = PID_OUTPUT_MAX_F;
}

void PIDController::begin(float _kp, float _ki, float _kd, float outMin_f, float outMax_f) {
  kp = _kp; ki = _ki; kd = _kd;
  outMin = outMin_f; outMax = outMax_f;
  reset();
}

void PIDController::setTunings(float _kp, float _ki, float _kd) {
  kp = _kp; ki = _ki; kd = _kd;
}

void PIDController::setTuningsContinuous(float Kp, float Ki_per_s, float Kd_seconds, float sampleRateHz) {
  // Convert continuous to per-sample if required: Ki_per_sample = Ki_per_s / fs; Kd_per_sample = Kd_seconds * fs
  float Ki_per_sample = Ki_per_s / sampleRateHz;
  float Kd_per_sample = Kd_seconds * sampleRateHz;
  // But our compute uses dt explicitly, so store continuous values directly:
  kp = Kp;
  ki = Ki_per_s; // keep per-second form; compute() will multiply by dt
  kd = Kd_seconds; // seconds form; compute divides by dt internally by (err - prev)/dt * kd
}

float PIDController::compute(float setpoint, float measurement, float dt_s) {
  if (dt_s <= 0.0f) return 0.0f;
  float error = setpoint - measurement;
  // P
  float P = kp * error;
  // I (continuous Ki_per_s)
  integral += error * dt_s;
  // anti-windup clamp (based on output range and Ki)
  float integralLimit = 0;
  if (ki != 0.0f) {
    // crude bound to avoid runaway; tune as needed
    integralLimit = fabs(outMax / (ki > 0 ? ki : 1.0f));
  } else integralLimit = 1e6;
  if (integral > integralLimit) integral = integralLimit;
  if (integral < -integralLimit) integral = -integralLimit;
  float I = ki * integral; // because ki is per-second

  // D
  float d_raw = (error - prevError) / dt_s;
  derivFiltered = d_alpha * d_raw + (1.0f - d_alpha) * derivFiltered;
  float D = kd * derivFiltered; // kd is seconds (i.e., Kd_cont) -> kd * d(error)/dt

  prevError = error;

  float out = P + I + D;
  if (out > outMax) out = outMax;
  if (out < outMin) out = outMin;
  return out;
}

void PIDController::reset() {
  integral = 0.0f;
  prevError = 0.0f;
  derivFiltered = 0.0f;
}

void PIDController::getTunings(float &out_kp, float &out_ki, float &out_kd) {
  out_kp = kp;
  out_ki = ki;
  out_kd = kd;
}
