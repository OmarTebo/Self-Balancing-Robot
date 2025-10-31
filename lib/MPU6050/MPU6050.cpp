#include "MPU6050.h"

MPU6050::MPU6050(TwoWire &w, uint8_t addr): _drv(w, addr), _proc(), _filter(nullptr) {
  _lastUpdateUs = 0;
  _hasNewSample = false;
}

MPU6050::~MPU6050() {}

bool MPU6050::begin(bool doInit) {
  bool ok = true;
  if (doInit) ok = _drv.begin();
  _lastUpdateUs = micros();
  return ok;
}

void MPU6050::attachFilter(MPU6050_Fusion *filter) {
  _filter = filter;
  if (_filter) _filter->begin();
}

void MPU6050::calibrateGyro(uint16_t samples) {
  if (samples == 0) return;
  uint64_t sumgx=0, sumgy=0, sumgz=0;
  RawSample r;
  for (uint16_t i=0;i<samples;i++) {
    if (!_drv.readRaw(r)) { delay(2); continue; }
    sumgx += (int32_t)r.gx;
    sumgy += (int32_t)r.gy;
    sumgz += (int32_t)r.gz;
    delay(2);
  }
  float offx = (float)sumgx / samples;
  float offy = (float)sumgy / samples;
  float offz = (float)sumgz / samples;
  _proc.setOffsets(0,0,0, offx, offy, offz);
}

void MPU6050::update(float dt) {
  RawSample r;
  if (!_drv.readRaw(r)) return;
  ProcessedSample ps;
  _proc.process(r, ps);
  _lastProcessed = ps;

  unsigned long now = micros();
  float dt_s = dt;
  if (dt_s <= 0) {
    if (_lastUpdateUs == 0) dt_s = 0.01f;
    else dt_s = (now - _lastUpdateUs) / 1000000.0f;
  }
  _lastUpdateUs = now;

  if (_filter) {
    _filter->update(ps, dt_s);
    Angles a; _filter->getAngles(a);
    _cachedAngles = a;
  } else {
    _cachedAngles.roll = atan2f(ps.ay_g, ps.az_g) * 180.0f / M_PI;
    _cachedAngles.pitch = atan2f(-ps.ax_g, sqrtf(ps.ay_g*ps.ay_g + ps.az_g*ps.az_g)) * 180.0f / M_PI;
    _cachedAngles.yaw = 0;
  }
}

float MPU6050::getPitch() const { return _cachedAngles.pitch; }
float MPU6050::getRoll()  const { return _cachedAngles.roll; }
float MPU6050::getYaw()   const { return _cachedAngles.yaw; }

void MPU6050::getAngles(Angles &out) const { out = _cachedAngles; }

void MPU6050::setAccelScale(float lsb_per_g) { _proc.setScales(lsb_per_g, 131.0f); }
void MPU6050::setGyroScale(float lsb_per_degps) { _proc.setScales(16384.0f, lsb_per_degps); }
