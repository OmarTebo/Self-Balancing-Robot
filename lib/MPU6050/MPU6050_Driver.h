#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "MPU6050_Config.h"

struct RawSample {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  unsigned long t_us;
};

class MPU6050_Driver {
public:
  MPU6050_Driver(TwoWire &wire = Wire, uint8_t addr = MPU6050_DEFAULT_ADDR);
  bool begin();
  bool readRaw(RawSample &out);
  uint8_t addr() const { return _addr; }
private:
  TwoWire &_wire;
  uint8_t _addr;
  bool writeReg(uint8_t reg, uint8_t val);
  bool readRegs(uint8_t reg, uint8_t *buf, uint8_t len);
};
