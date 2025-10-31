#include "MPU6050_Driver.h"
#include <Wire.h>

#define PWR_MGMT_1 0x6B
#define SMPLRT_DIV 0x19
#define CONFIG 0x1A
#define GYRO_CONFIG 0x1B
#define ACCEL_CONFIG 0x1C
#define ACCEL_XOUT_H 0x3B

MPU6050_Driver::MPU6050_Driver(TwoWire &wire, uint8_t addr): _wire(wire), _addr(addr) {}

bool MPU6050_Driver::begin() {
  _wire.begin();
  _wire.setClock(400000);
  if (!writeReg(PWR_MGMT_1, 0x00)) return false;
  writeReg(SMPLRT_DIV, 0x07);
  writeReg(CONFIG, 0x03);
  writeReg(GYRO_CONFIG, 0x00);
  writeReg(ACCEL_CONFIG, 0x00);
  delay(50);
  return true;
}

bool MPU6050_Driver::writeReg(uint8_t reg, uint8_t val) {
  _wire.beginTransmission(_addr);
  _wire.write(reg);
  _wire.write(val);
  return _wire.endTransmission() == 0;
}

bool MPU6050_Driver::readRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  _wire.beginTransmission(_addr);
  _wire.write(reg);
  if (_wire.endTransmission(false) != 0) return false;
  uint8_t got = _wire.requestFrom((int)_addr, (int)len);
  if (got < len) return false;
  for (uint8_t i=0;i<len;i++) buf[i] = _wire.read();
  return true;
}

bool MPU6050_Driver::readRaw(RawSample &out) {
  uint8_t buf[14];
  if (!readRegs(ACCEL_XOUT_H, buf, 14)) return false;
  out.ax = (int16_t)((buf[0]<<8) | buf[1]);
  out.ay = (int16_t)((buf[2]<<8) | buf[3]);
  out.az = (int16_t)((buf[4]<<8) | buf[5]);
  out.gx = (int16_t)((buf[8]<<8) | buf[9]);
  out.gy = (int16_t)((buf[10]<<8) | buf[11]);
  out.gz = (int16_t)((buf[12]<<8) | buf[13]);
  out.t_us = micros();
  return true;
}
