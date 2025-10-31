// src/IMU.cpp  -- replaced Adafruit usage with custom MPU6050 lib; adds robust reinit
#include "IMU.h"
#include "Config.h"
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

#include "MPU6050.h"

static MPU6050 *mpu = nullptr;

IMU::IMU() {
  pitch = roll = yaw = 0.0f;
  lastMillis = 0;
}

bool IMU::begin() {
  // explicit Wire init (ESP32 pin choices deterministic)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);
  if (mpu) { delete mpu; mpu = nullptr; }

  // try default addr 0x68 then 0x69
  mpu = new MPU6050(Wire, 0x68);
  if (!mpu->begin()) {
    delete mpu;
    mpu = new MPU6050(Wire, 0x69);
    if (!mpu->begin()) {
      Serial.println("MPU6050 not found (custom lib)");
      delete mpu; mpu = nullptr;
      return false;
    }
  }
  delay(50);
  lastMillis = millis();
  Serial.println("MPU6050 initialized (custom lib)");
  return true;
}

void IMU::update(float dt) {
  if (!mpu) return;

  float oldPitch = pitch;
  float oldRoll = roll;

  // delegate to library
  mpu->update(dt);

  float newPitch = mpu->getPitch();
  float newRoll  = mpu->getRoll();
  float newYaw   = mpu->getYaw();

  // detect stall / frozen readings: tiny change over a period
  if (fabsf(newPitch - oldPitch) < 1e-5f && fabsf(newRoll - oldRoll) < 1e-5f &&
      (lastMillis != 0) && (millis() - lastMillis) > 200) {
    Serial.println("IMU stalled — attempting I2C recover + reinit");
    IMU::i2cBusRecover(I2C_SDA_PIN, I2C_SCL_PIN);
    // try reinit addresses again
    if (mpu) { delete mpu; mpu = nullptr; }
    mpu = new MPU6050(Wire, 0x68);
    if (!mpu->begin()) {
      delete mpu;
      mpu = new MPU6050(Wire, 0x69);
      if (!mpu->begin()) {
        Serial.println("IMU reinit failed.");
        delete mpu; mpu = nullptr;
        return;
      }
    }
    Serial.println("IMU reinit successful.");
    lastMillis = millis();
    return;
  }

  // accept new angles
  pitch = newPitch;
  roll  = newRoll;
  yaw   = newYaw;
  lastMillis = millis();
}

float IMU::getPitch(){ return pitch; }
float IMU::getRoll(){ return roll; }
float IMU::getYaw(){ return yaw; }
unsigned long IMU::lastUpdateMillis(){ return lastMillis; }

// i2c bus recovery: bit-bang SCL to free stuck SDA, then toggle lines and re-init Wire
void IMU::i2cBusRecover(int sdaPin, int sclPin) {
  Wire.end();
  pinMode(sclPin, OUTPUT);
  pinMode(sdaPin, INPUT_PULLUP);
  for (int i = 0; i < 9; ++i) {
    digitalWrite(sclPin, HIGH);
    delayMicroseconds(5);
    digitalWrite(sclPin, LOW);
    delayMicroseconds(5);
    if (digitalRead(sdaPin) == HIGH) break;
  }
  pinMode(sdaPin, OUTPUT);
  digitalWrite(sdaPin, LOW);
  delayMicroseconds(5);
  digitalWrite(sclPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(sdaPin, HIGH);
  delayMicroseconds(5);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);
}
