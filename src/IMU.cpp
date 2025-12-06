// src/IMU.cpp  -- replaced Adafruit usage with custom MPU6050 lib; adds robust reinit + calibration persistence
#include "IMU.h"
#include "Config.h"
#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <Preferences.h>

#include "MPU6050.h"

static MPU6050 *mpu = nullptr;

IMU::IMU() {
  pitch = roll = yaw = 0.0f;
  lastMillis = 0;
  pitchOffset = 0.0f;
  rollOffset = 0.0f;
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

  // Try to load calibration from NVS
  // If no calibration exists, offsets remain at 0.0 (user must calibrate explicitly)
  if (loadCalibration()) {
    Serial.println("IMU calibration loaded from NVS");
  } else {
    Serial.println("IMU: No calibration found. Use CALIBRATE command to calibrate.");
    pitchOffset = 0.0f;
    rollOffset = 0.0f;
  }

  return true;
}

void IMU::update(float dt) {
  if (!mpu) return;

  float oldPitch = pitch;
  float oldRoll  = roll;

  // delegate to library
  mpu->update(dt);

  float newPitch = mpu->getPitch();
  float newRoll  = mpu->getRoll();
  float newYaw   = mpu->getYaw();

  // apply calibration offsets
  newPitch -= pitchOffset;
  newRoll  -= rollOffset;

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

// Blocking calibration: requires robot to be kept still during execution
// Averages IMU_CALIB_SAMPLES readings to compute zero offsets
bool IMU::calibrateBlocking() {
  if (!mpu) {
    Serial.println("ERR CALIBRATE: IMU not initialized");
    return false;
  }

  Serial.println("CALIBRATE: Starting calibration. Keep robot still...");
  Serial.printf("CALIBRATE: Collecting %d samples (will take ~%d seconds)...\n", 
                IMU_CALIB_SAMPLES, (IMU_CALIB_SAMPLES * IMU_CALIB_DELAY_MS) / 1000);

  float sumPitch = 0.0f;
  float sumRoll = 0.0f;

  for (int i = 0; i < IMU_CALIB_SAMPLES; ++i) {
    // Allow the lib to update internal filters; pass dt=0.0 for internal handling
    mpu->update(0.0f);
    sumPitch += mpu->getPitch();
    sumRoll += mpu->getRoll();
    delay(IMU_CALIB_DELAY_MS);
    
    // Progress indicator every 50 samples
    if ((i + 1) % 50 == 0) {
      Serial.printf("CALIBRATE: Progress %d/%d\n", i + 1, IMU_CALIB_SAMPLES);
    }
  }

  // Compute offsets
  pitchOffset = sumPitch / (float)IMU_CALIB_SAMPLES;
  rollOffset = sumRoll / (float)IMU_CALIB_SAMPLES;

  Serial.printf("CALIBRATE: Done. pitchOffset=%.3f rollOffset=%.3f\n", pitchOffset, rollOffset);
  Serial.println("CALIBRATE: Use SAVE_CAL to persist calibration to NVS");

  return true;
}

// Save calibration to NVS
bool IMU::saveCalibration() {
  Preferences prefs;
  if (!prefs.begin(PREFS_NAMESPACE, false)) {
    Serial.println("ERR SAVE_CAL: Failed to open NVS");
    return false;
  }

  prefs.putFloat(PREFS_KEY_CALIB_PITCH_OFFSET, pitchOffset);
  prefs.putFloat(PREFS_KEY_CALIB_ROLL_OFFSET, rollOffset);
  prefs.putUInt(PREFS_KEY_CALIB_MAGIC, IMU_CALIB_MAGIC);
  prefs.putUChar(PREFS_KEY_CALIB_VERSION, IMU_CALIB_VERSION);
  prefs.end();

  Serial.printf("OK SAVE_CAL: Saved pitchOffset=%.3f rollOffset=%.3f\n", pitchOffset, rollOffset);
  return true;
}

// Load calibration from NVS
bool IMU::loadCalibration() {
  Preferences prefs;
  if (!prefs.begin(PREFS_NAMESPACE, true)) { // read-only
    return false;
  }

  // Check magic number to verify valid calibration
  uint32_t magic = prefs.getUInt(PREFS_KEY_CALIB_MAGIC, 0);
  if (magic != IMU_CALIB_MAGIC) {
    prefs.end();
    return false; // No valid calibration
  }

  // Check version for future compatibility
  uint8_t version = prefs.getUChar(PREFS_KEY_CALIB_VERSION, 0);
  if (version != IMU_CALIB_VERSION) {
    Serial.printf("WARN: Calibration version mismatch (stored: %d, expected: %d)\n", 
                  version, IMU_CALIB_VERSION);
    // Could implement version migration here if needed
  }

  // Load offsets
  pitchOffset = prefs.getFloat(PREFS_KEY_CALIB_PITCH_OFFSET, 0.0f);
  rollOffset = prefs.getFloat(PREFS_KEY_CALIB_ROLL_OFFSET, 0.0f);
  prefs.end();

  return true;
}

// Clear calibration from NVS
bool IMU::clearCalibration() {
  Preferences prefs;
  if (!prefs.begin(PREFS_NAMESPACE, false)) {
    Serial.println("ERR CLEAR_CAL: Failed to open NVS");
    return false;
  }

  prefs.remove(PREFS_KEY_CALIB_PITCH_OFFSET);
  prefs.remove(PREFS_KEY_CALIB_ROLL_OFFSET);
  prefs.remove(PREFS_KEY_CALIB_MAGIC);
  prefs.remove(PREFS_KEY_CALIB_VERSION);
  prefs.end();

  // Reset offsets to zero
  pitchOffset = 0.0f;
  rollOffset = 0.0f;

  Serial.println("OK CLEAR_CAL: Calibration cleared");
  return true;
}

// Check if valid calibration exists
bool IMU::hasCalibration() {
  Preferences prefs;
  if (!prefs.begin(PREFS_NAMESPACE, true)) { // read-only
    return false;
  }

  uint32_t magic = prefs.getUInt(PREFS_KEY_CALIB_MAGIC, 0);
  prefs.end();

  return (magic == IMU_CALIB_MAGIC);
}

// Get current calibration info
void IMU::getCalibrationInfo(CalibrationData &out) {
  out.pitchOffset = pitchOffset;
  out.rollOffset = rollOffset;
  out.magic = IMU_CALIB_MAGIC;
  out.version = IMU_CALIB_VERSION;
}
