#pragma once
#include <Arduino.h>
#include "Config.h"

// Calibration data structure for persistence
struct CalibrationData {
  float pitchOffset;
  float rollOffset;
  uint32_t magic;
  uint8_t version;
};

class IMU {
public:
  IMU();
  bool begin();
  // update internal sensor fusion if needed; dt optional (some drivers do their own timing)
  void update(float dt);
  // returns angles in degrees
  float getPitch(); // -180..180
  float getRoll();
  float getYaw();

  // optionally expose last update time for health check
  unsigned long lastUpdateMillis();

  // I2C recovery helper (public so caller can invoke)
  static void i2cBusRecover(int sdaPin, int sclPin);

  // Calibration API
  // Blocking calibration: requires robot to be kept still during execution
  // Returns true if calibration succeeded, false on error
  bool calibrateBlocking();
  
  // Save current calibration offsets to NVS
  // Returns true if save succeeded, false on error
  bool saveCalibration();
  
  // Load calibration from NVS if it exists
  // Returns true if valid calibration was loaded, false if none exists or invalid
  bool loadCalibration();
  
  // Clear stored calibration from NVS
  // Returns true if cleared successfully
  bool clearCalibration();
  
  // Check if valid calibration exists in NVS
  // Returns true if valid calibration is stored
  bool hasCalibration();
  
  // Get current calibration data
  // Fills out struct with current offsets and metadata
  void getCalibrationInfo(CalibrationData &out);

private:
  float pitch, roll, yaw;
  unsigned long lastMillis;
  
  // Calibration offsets (degrees)
  float pitchOffset;
  float rollOffset;
};
