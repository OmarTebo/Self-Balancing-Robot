#include "SerialBridge.h"
#include "BLEHandler.h"
#include "IMU.h"
#include "BotController.h"
#include <Arduino.h>

SerialBridge::SerialBridge() {
  _buffer = "";
  _getPidRequested = false;
}

void SerialBridge::begin(unsigned long baud) {
  Serial.begin(baud);
}

void SerialBridge::printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  SET PID <kp> <ki> <kd>   -- set PID (floats)"));
  Serial.println(F("  GET PID                  -- show current PID"));
  Serial.println(F("  CALIBRATE                -- calibrate IMU (keep robot still)"));
  Serial.println(F("  SAVE_CAL                 -- save calibration to NVS"));
  Serial.println(F("  LOAD_CAL                 -- load calibration from NVS"));
  Serial.println(F("  CLEAR_CAL                -- clear stored calibration"));
  Serial.println(F("  GET_CAL_INFO             -- show calibration offsets"));
  Serial.println(F("  RUN_SELF_CHECKS          -- run self-check tests"));
  Serial.println(F("  GET_BOOT_TAG             -- show firmware boot tag"));
  Serial.println(F("  GET_STATUS               -- show system status"));
  Serial.println(F("  TEST_MODE_ON             -- enable test mode (disable motors)"));
  Serial.println(F("  TEST_MODE_OFF            -- disable test mode (enable motors)"));
  Serial.println(F("  HELP"));
}

// simplistic parser: called in loop
// imu: optional pointer for calibration commands (nullptr to skip)
// controller: optional pointer for test mode commands (nullptr to skip)
bool SerialBridge::poll(PIDParams &paramsOut, IMU *imu, BotController *controller) {
  bool gotSet = false;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String line = _buffer;
      line.trim();
      _buffer = "";
      if (line.length() == 0) continue;
      // tokenize
      // transform to upper for command matching
      String up = line;
      up.toUpperCase();
      if (up.startsWith("SET PID")) {
        // parse three floats from original line (to preserve signs)
        float kp, ki, kd;
        int parsed = sscanf(line.c_str(), "SET PID %f %f %f", &kp, &ki, &kd);
        if (parsed == 3) {
          paramsOut.kp = kp;
          paramsOut.ki = ki;
          paramsOut.kd = kd;
          gotSet = true;
          Serial.printf("ACK SET PID %.6f %.6f %.6f\n", kp, ki, kd);
        } else {
          Serial.println("ERR SET PID requires three floats");
        }
      } else if (up == "GET PID" || up.startsWith("GET PID ")) {
        // request that main prints current PID (so access remains in controller)
        _getPidRequested = true;
      } else if (up == "HELP" || up == "HELP ") {
        printHelp();
      } else if (imu != nullptr) {
        // Handle calibration commands if IMU reference provided
        if (up == "CALIBRATE") {
          if (imu->calibrateBlocking()) {
            Serial.println("OK CALIBRATE");
          } else {
            Serial.println("ERR CALIBRATE");
          }
        } else if (up == "SAVE_CAL") {
          if (imu->saveCalibration()) {
            // Message already printed by saveCalibration
          } else {
            Serial.println("ERR SAVE_CAL");
          }
        } else if (up == "LOAD_CAL") {
          if (imu->loadCalibration()) {
            Serial.println("OK LOAD_CAL: Calibration loaded");
          } else {
            Serial.println("ERR LOAD_CAL: No valid calibration found");
          }
        } else if (up == "CLEAR_CAL") {
          if (imu->clearCalibration()) {
            // Message already printed by clearCalibration
          } else {
            Serial.println("ERR CLEAR_CAL");
          }
        } else if (up == "GET_CAL_INFO") {
          CalibrationData cal;
          imu->getCalibrationInfo(cal);
          Serial.printf("DATA CAL_INFO pitchOffset=%.6f rollOffset=%.6f magic=0x%08X version=%d\n",
                        cal.pitchOffset, cal.rollOffset, cal.magic, cal.version);
        } else if (controller != nullptr) {
          // Handle test mode and status commands
          if (up == "RUN_SELF_CHECKS") {
            controller->runSelfChecks();
          } else if (up == "GET_BOOT_TAG") {
            Serial.printf("DATA BOOT_TAG: %s\n", BOOT_TAG);
          } else if (up == "GET_STATUS") {
            // Print system status
            Serial.println("DATA STATUS:");
            Serial.printf("  test_mode_compile: %s\n", TEST_MODE_ENABLED ? "true" : "false");
            Serial.printf("  test_mode_runtime: %s\n", controller->isTestMode() ? "true" : "false");
            Serial.printf("  boot_tag: %s\n", BOOT_TAG);
            // IMU status
            float pitch = imu->getPitch();
            float roll = imu->getRoll();
            float yaw = imu->getYaw();
            Serial.printf("  imu_pitch: %.2f\n", pitch);
            Serial.printf("  imu_roll: %.2f\n", roll);
            Serial.printf("  imu_yaw: %.2f\n", yaw);
            // Calibration status
            Serial.printf("  has_calibration: %s\n", imu->hasCalibration() ? "true" : "false");
          } else if (up == "TEST_MODE_ON") {
            controller->setTestMode(true);
          } else if (up == "TEST_MODE_OFF") {
            controller->setTestMode(false);
          } else {
            Serial.println("UNKNOWN CMD");
          }
        } else {
          Serial.println("UNKNOWN CMD");
        }
      } else {
        Serial.println("UNKNOWN CMD");
      }
    } else {
      _buffer += c;
      // guard buffer length
      if (_buffer.length() > 256) _buffer = _buffer.substring(_buffer.length()-256);
    }
  }
  return gotSet;
}

void SerialBridge::printCurrent(PIDController &pid) {
  float kp, ki, kd;
  pid.getTunings(kp, ki, kd);
  Serial.printf("KP: %.6f KI: %.6f KD: %.6f\n", kp, ki, kd);

}

bool SerialBridge::consumeGetPidRequest() {
  noInterrupts();
  bool v = _getPidRequested;
  _getPidRequested = false;
  interrupts();
  return v;
}
