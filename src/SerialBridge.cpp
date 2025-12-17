#include "SerialBridge.h"
#include "BLEHandler.h"
#include "IMU.h"
#include "BotController.h"
#include <Arduino.h>

// Application-level DLPF control helpers defined in IMU.cpp.
// We forward-declare them here instead of touching IMU.h to keep the public
// header surface stable. To revert, remove these declarations and the related
// command handling branches below.
extern bool configureDLPF(uint8_t dlpf_cfg, uint8_t smplrt_div, bool force);
extern bool getDLPFConfig(uint8_t &dlpf_cfg, uint8_t &smplrt_div);
extern const char *getImuDlpfLastError();
extern bool saveImuDlpfConfigToNvs(uint8_t dlpf_cfg, uint8_t smplrt_div);

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
  Serial.println(F("  IMU:GET DLPF             -- get MPU6050 DLPF/SMPLRT_DIV"));
  Serial.println(F("  IMU:SET DLPF <cfg> [div] -- set DLPF/SMPLRT_DIV (safe, motors off)"));
  Serial.println(F("  IMU:APPLY DLPF <cfg> [div] [FORCE] -- apply DLPF even if motors on"));
  Serial.println(F("  IMU:GET LASTERR          -- get last DLPF config error"));
  Serial.println(F("  IMU:HELP                 -- list IMU DLPF commands"));
  Serial.println(F("  HELP"));
}

// Helper: compute default SMPLRT_DIV for a desired DLPF setting if the user
// omits it. For DLPF enabled (cfg != 0), use 4 => 200 Hz from 1 kHz. For
// DLPF disabled (cfg == 0), use 39 => 200 Hz from 8 kHz.
static uint8_t defaultSmplrtForDlpf(uint8_t dlpf_cfg) {
  if ((dlpf_cfg & 0x07u) == 0) {
    return 39; // 8 kHz / (1 + 39) = 200 Hz
  }
  return 4; // 1 kHz / (1 + 4) = 200 Hz
}

// Helper: approximate delay string for replies. Matches IMU.cpp mapping.
static const char *dlpfDelayMsStringReply(uint8_t cfg) {
  switch (cfg & 0x07) {
    case 0: return "0.98";
    case 1: return "1.90";
    case 2: return "2.80";
    case 3: return "4.90";
    case 4: return "8.30";
    case 5: return "13.40";
    case 6: return "18.60";
    case 7: return "approx";
    default: return "approx";
  }
}

// Handle IMU:... commands. All responses are single-line and start with
// OK or ERR for easy machine parsing.
static void handleImuDlpfCommand(const String &line, const String &up, IMU *imu, BotController *controller) {
  (void)imu; // currently unused, kept for future extension
  (void)controller;

  if (up == "IMU:HELP") {
    Serial.println("OK IMU:GET DLPF");
    Serial.println("OK IMU:SET DLPF <cfg> [div]");
    Serial.println("OK IMU:APPLY DLPF <cfg> [div] [FORCE]");
    Serial.println("OK IMU:GET LASTERR");
    return;
  }

  if (up == "IMU:GET DLPF") {
    uint8_t cfg = 0, div = 0;
    if (!getDLPFConfig(cfg, div)) {
      const char *err = getImuDlpfLastError();
      Serial.print("ERR ");
      Serial.println(err ? err : "GET_FAILED");
      return;
    }
    Serial.printf("OK DLPF=%u SMPLRT=%u DELAY_MS=%s\n",
                  cfg, div, dlpfDelayMsStringReply(cfg));
    return;
  }

  if (up == "IMU:GET LASTERR") {
    const char *err = getImuDlpfLastError();
    Serial.printf("OK LASTERR=%s\n", err ? err : "NONE");
    return;
  }

  // Commands with arguments: IMU:SET DLPF ... / IMU:APPLY DLPF ...
  if (up.startsWith("IMU:SET DLPF") || up.startsWith("IMU:APPLY DLPF")) {
    bool isApply = up.startsWith("IMU:APPLY");
    bool force = false;

    // Extract tokens from original line to preserve spacing for numbers
    // Expected forms:
    //  IMU:SET DLPF <cfg> [div]
    //  IMU:APPLY DLPF <cfg> [div] [FORCE]
    char cmd[16], keyword[16];
    int cfg = -1;
    int div = -1;
    char extra[16] = {0};

    int parsed = sscanf(line.c_str(), "%15s %15s %d %d %15s", cmd, keyword, &cfg, &div, extra);
    if (parsed < 3) {
      Serial.println("ERR BAD_PARAMS");
      return;
    }

    // If only cfg provided, compute default divider
    if (parsed == 3) {
      div = (int)defaultSmplrtForDlpf((uint8_t)cfg);
    } else if (parsed >= 5) {
      String extraStr(extra);
      extraStr.toUpperCase();
      if (extraStr == "FORCE") {
        force = true;
      }
    }

    if (cfg < 0 || cfg > 7 || div < 0 || div > 255) {
      Serial.println("ERR BAD_PARAMS");
      return;
    }

    if (isApply && !force) {
      // For APPLY, we require explicit FORCE to override safety.
      Serial.println("ERR MOTORS_ENABLED_USE_FORCE");
      return;
    }

    bool ok = configureDLPF((uint8_t)cfg, (uint8_t)div, force);
    if (!ok) {
      const char *err = getImuDlpfLastError();
      Serial.print("ERR ");
      Serial.println(err ? err : "CONFIG_FAILED");
      return;
    }

    // Persist configuration so it is restored automatically on next boot.
    if (!saveImuDlpfConfigToNvs((uint8_t)cfg, (uint8_t)div)) {
      const char *err = getImuDlpfLastError();
      Serial.print("ERR ");
      Serial.println(err ? err : "NVS_SAVE_FAILED");
      return;
    }

    Serial.printf("OK DLPF=%d SMPLRT=%d DELAY_MS=%s NOTE=ignoring_next_2_samples\n",
                  cfg, div, dlpfDelayMsStringReply((uint8_t)cfg));
    return;
  }

  Serial.println("ERR UNKNOWN_IMU_CMD");
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
        if (up.startsWith("IMU:")) {
          handleImuDlpfCommand(line, up, imu, controller);
        } else if (up == "CALIBRATE") {
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
