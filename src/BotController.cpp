#include "Config.h" // for STEPS_PER_DEGREE etc.
#include "HardwareMap.h" // optional: provides LEFT_STEP, LEFT_DIR, etc.
#include "BotController.h"
#include "test_mode.h"
#include <Preferences.h>

BotController::BotController() : 
  leftMotor(
    SWAP_MOTORS ? RIGHT_STEP : LEFT_STEP,
    SWAP_MOTORS ? RIGHT_DIR : LEFT_DIR,
    SWAP_MOTORS ? RIGHT_EN : LEFT_EN
  ),
  rightMotor(
    SWAP_MOTORS ? LEFT_STEP : RIGHT_STEP,
    SWAP_MOTORS ? LEFT_DIR : RIGHT_DIR,
    SWAP_MOTORS ? LEFT_EN : RIGHT_EN
  ) {
  portMUX_INITIALIZE(&mux);
  pendingPid = false;
  stepsPerDegree = STEPS_PER_DEGREE;
  testModeRuntime = TEST_MODE_RUNTIME; // Initialize from config
}

void BotController::begin() {
  // Print boot tag for firmware identification
  Serial.printf("BOOT_TAG: %s\n", BOOT_TAG);
  
  leftMotor.begin();
  rightMotor.begin();
  // init display
  display.begin();
  // init IMU with retry+recover on failure
  if (!imu.begin()) {
    Serial.println("IMU init failed — attempting I2C recover + retry");
    IMU::i2cBusRecover(I2C_SDA_PIN, I2C_SCL_PIN);
    delay(50);
    if (!imu.begin()) {
      Serial.println("IMU init failed after recover. Continuing without IMU.");
    } else {
      Serial.println("IMU init succeeded after recover.");
    }
  }
  // ble.begin(); // Disabled for now to avoid radio interference during DLPF runtime tuning. Re-enable if needed.
  // default PID values (Kp, Ki, Kd) in degrees/deg-s/seconds form
  loadStoredPid();
  
  // Print test mode status
  if (TEST_MODE_ENABLED || testModeRuntime) {
    Serial.println("TEST_MODE: ENABLED (motors disabled)");
  }
}

void BotController::update(float dt) {
  // read imu
  imu.update(dt);

  // check for pending BLE params and apply safely
  PIDParams p;
  if (ble.takePending(p)) {
    portENTER_CRITICAL(&mux);
    pendingParams = p;
    pendingPid = true;
    portEXIT_CRITICAL(&mux);
  }
  if (pendingPid) applyPendingPid();

  // Get current roll angle (x-axis rotation) - primary control axis
  // Both motor shafts are parallel to x-axis, so both motors use roll for control
  float currentRoll = imu.getRoll();
  float currentPitch = imu.getPitch(); // Optional, kept for future use if MPU6050 rotated

  // Non-blocking telemetry emit (throttled).
  static unsigned long _lastTelemetryMs = 0;
  const unsigned long _telemetryIntervalMs = 20; // 50 Hz
  unsigned long _nowMs = millis();
  if (_nowMs - _lastTelemetryMs >= _telemetryIntervalMs) {
    _lastTelemetryMs = _nowMs;
    Serial.printf("PITCH:%.2f ROLL:%.2f YAW:%.2f\n", currentPitch, currentRoll, imu.getYaw());
  }

  // Skip motor control in test mode (compile-time or runtime)
  if (TEST_MODE_ENABLED || testModeRuntime) {
    // In test mode, still update display but don't drive motors
    display.update();
    return;
  }

  // PID compute: returns angular velocity (deg/s)
  // Both motors rotate around x-axis, so roll is the primary control axis
  // Pitch is optional (if MPU6050 rotated, but currently not used)
  float rollOutDegPerSec = rollPid.compute(targetRoll, currentRoll, dt); // out in deg/s
  float rollStepsPerSec = rollOutDegPerSec * stepsPerDegree; // convert to steps/sec once

  // Calculate base motor signs with invert flags
  // This allows software testing without hardware rewiring
  float leftSign = LEFT_MOTOR_SIGN * (INVERT_LEFT_MOTOR ? -1.0f : 1.0f);
  float rightSign = RIGHT_MOTOR_SIGN * (INVERT_RIGHT_MOTOR ? -1.0f : 1.0f);
  
  // apply motor sign configuration so left/right can be inverted without code edits
  float leftSteps = rollStepsPerSec * leftSign;
  float rightSteps = rollStepsPerSec * rightSign;

  // drive both wheels from roll controller (non-blocking)
  // Both motors respond to roll (x-axis rotation) together
  leftMotor.setSpeedStepsPerSec(leftSteps);
  rightMotor.setSpeedStepsPerSec(rightSteps);

  // non-blocking stepper service (must be called frequently)
  leftMotor.runSpeed();
  rightMotor.runSpeed();

  // update display animation (throttled)
  display.update();
}

void BotController::requestPidParams(const PIDParams &p) {
  portENTER_CRITICAL(&mux);
  pendingParams = p;
  pendingPid = true;
  portEXIT_CRITICAL(&mux);
}

void BotController::applyPendingPid() {
  portENTER_CRITICAL(&mux);
  if (pendingPid) {
    // interpret pendingParams as continuous (Kp, Ki_per_s, Kd_seconds)
    // Apply to roll PID (primary controller for x-axis rotation)
    rollPid.setTunings(pendingParams.kp, pendingParams.ki, pendingParams.kd);
    rollPid.reset();
    // persist immediately so values survive power cycles
    savePidToStorage(pendingParams.kp, pendingParams.ki, pendingParams.kd);
    pendingPid = false;
  }
  portEXIT_CRITICAL(&mux);
}

void BotController::printCurrentPid() {
  float kp, ki, kd;
  rollPid.getTunings(kp, ki, kd);
  Serial.printf("Roll PID - KP: %.6f KI: %.6f KD: %.6f\n", kp, ki, kd);
}

void BotController::loadStoredPid() {
  Preferences prefs;
  // open namespace for read/write
  if (!prefs.begin(PREFS_NAMESPACE, false)) {
    Serial.println("Prefs begin failed - using defaults");
    // use compile-time defaults for roll PID (primary controller)
    rollPid.begin(DEFAULT_PID_KP, DEFAULT_PID_KI, DEFAULT_PID_KD, PID_OUTPUT_MIN_F, PID_OUTPUT_MAX_F);
    // Initialize pitch PID with same defaults (optional, for future use)
    pitchPid.begin(DEFAULT_PID_KP, DEFAULT_PID_KI, DEFAULT_PID_KD, PID_OUTPUT_MIN_F, PID_OUTPUT_MAX_F);
    return;
  }

  float kp = prefs.getFloat(PREFS_KEY_KP, DEFAULT_PID_KP);
  float ki = prefs.getFloat(PREFS_KEY_KI, DEFAULT_PID_KI);
  float kd = prefs.getFloat(PREFS_KEY_KD, DEFAULT_PID_KD);

  prefs.end();

  Serial.printf("Loaded Roll PID from NVS: KP=%.6f KI=%.6f KD=%.6f\n", kp, ki, kd);
  rollPid.begin(kp, ki, kd, PID_OUTPUT_MIN_F, PID_OUTPUT_MAX_F);
  // Initialize pitch PID with same values (optional, for future use)
  pitchPid.begin(kp, ki, kd, PID_OUTPUT_MIN_F, PID_OUTPUT_MAX_F);
}

void BotController::savePidToStorage(float kp, float ki, float kd) {
  Preferences prefs;
  if (!prefs.begin(PREFS_NAMESPACE, false)) {
    Serial.println("Prefs begin failed - cannot save PID");
    return;
  }
  prefs.putFloat(PREFS_KEY_KP, kp);
  prefs.putFloat(PREFS_KEY_KI, ki);
  prefs.putFloat(PREFS_KEY_KD, kd);
  prefs.end();
  Serial.printf("Saved Roll PID to NVS: KP=%.6f KI=%.6f KD=%.6f\n", kp, ki, kd);
}

void BotController::setTestMode(bool enabled) {
  testModeRuntime = enabled;
  if (enabled) {
    Serial.println("OK TEST_MODE: ON (motors disabled)");
  } else {
    Serial.println("OK TEST_MODE: OFF (motors enabled)");
  }
}

void BotController::runSelfChecks() {
  // Run self-checks using the test_mode module
  bool allPassed = ::runSelfChecks(imu, leftMotor, rightMotor, display, ble, rollPid);
  // Result is already printed by runSelfChecks()
}
