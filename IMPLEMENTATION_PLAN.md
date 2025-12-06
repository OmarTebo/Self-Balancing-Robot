# SBR Implementation Plan

## Overview
Plan for implementing calibration persistence, test modes, serial commands, unit tests, and documentation improvements.

---

## 1. Calibration Persistence ✅ HIGH PRIORITY

### Problem
Currently, calibration happens automatically in `IMU::begin()`. If the robot powers up while tilted, it will calibrate to that tilt and think it's the zero position. This causes permanent offset errors.

### Solution
- **Remove automatic calibration** from `begin()`
- **Add explicit calibration command** via serial: `CALIBRATE`
- **Store calibration data** in NVS (using same Preferences pattern as PID)
- **Load calibration on boot** if it exists
- **Add commands**: `CALIBRATE`, `SAVE_CAL`, `LOAD_CAL`, `CLEAR_CAL`, `GET_CAL_INFO`

### Implementation Details

**Config.h additions:**
```cpp
// IMU Calibration constants
#define IMU_CALIB_MAGIC 0xCA1B1234  // Magic number to verify valid calibration data
#define IMU_CALIB_VERSION 1          // Version for future compatibility
#define IMU_CALIB_SAMPLES 200        // Number of samples for calibration (moved from IMU.cpp)
#define IMU_CALIB_DELAY_MS 5         // Delay between samples (moved from IMU.cpp)
#define PREFS_KEY_CALIB_PITCH_OFFSET "cal_pitch"
#define PREFS_KEY_CALIB_ROLL_OFFSET  "cal_roll"
#define PREFS_KEY_CALIB_MAGIC        "cal_magic"
#define PREFS_KEY_CALIB_VERSION      "cal_ver"
```

**IMU.h additions:**
```cpp
struct CalibrationData {
  float pitchOffset;
  float rollOffset;
  uint32_t magic;
  uint8_t version;
};

bool calibrateBlocking();  // Blocking calibration (user must keep robot still)
bool saveCalibration();     // Save to NVS
bool loadCalibration();     // Load from NVS
bool clearCalibration();    // Clear stored calibration
bool hasCalibration();      // Check if valid calibration exists
void getCalibrationInfo(CalibrationData &out);  // Get current calibration data
void applyCalibration();    // Apply offsets to current readings
```

**IMU.cpp changes:**
- Remove automatic calibration from `begin()`
- Add `calibrateBlocking()` - runs calibration loop, requires robot to be still
- Add NVS save/load using Preferences (same pattern as PID storage)
- Add checksum/magic number verification
- Apply calibration offsets in `update()` or `getPitch()/getRoll()`

**SerialBridge.cpp additions:**
- `CALIBRATE` - triggers blocking calibration
- `SAVE_CAL` - save current calibration to NVS
- `LOAD_CAL` - load calibration from NVS
- `CLEAR_CAL` - clear stored calibration
- `GET_CAL_INFO` - print current calibration offsets

---

## 2. Test Modes

### Purpose
Allow testing different mechanical, electrical, or information states without spinning motors or risking damage.

### Implementation

**Config.h:**
```cpp
#define TEST_MODE_ENABLED true  // Enable test mode compilation
```

**test_mode.cpp (new file):**
```cpp
// Self-check routines that don't require motors
void runSelfChecks() {
  // 1. IMU detection test
  // 2. I2C bus test
  // 3. Motor driver pin test (without enabling motors)
  // 4. Display test
  // 5. BLE test
  // 6. Serial communication test
  // 7. NVS read/write test
  // 8. PID computation test (with dummy values)
}
```

**main.cpp or BotController.cpp:**
- Add `TEST_MODE` flag check
- If enabled, skip motor control, run self-checks instead
- Can be enabled via serial command or compile-time flag

**SerialBridge.cpp:**
- `RUN_SELF_CHECKS` - execute test mode self-checks
- `GET_BOOT_TAG` - print firmware version/tag

---

## 3. Serial Commands for Python Tools

### Commands to Add

**Calibration:**
- `CALIBRATE` - Start blocking calibration
- `SAVE_CAL` - Save calibration to NVS
- `LOAD_CAL` - Load calibration from NVS
- `CLEAR_CAL` - Clear stored calibration
- `GET_CAL_INFO` - Get calibration offsets (returns JSON or formatted text)

**System Info:**
- `GET_BOOT_TAG` - Get firmware version/tag
- `GET_STATUS` - Get system status (IMU health, motors enabled, etc.)
- `RUN_SELF_CHECKS` - Run test mode checks

**PID Control:**
- `GET PID` - Already exists, keep it
- `SET PID <kp> <ki> <kd>` - Already exists, keep it
- `LOAD_SAFE` - Load conservative safe PID gains (for testing)
- `SAVE_PID` - Explicitly save current PID (currently auto-saves)

**Motor Control:**
- `ENABLE_MOTORS` - Enable motors (if we add enable control later)
- `DISABLE_MOTORS` - Disable motors (if we add enable control later)
- `SET_TARGET_ROLL <value>` - Set target roll angle
- `GET_TARGET_ROLL` - Get current target roll

**Telemetry:**
- `START_TELEMETRY` - Enable continuous telemetry output
- `STOP_TELEMETRY` - Disable telemetry output
- `TELEMETRY_FORMAT <json|text>` - Set telemetry format

### Response Format
- Success: `OK <command>`
- Error: `ERR <command> <reason>`
- Data: `DATA <command> <data>`
- Or use structured format (JSON-like) for easier parsing

---

## 4. Smoke Test Explanation

### What is a "Smoke Test"?
A **smoke test** is a basic sanity check - the minimum test to verify something works. The name comes from hardware: "if you power it on and smoke comes out, something's wrong."

### For SBR, smoke test means:
A Python script that:
1. **Connects to serial port** (pyserial)
2. **Sends commands** and verifies responses
3. **Checks basic functionality**:
   - Boot tag is printed correctly
   - IMU is detected
   - Calibration can be loaded/saved
   - Serial commands work
   - Self-checks pass
4. **Reports pass/fail** for each check

### Purpose
- **Before PCB arrives**: Verify firmware works on current hardware
- **After PCB arrives**: Quick verification that board works before full testing
- **Automated testing**: Can be run repeatedly, integrated into CI/CD later

### Implementation: `tools/hw_smoke_test.py`
```python
# Pseudo-code structure:
1. Open serial port
2. Wait for boot message, extract BOOT_TAG
3. Send "RUN_SELF_CHECKS", verify response
4. Send "GET_CAL_INFO", verify response format
5. Send "GET PID", verify response
6. Send "GET_STATUS", verify response
7. Report pass/fail for each check
```

---

## 5. Unit Tests

### Priority: IMU, Kalman Filter, PID

### Test Structure (PlatformIO + Unity)

**test/ directory:**
```
test/
  platformio.ini  (test environment config)
  src/
    test_imu.cpp
    test_kalman.cpp
    test_pid.cpp
    test_main.cpp
```

### Test Cases

**test_pid.cpp:**
- Test PID computation with known inputs
- Test integral windup prevention
- Test derivative filtering
- Test output clamping
- Test reset() function
- Test setTunings()

**test_kalman.cpp:**
- Test Kalman filter initialization
- Test prediction step
- Test update step
- Test with known sensor data
- Test convergence behavior

**test_imu.cpp:**
- Test calibration offset application
- Test angle calculation
- Test I2C recovery (mock)
- Test calibration save/load

### Unity Framework
PlatformIO supports Unity test framework. We'll use it for assertions and test organization.

---

## 6. Magic Numbers Documentation

### Current Magic Numbers to Document

**IMU.cpp:**
- `CAL_SAMPLES = 200` - Move to Config.h, document: "200 samples at 5ms = 1 second of averaging for stable calibration"
- `CAL_DELAY_MS = 5` - Move to Config.h, document: "5ms delay allows sensor to settle between readings"
- `1e-5f` (stall detection threshold) - Document: "Threshold for detecting frozen IMU readings (0.00001 degrees)"
- `200` (stall timeout ms) - Document: "Timeout for IMU stall detection (200ms)"

**Config.h:**
- `STEPS_PER_DEGREE = 3200.0f/360.0f` - Document: "Motor steps per degree: 3200 steps/revolution ÷ 360 degrees = 8.888 steps/degree"
- `CONTROL_LOOP_HZ = 200` - Document: "Control loop frequency: 200Hz = 5ms timestep for stable PID control"
- `PID_OUTPUT_MIN/MAX = ±1000.0f` - Document: "PID output limits in deg/s: prevents excessive motor speeds"
- `I2C_CLOCK_HZ = 100000` - Document: "I2C clock speed: 100kHz standard speed for MPU6050"

**PIDController.cpp:**
- `d_alpha = 0.6f` - Document: "Derivative filter alpha: 0.6 = 60% new value, 40% old value (low-pass filter)"

**MotorDriver.cpp:**
- `1000.0f` (default max speed) - Document: "Default max speed: 1000 steps/sec"
- `1000.0f` (default acceleration) - Document: "Default acceleration: 1000 steps/sec²"

### Documentation Format
```cpp
// Magic number: <value>
// Origin: <where it comes from>
// Rationale: <why this value>
// Reference: <any papers/standards>
#define CONSTANT_NAME value
```

---

## Implementation Order

### Phase 1: Calibration Persistence (Critical)
1. Add calibration constants to Config.h
2. Add CalibrationData struct and API to IMU.h
3. Implement calibration persistence in IMU.cpp
4. Add serial commands to SerialBridge.cpp
5. Test calibration save/load
6. Document and commit

### Phase 2: Test Mode
1. Create test_mode.cpp
2. Add TEST_MODE flag to Config.h
3. Implement self-checks
4. Add RUN_SELF_CHECKS command
5. Test on hardware
6. Document and commit

### Phase 3: Serial Commands
1. Add all new serial commands to SerialBridge.cpp
2. Implement response formatting
3. Test each command
4. Document command reference
5. Commit

### Phase 4: Unit Tests
1. Set up PlatformIO test environment
2. Write PID unit tests
3. Write Kalman unit tests
4. Write IMU unit tests
5. Run tests, verify pass
6. Document and commit

### Phase 5: Magic Numbers & Documentation
1. Move magic numbers to Config.h
2. Add detailed comments for each
3. Update DOCUMENTATION.md
4. Commit

### Phase 6: Smoke Test Tool
1. Create tools/hw_smoke_test.py
2. Implement serial communication
3. Implement test checks
4. Test on hardware
5. Document usage
6. Commit

---

## Files to Create/Modify

### New Files:
- `src/test_mode.cpp`
- `test/src/test_pid.cpp`
- `test/src/test_kalman.cpp`
- `test/src/test_imu.cpp`
- `test/src/test_main.cpp`
- `tools/hw_smoke_test.py`

### Modified Files:
- `include/Config.h` - Add calibration constants, magic number documentation
- `include/IMU.h` - Add calibration API
- `src/IMU.cpp` - Remove auto-calibration, add persistence
- `src/SerialBridge.cpp` - Add new commands
- `src/BotController.cpp` - Handle calibration loading
- `src/PIDController.cpp` - Document magic numbers
- `src/MotorDriver.cpp` - Document magic numbers
- `DOCUMENTATION.md` - Update with new features

---

## Success Criteria

- [ ] Calibration can be saved and persists across power cycles
- [ ] Calibration only happens on explicit command (not auto on boot)
- [ ] Test mode can run self-checks without motors
- [ ] All serial commands work and return proper responses
- [ ] Unit tests pass for PID, Kalman, IMU
- [ ] All magic numbers are documented with origins
- [ ] Smoke test script can verify basic functionality
- [ ] All changes documented and committed

---

## Notes

- **Reversibility**: Each phase will be committed separately for easy rollback
- **Testing**: Test each feature before moving to next phase
- **Documentation**: Update docs as we go, not at the end
- **Safety**: Test mode prevents accidental motor activation during development

