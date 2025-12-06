# Phase 2: Test Mode - Detailed Implementation Plan

## Overview
Test Mode allows running self-checks and diagnostics without spinning motors, enabling safe testing of mechanical, electrical, and information states.

---

## Current State Analysis

### Motor Control Status
- **Enable pins**: `LEFT_EN_PIN = -1`, `RIGHT_EN_PIN = -1` (floating, not connected)
- **MotorDriver::enable()**: Exists but does nothing when enPin < 0
- **Current behavior**: Motors can't be disabled via enable pins
- **Motor control**: Happens in `BotController::update()` via `setSpeedStepsPerSec()` and `runSpeed()`

### What Needs to Change
1. Add TEST_MODE flag (compile-time or runtime)
2. Skip motor control when TEST_MODE is active
3. Create self-check routines
4. Add serial commands to trigger tests

---

## Implementation Details

### 1. Config.h - Test Mode Flag

**Option A: Compile-time flag (recommended for safety)**
```cpp
// Test mode configuration
#define TEST_MODE_ENABLED false  // Set to true to enable test mode compilation
```

**Option B: Runtime flag (more flexible)**
- Add runtime flag that can be toggled via serial command
- More flexible but less safe (could accidentally enable motors)

**Decision**: Use compile-time flag for safety, but also add runtime flag for flexibility.

```cpp
// Test mode configuration
#define TEST_MODE_ENABLED false  // Compile-time test mode (disables motor control)
#define TEST_MODE_RUNTIME false  // Runtime test mode flag (can be toggled via serial)
```

---

### 2. test_mode.cpp - Self-Check Implementation

**File**: `src/test_mode.cpp` (new file)

**Self-check categories:**

#### A. IMU Detection Test
- Verify IMU is detected and responding
- Check I2C communication
- Verify calibration status
- Test angle reading (should be stable when still)

#### B. I2C Bus Test
- Test I2C bus health
- Verify I2C recovery function works
- Check for bus errors

#### C. Motor Driver Pin Test (without enabling motors)
- Test STEP pin can be toggled (digitalWrite)
- Test DIR pin can be set (digitalWrite)
- Verify pin configurations are correct
- **Important**: Don't call `runSpeed()` or `setSpeedStepsPerSec()` in test mode

#### D. Display Test
- Verify display initialization
- Test bitmap rendering
- Check SPI communication

#### E. BLE Test
- Verify BLE device is advertising
- Test characteristic read/write
- Check connection status

#### F. Serial Communication Test
- Echo test (send command, verify response)
- Command parsing test
- Buffer overflow test

#### G. NVS Read/Write Test
- Test Preferences can be opened
- Test read/write operations
- Verify data persistence

#### H. PID Computation Test
- Test PID with known inputs
- Verify output clamping
- Test integral windup prevention
- Use dummy values (no actual motor control)

**Return format:**
- Each test returns pass/fail
- Print detailed results to Serial
- Return overall status (all pass = true)

---

### 3. BotController Changes

**Modifications needed:**

**BotController.h:**
```cpp
private:
  bool testMode;  // Runtime test mode flag
  void runSelfChecks();  // Self-check routine
```

**BotController.cpp:**
- Add `testMode` initialization (from Config.h or serial command)
- Modify `update()` to skip motor control when `testMode` is true:
  ```cpp
  if (testMode) {
    // Skip motor control, maybe run self-checks periodically
    return;  // or run limited checks
  }
  // Normal motor control...
  ```
- Add `runSelfChecks()` method that calls test_mode functions
- Add method to enable/disable test mode: `setTestMode(bool enabled)`

**Decision**: 
- Skip motor control entirely in test mode (don't call `setSpeedStepsPerSec()` or `runSpeed()`)
- Optionally run self-checks periodically in test mode
- Allow toggling test mode via serial command

---

### 4. Serial Commands

**Commands to add:**

**SerialBridge.cpp:**
- `RUN_SELF_CHECKS` - Execute all self-checks immediately
- `GET_BOOT_TAG` - Print firmware version/tag (for smoke test)
- `GET_STATUS` - Get system status (IMU health, test mode, calibration status, etc.)
- `TEST_MODE_ON` - Enable test mode (disables motors)
- `TEST_MODE_OFF` - Disable test mode (enables motors)
- `GET_TEST_MODE` - Check if test mode is active

**Response format:**
```
OK RUN_SELF_CHECKS
TEST: IMU_DETECTION PASS
TEST: I2C_BUS PASS
TEST: MOTOR_PINS PASS
TEST: DISPLAY PASS
TEST: BLE PASS
TEST: SERIAL PASS
TEST: NVS PASS
TEST: PID_COMPUTE PASS
RESULT: ALL_PASS
```

---

### 5. Boot Tag Implementation

**Purpose**: Identify firmware version for smoke test

**Implementation:**
- Add `BOOT_TAG` constant in Config.h (e.g., "pre-pcb-1")
- Print on boot: `BOOT_TAG: pre-pcb-1`
- `GET_BOOT_TAG` command returns this tag

**Config.h:**
```cpp
// Boot tag for firmware identification
// Magic number: "pre-pcb-1"
// Origin: Tag name for firmware version identification
// Rationale: Allows smoke test scripts to verify correct firmware is running
#define BOOT_TAG "pre-pcb-1"
```

**main.cpp or BotController.cpp:**
```cpp
Serial.printf("BOOT_TAG: %s\n", BOOT_TAG);
```

---

### 6. Status Command Implementation

**GET_STATUS command:**
Returns system status in parseable format:
```
DATA STATUS testMode=0 imuHealthy=1 hasCalibration=1 motorsEnabled=1
```

Or more detailed:
```
DATA STATUS
  testMode: false
  imuHealthy: true
  imuLastUpdate: 1234ms
  hasCalibration: true
  calibrationPitchOffset: 0.123
  calibrationRollOffset: -0.456
  motorsEnabled: true
  rollPidKp: 1.000000
  rollPidKi: 0.000000
  rollPidKd: 1.000000
```

---

## File Structure

### New Files:
- `src/test_mode.cpp` - Self-check implementation

### Modified Files:
- `include/Config.h` - Add TEST_MODE flags, BOOT_TAG
- `include/BotController.h` - Add test mode flag and methods
- `src/BotController.cpp` - Skip motor control in test mode, add self-checks
- `include/SerialBridge.h` - Add new command methods
- `src/SerialBridge.cpp` - Implement new commands
- `src/main.cpp` - Print BOOT_TAG on startup

---

## Safety Considerations

1. **Compile-time flag**: Primary safety - if `TEST_MODE_ENABLED = true`, motors never run
2. **Runtime flag**: Secondary safety - can be toggled but compile-time flag overrides
3. **Motor control skip**: In test mode, completely skip `setSpeedStepsPerSec()` and `runSpeed()` calls
4. **Self-checks**: Don't attempt to move motors, only test pin configurations

---

## Test Mode Behavior

### When TEST_MODE is enabled:
- ✅ IMU updates normally (for testing)
- ✅ PID computation runs (for testing)
- ✅ Display updates normally
- ✅ Serial/BLE communication works
- ✅ Self-checks can run
- ❌ Motors never receive speed commands
- ❌ `runSpeed()` is never called
- ❌ No motor movement

### When TEST_MODE is disabled:
- Normal operation
- Motors controlled by PID
- All functionality active

---

## Implementation Steps

1. **Add Config.h flags** (TEST_MODE_ENABLED, TEST_MODE_RUNTIME, BOOT_TAG)
2. **Create test_mode.cpp** with self-check functions
3. **Add test mode to BotController** (flag, skip motor control, self-check method)
4. **Add serial commands** (RUN_SELF_CHECKS, GET_BOOT_TAG, GET_STATUS, test mode toggles)
5. **Add BOOT_TAG printing** in main.cpp or BotController::begin()
6. **Test compilation**
7. **Document and commit**

---

## Self-Check Details

### IMU Detection Test
```cpp
bool testImuDetection(IMU &imu) {
  // Check if IMU is initialized
  // Try to read angles
  // Verify readings are reasonable
  // Check calibration status
}
```

### I2C Bus Test
```cpp
bool testI2cBus() {
  // Test I2C communication
  // Verify bus is not stuck
  // Test recovery function
}
```

### Motor Pin Test
```cpp
bool testMotorPins() {
  // Test STEP pins can be toggled (without motors connected)
  // Test DIR pins can be set
  // Verify pin modes are correct
  // Don't call runSpeed()!
}
```

### Display Test
```cpp
bool testDisplay(Display &display) {
  // Verify display initialized
  // Test rendering a test pattern
  // Check SPI communication
}
```

### BLE Test
```cpp
bool testBle(BLEHandler &ble) {
  // Check if BLE is advertising
  // Test characteristic access
  // Verify device name
}
```

### Serial Test
```cpp
bool testSerial() {
  // Echo test
  // Command parsing test
  // Buffer test
}
```

### NVS Test
```cpp
bool testNvs() {
  // Test Preferences open/close
  // Test read/write operations
  // Verify data persistence
}
```

### PID Test
```cpp
bool testPid(PIDController &pid) {
  // Test with known inputs
  // Verify output clamping
  // Test reset function
  // Use dummy values only
}
```

---

## Expected Output

When `RUN_SELF_CHECKS` is executed:
```
OK RUN_SELF_CHECKS
Starting self-checks...
TEST: IMU_DETECTION - Checking IMU...
TEST: IMU_DETECTION PASS (IMU detected, angles readable)
TEST: I2C_BUS - Testing I2C...
TEST: I2C_BUS PASS (I2C communication OK)
TEST: MOTOR_PINS - Testing motor pins...
TEST: MOTOR_PINS PASS (STEP/DIR pins configurable)
TEST: DISPLAY - Testing display...
TEST: DISPLAY PASS (Display initialized, pattern rendered)
TEST: BLE - Testing BLE...
TEST: BLE PASS (BLE advertising, characteristics accessible)
TEST: SERIAL - Testing serial...
TEST: SERIAL PASS (Commands parseable, responses correct)
TEST: NVS - Testing NVS...
TEST: NVS PASS (Read/write operations successful)
TEST: PID_COMPUTE - Testing PID...
TEST: PID_COMPUTE PASS (Output clamped, integral windup prevented)
RESULT: ALL_PASS (8/8 tests passed)
```

---

## Questions to Resolve

1. **Test mode flag location**: 
   - Compile-time only? Runtime toggle? Both?
   - **Recommendation**: Both - compile-time for safety, runtime for flexibility

2. **Self-check frequency**:
   - Run once on command? Run periodically in test mode?
   - **Recommendation**: On command only (RUN_SELF_CHECKS), but allow periodic checks if needed

3. **Motor pin testing**:
   - How to test pins without motors? Just digitalWrite tests?
   - **Recommendation**: Test pin modes and digitalWrite, don't attempt actual motor control

4. **Status format**:
   - Simple key=value? JSON? Structured text?
   - **Recommendation**: Simple key=value for easy parsing, with optional detailed format

---

## Success Criteria

- [ ] TEST_MODE flag prevents all motor control
- [ ] Self-checks can run without motors
- [ ] All 8 self-check categories implemented
- [ ] Serial commands work (RUN_SELF_CHECKS, GET_BOOT_TAG, GET_STATUS)
- [ ] BOOT_TAG printed on startup
- [ ] Test mode can be toggled via serial
- [ ] No compilation errors
- [ ] Documentation updated
- [ ] Changes committed

---

## Estimated Time

- Config.h additions: 15 min
- test_mode.cpp creation: 2-3 hours
- BotController modifications: 1 hour
- Serial command additions: 1 hour
- Testing and debugging: 1-2 hours
- Documentation: 30 min
- **Total: ~5-7 hours**

---

## Notes

- Test mode is critical for safe development
- Allows testing without risk of motor damage
- Essential for smoke test automation
- Can be used for bench testing before PCB assembly
- Self-checks help identify hardware issues early

