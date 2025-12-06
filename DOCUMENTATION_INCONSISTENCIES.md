# Documentation Inconsistencies Report

This document lists inconsistencies found between `DOCUMENTATION.md` and the actual implementation files.

## 1. Config.h - Missing Constants

**Documentation mentions:**
- CONTROL_LOOP_HZ ✓
- STEPS_PER_DEGREE ✓
- I2C pins (I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ) ✓
- Motor pins (PITCH_STEP_PIN, PITCH_DIR_PIN, ROLL_STEP_PIN, ROLL_DIR_PIN) ✓
- SERIAL_BAUD ✓
- Default PID values (KP, KI, KD) ✓

**Missing from documentation:**
- `CONTROL_LOOP_DT_S` - Calculated timestep constant
- `LEFT_MOTOR_SIGN` and `RIGHT_MOTOR_SIGN` - Motor direction configuration
- `DISPLAY_DIN_PIN`, `DISPLAY_CLK_PIN`, `DISPLAY_CS1_PIN`, `DISPLAY_CS2_PIN` - Display pin definitions
- `PITCH_EN_PIN`, `ROLL_EN_PIN` - Motor enable pins (documented as "optional EN pins" but not explicitly listed)
- `PREFS_NAMESPACE`, `PREFS_KEY_KP`, `PREFS_KEY_KI`, `PREFS_KEY_KD` - NVS/Preferences namespace and keys
- `USE_MPU6050` - Feature flag

**Value discrepancy:**
- Documentation example shows `DEFAULT_PID_KD` as `0.01f` (line 125)
- Actual value in Config.h is `1.0f` (line 16)

## 2. IMU.h - Unused Private Method Declarations

**Documentation mentions:**
- All public methods are documented ✓

**Inconsistencies:**
- Header declares private methods `beginMPU()` and `beginBNO()` (lines 26-27 in IMU.h)
- **Neither method is implemented in IMU.cpp** - `begin()` directly initializes MPU6050 without calling these methods
- `beginBNO()` suggests BNO055 support that doesn't exist
- These appear to be leftover declarations from refactoring
- Documentation doesn't mention these (which is fine for private methods), but they're dead code in the header

## 3. PIDController.h - Undocumented Function

**Documentation mentions:**
- All class methods are documented ✓

**Missing:**
- Standalone function `float compute(float setpoint, float measurement, float dt_s);` at line 7 (before the class definition)
- This function is declared but never implemented in PIDController.cpp
- Appears to be dead code or a leftover from refactoring

## 4. BotController - Missing Display Functionality Documentation

**Documentation mentions:**
- `begin()`, `update()`, `requestPidParams()`, `printCurrentPid()`
- Public members: `leftMotor`, `rightMotor`, `targetPitch`, `targetRoll`
- Mentions display initialization in `begin()` but doesn't document the display system

**Missing from documentation:**
- `updateDisplay()` - Private method for display animation
- `displayBitmap()` - Private method for rendering bitmaps
- `displayMatrix` - MD_MAX72XX member variable
- `lastDisplayUpdateMs` - Display update timing
- Display bitmaps (`smileyBitmap`, `sadBitmap`)
- The fact that BotController uses MD_MAX72XX library for LED matrix display

**Note:** The documentation mentions display initialization in `begin()` but doesn't explain what the display does or how it works.

## 5. SerialBridge.h - Unused Struct

**Documentation mentions:**
- All class methods are documented ✓

**Missing:**
- `SerialCmdResult` struct (lines 7-10) - Defined but never used in the implementation
- Appears to be dead code or planned for future use

## 6. BLEHandler - Implementation Details

**Documentation mentions:**
- `begin()`, `takePending()`
- Basic behavior description

**Missing details:**
- BLE device name: "SBR-Bot" (hardcoded in implementation)
- UUIDs used for service and characteristics
- The fact that it uses separate characteristics for KP, KI, KD
- Write callback mechanism details

## 7. main.cpp - Minor Details

**Documentation mentions:**
- Fixed-timestep loop structure ✓
- Accumulator-based timing ✓

**Missing:**
- `MAX_CATCHUP_TICKS` constant (line 11) - Limits catch-up iterations
- The fact that Serial.begin() is called twice (once in setup, once via serialBridge.begin())

## 8. Missing Files from Documentation

**Files that exist but are not documented:**
- `include/display.h` and `src/display.cpp` - Standalone display module
  - These files exist but appear to be unused (BotController implements its own display)
  - Should either be documented or removed if obsolete

## 9. MotorDriver - Implementation Details

**Documentation mentions:**
- All public methods ✓
- Notes about AccelStepper wrapper ✓

**Missing details:**
- Default acceleration: 1000.0f (line 20 in MotorDriver.cpp)
- Default max speed: 1000.0f (line 21 in MotorDriver.cpp)
- Enable pin is assumed to be active LOW (line 14, 37 in MotorDriver.cpp)

## 10. BotController - Motor Naming

**Documentation says:**
- "leftMotor" and "rightMotor" (lines 178-179)

**Implementation:**
- Matches ✓ (lines 21-22 in BotController.h)
- But comments indicate these were renamed from "pitchMotor" and "rollMotor"
- The motors are actually controlling pitch (both motors move together for pitch control)

## Summary

### ✅ COMPLETED

**Dead code removal:**
1. ✅ Removed standalone `compute()` function from PIDController.h
2. ✅ Removed `SerialCmdResult` struct from SerialBridge.h
3. ✅ Removed `beginMPU()` and `beginBNO()` declarations from IMU.h
4. ✅ Refactored display implementation from BotController to Display class

**Documentation updates:**
1. ✅ Fixed DEFAULT_PID_KD value in documentation (0.01f → 1.0f)
2. ✅ Added Display class documentation
3. ✅ Updated BotController section to reflect Display class usage
4. ✅ Added missing Config.h constants documentation
5. ✅ Added missing implementation details (BLE, motor defaults, MAX_CATCHUP_TICKS)

---

### 🔴 REMAINING CODE INCONSISTENCIES (For Next Session)

**Critical inconsistencies:**
1. **Motor pin naming mismatch**: Motors are named `leftMotor`/`rightMotor` but use `PITCH_STEP_PIN`/`ROLL_STEP_PIN` in constructor
   - Current: `leftMotor(PITCH_STEP, PITCH_DIR, PITCH_EN)`
   - Issue: Pin names suggest pitch/roll but motors are left/right
   - Action needed: Rename pins to `LEFT_STEP_PIN`/`RIGHT_STEP_PIN` or clarify naming convention

2. **Roll control not implemented**: `targetRoll` exists but roll PID is not computed
   - Current: Only pitch control is active (`pitchPid.compute()`)
   - Issue: Roll data is read (`imu.getRoll()`) but not used for control
   - Action needed: Implement roll PID controller and motor control
   - Note: User wants to use roll (x-axis tilt) data for control

**Minor inconsistencies:**
1. MotorDriver default values not documented (acceleration: 1000.0f, max speed: 1000.0f)
2. HardwareMap.h still uses PITCH_STEP/ROLL_STEP naming (should align with motor naming)

---

### 📋 NEXT SESSION PLAN

**Code consistency fixes:**
1. Rename motor pins from PITCH_STEP/ROLL_STEP to LEFT_STEP/RIGHT_STEP (or clarify naming)
2. Implement roll PID controller
3. Add roll control to motor output (currently only pitch controls both motors)
4. Update HardwareMap.h to match new naming
5. Update all references to use consistent left/right motor naming

**Verification:**
- Ensure MPU6050 roll data (x-axis tilt) is correctly used
- Verify both motors can be controlled independently if needed
- Test that roll control works as expected

