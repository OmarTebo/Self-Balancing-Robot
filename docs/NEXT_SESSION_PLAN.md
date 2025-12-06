# Next Session Plan: Code Consistency Fixes

## Overview
This document outlines the code consistency issues to be addressed in the next session, focusing on motor naming and roll control implementation.

---

## Current State Analysis

### Motor Naming Inconsistency
- **Motors are named**: `leftMotor` and `rightMotor` ✅
- **But pins are named**: `PITCH_STEP_PIN`, `ROLL_STEP_PIN` ❌
- **Constructor usage**: `leftMotor(PITCH_STEP, PITCH_DIR, PITCH_EN)`
- **Issue**: Pin names suggest pitch/roll axis control, but motors are physically left/right

### Roll Control Status
- **Roll data is read**: `imu.getRoll()` - returns rotation around X-axis ✅
- **Roll target exists**: `targetRoll` member variable ✅
- **Roll PID missing**: No `rollPid` controller ❌
- **Roll control missing**: Roll is not used to drive motors ❌
- **Current behavior**: Only pitch controls both motors together

---

## Tasks for Next Session

### 1. Rename Motor Pins (High Priority)
**Goal**: Align pin naming with motor naming (left/right instead of pitch/roll)

**Changes needed:**
- [ ] Rename `PITCH_STEP_PIN` → `LEFT_STEP_PIN` (or keep as-is if pitch = left motor)
- [ ] Rename `PITCH_DIR_PIN` → `LEFT_DIR_PIN`
- [ ] Rename `PITCH_EN_PIN` → `LEFT_EN_PIN`
- [ ] Rename `ROLL_STEP_PIN` → `RIGHT_STEP_PIN` (or keep as-is if roll = right motor)
- [ ] Rename `ROLL_DIR_PIN` → `RIGHT_DIR_PIN`
- [ ] Rename `ROLL_EN_PIN` → `RIGHT_EN_PIN`
- [ ] Update `HardwareMap.h` to match new naming
- [ ] Update `BotController.cpp` constructor
- [ ] Update `DOCUMENTATION.md` to reflect new naming

**Decision needed**: 
- Option A: Rename pins to LEFT/RIGHT (clearer, matches motor names)
- Option B: Keep PITCH/ROLL names but document that pitch=left, roll=right
- **User preference**: User wants left/right naming

### 2. Implement Roll Control (High Priority)
**Goal**: Use roll (x-axis tilt) data to control motors

**Changes needed:**
- [ ] Add `PIDController rollPid;` member to BotController
- [ ] Initialize rollPid in `loadStoredPid()` (use same values as pitch initially, or separate storage)
- [ ] In `update()`, compute roll PID: `rollPid.compute(targetRoll, currentRoll, dt)`
- [ ] Decide on control strategy:
  - **Option A**: Differential control - roll affects left/right motor difference
  - **Option B**: Independent control - roll directly controls one motor
  - **Option C**: Combined control - pitch + roll combined for each motor
- [ ] Update motor speed calculations to incorporate roll control
- [ ] Test roll control behavior

**Control Strategy Recommendation:**
For a self-balancing robot:
- **Pitch** (forward/backward tilt) → controls both motors together (current behavior)
- **Roll** (left/right tilt) → controls motor differential (left motor speed up, right motor speed down, or vice versa)

Example implementation:
```cpp
float pitchOut = pitchPid.compute(targetPitch, currentPitch, dt);
float rollOut = rollPid.compute(targetRoll, currentRoll, dt);

// Pitch affects both motors equally
// Roll creates differential (left +roll, right -roll)
float leftSteps = (pitchOut + rollOut) * stepsPerDegree * LEFT_MOTOR_SIGN;
float rightSteps = (pitchOut - rollOut) * stepsPerDegree * RIGHT_MOTOR_SIGN;
```

### 3. Update Documentation
**Changes needed:**
- [ ] Update Config.h section with new pin names
- [ ] Update BotController section to document roll control
- [ ] Add roll PID tuning information
- [ ] Document control strategy (pitch + roll combination)

### 4. Storage Considerations
**Decision needed**: Should roll PID have separate storage or share with pitch?
- **Option A**: Separate storage (PREFS_KEY_ROLL_KP, etc.) - allows independent tuning
- **Option B**: Share pitch PID values - simpler, roll uses same gains as pitch
- **Recommendation**: Option A for flexibility, but start with Option B for simplicity

---

## Verification Checklist

After changes:
- [ ] Motors are correctly named left/right throughout codebase
- [ ] Pin names match motor names (or are clearly documented)
- [ ] Roll data (x-axis tilt) is correctly read from IMU
- [ ] Roll PID controller is initialized and updated
- [ ] Roll control affects motor outputs
- [ ] Both pitch and roll can be controlled simultaneously
- [ ] Documentation reflects all changes
- [ ] No compilation errors
- [ ] MPU6050 functionality preserved

---

## Notes

- **Roll = X-axis rotation**: User is correct - tilting around x-axis is roll
- **Current motors**: Both motors currently respond only to pitch
- **Target**: Use roll data to control motors (likely differential control)
- **Safety**: Test roll control carefully to avoid unexpected behavior

---

## Estimated Impact

- **Files to modify**: Config.h, HardwareMap.h, BotController.h, BotController.cpp
- **New code**: ~50-100 lines (roll PID implementation)
- **Breaking changes**: Pin name changes (if renamed)
- **Testing required**: Roll control behavior verification

