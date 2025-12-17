# Complete Test Flow Guide - SBR Firmware Testing

This guide walks you through the complete testing process from code completion to validation.

---

## Overview: What We've Built

### Firmware Features:
1. **Calibration Persistence** - IMU calibration saves to NVS, loads on boot
2. **Test Mode** - Safe testing without motors (compile-time and runtime flags)
3. **Self-Check Tests** - 8 diagnostic tests (IMU, I2C, motors, display, BLE, serial, NVS, PID)
4. **Serial Commands** - Full command set for testing and control
5. **Boot Tag** - Firmware version identification

### Testing Tools:
1. **imu_telemetry_graph.py** - Real-time plotting of IMU data
2. **dummy_imu_generator.py** - Generate test data without physical IMU
3. **ALGORITHM_EXTRACTION.md** - Algorithm documentation for expected value calculation

---

## Phase 1: Initial Setup & Upload

### Step 1.1: Verify Configuration
Check `include/Config.h`:
- `TEST_MODE_ENABLED = false` (motors will run normally)
- `TEST_MODE_RUNTIME = false` (can be toggled via serial)
- `BOOT_TAG = "pcb-v2-prototype-1"` (increment this with each code change)

### Step 1.2: Compile & Upload
```bash
# Compile to check for errors
pio run

# Upload to ESP32
pio run -t upload

# Open serial monitor (optional, for watching boot)
pio device monitor
```

**Expected Output on Boot:**
```
SBR minimal starting...
BOOT_TAG: pcb-v2-prototype-1
MPU6050 initialized (custom lib)  [or: IMU init failed...]
IMU: No calibration found. Use CALIBRATE command to calibrate.
Loaded Roll PID from NVS: KP=1.000000 KI=0.000000 KD=1.000000
```

---

## Phase 2: Basic Functionality Test (Without IMU)

Since you don't have the IMU yet, test basic functionality:

### Step 2.1: Connect Serial Monitor
```bash
pio device monitor
# Or use any serial terminal at 115200 baud
```

### Step 2.2: Test Serial Commands
Send these commands and verify responses:

```
HELP
```
**Expected:** List of all available commands

```
GET_BOOT_TAG
```
**Expected:** `DATA BOOT_TAG: pcb-v2-prototype-1`

```
GET_STATUS
```
**Expected:** System status with test mode flags, IMU status, etc.

```
GET PID
```
**Expected:** Current PID values

### Step 2.3: Enable Test Mode (Recommended Without IMU)
Since IMU is missing, enable test mode to prevent motors from running with invalid data:

```
TEST_MODE_ON
```
**Expected:** `OK TEST_MODE: ON (motors disabled)`

### Step 2.4: Run Self-Checks
```
RUN_SELF_CHECKS
```
**Expected:** 8 tests run, some may fail (especially IMU_DETECTION)

**What to Expect:**
- ✅ SERIAL - Should pass (you're using it!)
- ✅ NVS - Should pass (storage works)
- ✅ PID_COMPUTE - Should pass (math works)
- ❌ IMU_DETECTION - Will fail (no IMU connected)
- ❌ I2C_BUS - May fail (no IMU on bus)
- ⚠️ Others - May pass/fail depending on hardware

---

## Phase 3: Testing with Dummy IMU Data

### Step 3.1: Generate Expected Values CSV
Use the dummy generator to create test patterns:

```bash
# Generate sine wave pattern (10 seconds)
python tools/dummy_imu_generator.py --pattern sine --duration 10 --output-csv tools/expected_sine.csv

# Generate step response
python tools/dummy_imu_generator.py --pattern step --duration 10 --output-csv tools/expected_step.csv

# Generate tilt pattern
python tools/dummy_imu_generator.py --pattern tilt --duration 15 --output-csv tools/expected_tilt.csv
```

**Note:** The dummy generator currently outputs telemetry format, not I2C data. For full testing without IMU, you'd need to modify firmware to accept serial-injected sensor data (future enhancement).

### Step 3.2: Use Telemetry Grapher (When IMU is Connected)
Once you have the IMU:

```bash
# Basic real-time plotting
python tools/imu_telemetry_graph.py --port COM3

# With expected values overlay
python tools/imu_telemetry_graph.py --port COM3 --expected-file tools/expected_sine.csv

# Custom time window
python tools/imu_telemetry_graph.py --port COM3 --window 20 --save
```

**What You'll See:**
- Three subplots: Pitch, Roll, Yaw
- Real-time data from firmware
- Expected values as dashed lines (if CSV provided)
- Statistics (mean, std dev, range)

---

## Phase 4: IMU Calibration (When IMU is Connected)

### Step 4.1: Physical Setup
1. Place robot on **flat, level surface**
2. Ensure robot is **completely still**
3. No vibrations or movement

### Step 4.2: Calibrate IMU
```
CALIBRATE
```
**Expected:** 
```
CALIBRATE: Starting calibration. Keep robot still...
CALIBRATE: Collecting 200 samples (will take ~1 seconds)...
CALIBRATE: Progress 50/200
CALIBRATE: Progress 100/200
CALIBRATE: Progress 150/200
CALIBRATE: Progress 200/200
CALIBRATE: Done. pitchOffset=0.123 rollOffset=-0.456
CALIBRATE: Use SAVE_CAL to persist calibration to NVS
```

### Step 4.3: Save Calibration
```
SAVE_CAL
```
**Expected:** `OK SAVE_CAL: Saved pitchOffset=0.123 rollOffset=-0.456`

### Step 4.4: Verify Calibration
```
GET_CAL_INFO
```
**Expected:** `DATA CAL_INFO pitchOffset=0.123 rollOffset=-0.456 magic=0xCA1B1234 version=1`

### Step 4.5: Test Persistence
1. Power cycle the ESP32
2. Check serial output on boot - should see: `IMU calibration loaded from NVS`
3. Run `GET_CAL_INFO` - should show same offsets

### Step 4.6 (Optional but Recommended): Configure IMU DLPF & Store in NVS
Once calibration is stable, configure the MPU6050's on-chip Digital Low-Pass Filter (DLPF)
and sample rate for a 200 Hz control loop. This is done once, then restored automatically
from NVS on each boot.

1. **Check current DLPF configuration:**
   ```text
   IMU:GET DLPF
   ```
   **Expected (before tuning):**
   ```text
   OK DLPF=<cfg> SMPLRT=<div> DELAY_MS=<approx>
   ```

2. **Set recommended 200 Hz settings and persist to NVS:**
   ```text
   IMU:SET DLPF 1 4
   ```
   **Expected on success:**
   ```text
   OK DLPF=1 SMPLRT=4 DELAY_MS=1.90 NOTE=ignoring_next_2_samples
   ```
   - If you see `ERR ...`, inspect the last error:
     ```text
     IMU:GET LASTERR
     ```

3. **Verify stored configuration:**
   ```text
   IMU:GET DLPF
   ```
   **Expected:**
   ```text
   OK DLPF=1 SMPLRT=4 DELAY_MS=1.90
   ```

4. **Verify persistence after reboot:**
   - Power cycle the ESP32.
   - On boot you should see a line similar to:
     ```text
     IMU: Restored DLPF from NVS (cfg=1 div=4)
     ```
   - You can double-check with:
     ```text
     IMU:GET DLPF
     ```

5. **(Advanced) Apply with FORCE while motors are running (not recommended unless you know what you're doing):**
   ```text
   IMU:APPLY DLPF 1 4 FORCE
   ```
   - If you omit `FORCE`:
     ```text
     IMU:APPLY DLPF 1 4
     ```
     you will get:
     ```text
     ERR MOTORS_ENABLED_USE_FORCE
     ```

---

## Phase 5: PID Tuning (When Motors & IMU are Connected)

### Step 5.1: Safety First
- **Keep wheels off ground** during initial tuning
- Start with **conservative PID values**
- Have **emergency stop** ready

### Step 5.2: Get Current PID
```
GET PID
```
**Expected:** `Roll PID - KP: 1.000000 KI: 0.000000 KD: 1.000000`

### Step 5.3: Set PID Values
```
SET PID 1.0 0.0 1.0
```
**Expected:** `ACK SET PID 1.000000 0.000000 1.000000`

**Note:** Values are automatically saved to NVS and persist across power cycles.

### Step 5.4: Monitor Telemetry
Watch the telemetry output:
```
PITCH:0.12 ROLL:-0.45 YAW:89.01
PITCH:0.13 ROLL:-0.44 YAW:89.02
...
```

Or use the telemetry grapher for visual feedback.

### Step 5.5: Tune Iteratively
1. Start with **Kp only** (Ki=0, Kd=0)
2. Increase Kp until robot responds (but not oscillating)
3. Add **Kd** to dampen oscillations
4. Add small **Ki** only if steady-state error exists
5. Save good values (they auto-save)

---

## Phase 6: Full System Test

### Step 6.1: Disable Test Mode
```
TEST_MODE_OFF
```
**Expected:** `OK TEST_MODE: OFF (motors enabled)`

### Step 6.2: Run Full Self-Checks
```
RUN_SELF_CHECKS
```
**Expected:** All 8 tests should pass:
- ✅ IMU_DETECTION
- ✅ I2C_BUS
- ✅ MOTOR_PINS
- ✅ DISPLAY
- ✅ BLE
- ✅ SERIAL
- ✅ NVS
- ✅ PID_COMPUTE

### Step 6.3: Monitor System Status
```
GET_STATUS
```
**Expected:** Complete system status including:
- Test mode flags
- IMU readings
- Calibration status
- PID values

---

## Phase 7: Validation & Documentation

### Step 7.1: Verify All Features
- [ ] Calibration saves and loads correctly
- [ ] Test mode disables motors
- [ ] Self-checks run successfully
- [ ] Serial commands work
- [ ] PID values persist
- [ ] Boot tag is correct

### Step 7.2: Update Boot Tag
After significant changes, update in `Config.h`:
```cpp
#define BOOT_TAG "pcb-v2-prototype-2"  // Increment number
```

### Step 7.3: Document Results
- Note any issues found
- Record successful PID values
- Document hardware-specific findings

---

## Troubleshooting Guide

### Problem: IMU Not Detected
**Symptoms:** `IMU init failed after recover. Continuing without IMU.`
**Solutions:**
- Check I2C wiring (SDA=21, SCL=22)
- Verify IMU power (3.3V)
- Check I2C pull-up resistors
- Try I2C bus recovery: firmware does this automatically

### Problem: Motors Not Responding
**Check:**
1. Is test mode enabled? (`GET_STATUS`)
2. Are motors physically connected?
3. Are motor drivers powered?
4. Check motor pin configuration in `Config.h`

### Problem: Calibration Fails
**Solutions:**
- Ensure robot is completely still during calibration
- Wait for full calibration to complete (1 second)
- Check NVS storage: `GET_CAL_INFO`
- Clear and retry: `CLEAR_CAL` then `CALIBRATE`

### Problem: Serial Commands Not Working
**Check:**
- Serial baud rate: 115200
- Line endings: Use `\n` (newline)
- Command format: Uppercase, exact spelling
- Try `HELP` to see available commands

---

## Quick Reference: All Serial Commands

### PID Control
- `SET PID <kp> <ki> <kd>` - Set PID values
- `GET PID` - Show current PID values

### Calibration
- `CALIBRATE` - Run blocking calibration (keep robot still!)
- `SAVE_CAL` - Save calibration to NVS
- `LOAD_CAL` - Load calibration from NVS
- `CLEAR_CAL` - Clear stored calibration
- `GET_CAL_INFO` - Show calibration offsets

### Test Mode
- `RUN_SELF_CHECKS` - Run all self-check tests
- `TEST_MODE_ON` - Enable test mode (disable motors)
- `TEST_MODE_OFF` - Disable test mode (enable motors)

### System Info
- `GET_BOOT_TAG` - Show firmware version tag
- `GET_STATUS` - Show complete system status
- `HELP` - Show all commands

---

## Next Steps After Testing

1. **If all tests pass:** Proceed to physical robot assembly
2. **If issues found:** Document and fix before proceeding
3. **Update documentation:** Record findings in project docs
4. **Increment boot tag:** Update to next prototype number
5. **Commit changes:** Save working state to git

---

## Summary: Your Immediate Next Steps

1. ✅ **Upload firmware:** `pio run -t upload`
2. ✅ **Open serial monitor:** `pio device monitor`
3. ✅ **Test basic commands:** `HELP`, `GET_BOOT_TAG`, `GET_STATUS`
4. ✅ **Enable test mode:** `TEST_MODE_ON` (since no IMU yet)
5. ✅ **Run self-checks:** `RUN_SELF_CHECKS`
6. ⏳ **Wait for IMU:** Then proceed with calibration and full testing

---

## Files Reference

- **Firmware config:** `include/Config.h`
- **Test mode code:** `src/test_mode.cpp`, `include/test_mode.h`
- **Calibration code:** `src/IMU.cpp`, `include/IMU.h`
- **Telemetry grapher:** `tools/imu_telemetry_graph.py`
- **Dummy generator:** `tools/dummy_imu_generator.py`
- **Algorithm docs:** `tools/ALGORITHM_EXTRACTION.md`

---

**Good luck with your testing! 🚀**

