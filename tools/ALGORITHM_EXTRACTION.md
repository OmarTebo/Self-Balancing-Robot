# IMU Algorithm Extraction for Expected Value Calculation

This document extracts the complete algorithm used by the SBR firmware to convert MPU6050 sensor data into pitch, roll, and yaw angles. Use this information to prompt another model (e.g., ChatGPT, Claude) to calculate expected values for the CSV file.

---

## Input Data Format

The MPU6050 sensor provides:
- **Acceleration** (3-axis): `ax`, `ay`, `az` in **g-force units** (1g = 9.81 m/s²)
- **Gyroscope** (3-axis): `gx`, `gy`, `gz` in **degrees/second**

**Sensor Characteristics:**
- Acceleration scale: `16384.0` LSB per g (for ±2g range)
- Gyroscope scale: `131.0` LSB per deg/s (for ±250 deg/s range)
- Update rate: `200 Hz` (control loop frequency)
- Timestep: `dt = 0.005 seconds` (1/200 Hz)

---

## Processing Pipeline

### Step 1: Raw to Physical Units Conversion

**Location:** `lib/MPU6050/MPU6050_Processing.cpp`

```cpp
// Acceleration conversion (g-force)
ax_g = (raw_ax - accel_offset_x) / 16384.0
ay_g = (raw_ay - accel_offset_y) / 16384.0
az_g = (raw_az - accel_offset_z) / 16384.0

// Gyroscope conversion (radians/second)
gx_rads = ((raw_gx - gyro_offset_x) / 131.0) * (π / 180.0)
gy_rads = ((raw_gy - gyro_offset_y) / 131.0) * (π / 180.0)
gz_rads = ((raw_gz - gyro_offset_z) / 131.0) * (π / 180.0)
```

**Note:** Offsets are typically zero unless gyroscope calibration is performed.

---

### Step 2: Angle Calculation (Two Methods)

The firmware uses **one of two methods** depending on whether a Kalman filter is attached:

#### Method A: Simple Atan2 (No Filter)

**Location:** `lib/MPU6050/MPU6050.cpp` (lines 59-61)

```cpp
roll  = atan2(ay_g, az_g) * 180.0 / π
pitch = atan2(-ax_g, sqrt(ay_g² + az_g²)) * 180.0 / π
yaw   = 0  // (not calculated from accelerometer alone)
```

**When used:** If no Kalman filter is attached to the MPU6050 object.

#### Method B: Kalman Filter Fusion (With Filter)

**Location:** `lib/MPU6050/MPU6050_Kalman.cpp` (lines 61-73)

**Step 2a: Calculate accelerometer-based angles**
```cpp
roll_acc  = atan2(ay_g, az_g) * 180.0 / π
pitch_acc = atan2(-ax_g, sqrt(ay_g² + az_g²)) * 180.0 / π
```

**Step 2b: Convert gyroscope to degrees/second**
```cpp
gx_dps = gx_rads * 180.0 / π  // Roll rate around x-axis
gy_dps = gy_rads * 180.0 / π  // Pitch rate around y-axis
```

**Step 2c: Kalman Filter Update (1D filter for each axis)**

For **Roll** (using `gx_dps` and `roll_acc`):
```cpp
roll_estimate = kalman_roll.update(gx_dps, roll_acc, dt)
```

For **Pitch** (using `gy_dps` and `pitch_acc`):
```cpp
pitch_estimate = kalman_pitch.update(gy_dps, pitch_acc, dt)
```

**Kalman Filter Parameters:**
- `Q_angle = 0.001` (process noise for angle)
- `Q_bias = 0.003` (process noise for gyro bias)
- `R_measure = 0.03` (measurement noise)
- Update rate: `200 Hz`

**Kalman Filter Algorithm (1D):**

```cpp
// Prediction step
angle += dt * (gyro_rate - bias)
P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle)
P[0][1] -= dt * P[1][1]
P[1][0] -= dt * P[1][1]
P[1][1] += Q_bias * dt

// Update step
S = P[0][0] + R_measure
K0 = P[0][0] / S
K1 = P[1][0] / S

y = accel_angle - angle
angle += K0 * y
bias  += K1 * y

// Update covariance
P[0][0] = P[0][0] - K0 * P[0][0]
P[0][1] = P[0][1] - K0 * P[0][1]
P[1][0] = P[1][0] - K1 * P[0][0]
P[1][1] = P[1][1] - K1 * P[0][1]
```

**When used:** If a Kalman filter is attached via `mpu->attachFilter(&kalman_filter)`.

**Note:** The current firmware code does **not** explicitly attach a Kalman filter in `IMU::begin()`, so it likely uses **Method A (Simple Atan2)**.

---

### Step 3: Calibration Offset Application

**Location:** `src/IMU.cpp` (lines 66-68)

```cpp
pitch_final = pitch - pitch_offset
roll_final  = roll - roll_offset
yaw_final   = yaw  // (no yaw offset applied)
```

**Calibration offsets:**
- `pitch_offset`: Stored in NVS, computed during calibration
- `roll_offset`: Stored in NVS, computed during calibration
- Default: `0.0` if no calibration exists

---

## Complete Algorithm Summary

### For Expected Value Calculation (Simple Case - No Kalman):

Given input acceleration `(ax_g, ay_g, az_g)` in g-force units:

```python
import math

def calculate_angles(ax_g, ay_g, az_g, pitch_offset=0.0, roll_offset=0.0):
    """
    Calculate pitch, roll, yaw from accelerometer data.
    
    Args:
        ax_g, ay_g, az_g: Acceleration in g-force units
        pitch_offset, roll_offset: Calibration offsets (default: 0.0)
    
    Returns:
        (pitch, roll, yaw) in degrees
    """
    # Calculate angles from accelerometer
    roll  = math.atan2(ay_g, az_g) * 180.0 / math.pi
    pitch = math.atan2(-ax_g, math.sqrt(ay_g**2 + az_g**2)) * 180.0 / math.pi
    yaw   = 0.0
    
    # Apply calibration offsets
    pitch -= pitch_offset
    roll  -= roll_offset
    
    return (pitch, roll, yaw)
```

### For Expected Value Calculation (With Kalman Filter):

This requires implementing the full Kalman filter algorithm as shown above. The filter needs to be initialized and updated at 200 Hz.

---

## Expected CSV Format

The expected values CSV file should have the following format:

```csv
time,pitch,roll,yaw
0.0,0.0,0.0,0.0
0.005,0.1,0.0,0.0
0.01,0.2,0.0,0.0
...
```

**Columns:**
- `time`: Time in seconds (relative to start, at 200 Hz = 0.005s intervals)
- `pitch`: Expected pitch angle in degrees
- `roll`: Expected roll angle in degrees
- `yaw`: Expected yaw angle in degrees (typically 0.0)

**Time intervals:** Should match control loop frequency (200 Hz = 0.005s between samples)

---

## Example: Generating Expected Values

### Scenario: Robot tilts 10 degrees around x-axis (roll)

**Input acceleration in g-force units (1.0 = 1g = 9.81 m/s²):**
- `ax_g = 0.0` (no acceleration in x)
- `ay_g = sin(10°) = 0.174` g (gravity component in y-axis)
- `az_g = cos(10°) = 0.985` g (gravity component in z-axis)

**Calculation:**
```python
roll = atan2(0.174, 0.985) * 180 / π = 10.0°
pitch = atan2(-0.0, sqrt(0.174² + 0.985²)) * 180 / π = 0.0°
yaw = 0.0°
```

**Note:** Acceleration values are in g-force units where 1.0 = 1g. When level, az = 1.0g (not 9.81g).

**Expected CSV row:**
```csv
time,pitch,roll,yaw
0.0,0.0,10.0,0.0
```

---

## Prompt Template for AI Model

Use this prompt to generate expected values:

```
I need to calculate expected IMU angle values (pitch, roll, yaw) for a self-balancing robot.

The robot uses an MPU6050 IMU sensor. The firmware calculates angles using this algorithm:

1. Input: Acceleration in g-force units (ax_g, ay_g, az_g)
2. Calculate angles:
   - roll = atan2(ay_g, az_g) * 180.0 / π
   - pitch = atan2(-ax_g, sqrt(ay_g² + az_g²)) * 180.0 / π
   - yaw = 0.0

3. Apply calibration offsets (if any):
   - pitch_final = pitch - pitch_offset
   - roll_final = roll - roll_offset

The control loop runs at 200 Hz (dt = 0.005 seconds).

Given a test scenario where [DESCRIBE MOTION], generate a CSV file with expected values:
- Format: time,pitch,roll,yaw
- Time starts at 0.0, increments by 0.005 seconds (200 Hz)
- Duration: [X] seconds
- Calibration offsets: pitch_offset=0.0, roll_offset=0.0 (unless specified)

Example scenario: Robot tilts 10 degrees around x-axis (roll) at t=1.0s, holds for 2 seconds, then returns to 0 degrees.
```

---

## Notes

1. **Kalman Filter:** The current firmware may not use Kalman filtering (no explicit attachment in code). Use the simple atan2 method unless you verify Kalman is enabled.

2. **Gyroscope:** Gyroscope data is used only if Kalman filtering is enabled. For simple atan2 method, only accelerometer data is needed.

3. **Yaw:** Yaw cannot be calculated from accelerometer alone (requires magnetometer or gyroscope integration). The firmware sets yaw = 0.0.

4. **Calibration:** Calibration offsets are applied after angle calculation. Default offsets are 0.0 unless calibration has been performed and saved.

5. **Update Rate:** All calculations assume 200 Hz update rate (0.005s timestep).

---

## References

- **MPU6050 Processing:** `lib/MPU6050/MPU6050_Processing.cpp`
- **Angle Calculation:** `lib/MPU6050/MPU6050.cpp` (lines 59-61)
- **Kalman Filter:** `lib/MPU6050/MPU6050_Kalman.cpp`
- **Calibration Application:** `src/IMU.cpp` (lines 66-68)
- **Config Constants:** `lib/MPU6050/MPU6050_Config.h`

