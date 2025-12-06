# Self-Balancing Robot (SBR)

A robust, feature-rich firmware for a two-wheeled self-balancing robot running on ESP32. Uses MPU6050 IMU with Kalman filtering, PID control, and supports both serial and BLE interfaces for telemetry and parameter tuning.

[![PlatformIO](https://img.shields.io/badge/Platform-ESP32-blue)](https://platformio.org/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

---

## Features

- **IMU Integration**: MPU6050 with Kalman filter fusion for stable angle estimation
- **PID Control**: Continuous-time PID controller with roll-axis balancing
- **Calibration Persistence**: NVS-stored calibration data with integrity checks
- **Test Mode**: Safe development mode with self-checks (motors disabled)
- **Multiple Interfaces**: Serial commands and BLE for remote control
- **Fixed-Timestep Loop**: 200Hz control loop with catch-up mechanism
- **Unit Tests**: Comprehensive test suite (34 tests) for algorithm verification
- **Development Tools**: Python utilities for telemetry plotting and PID tuning

---

## Hardware Requirements

- **MCU**: ESP32 (NodeMCU-32S or `esp32dev` board recommended)
- **IMU**: MPU6050 via I²C (SDA/SCL configurable in `Config.h`)
- **Motors**: Two NEMA stepper motors with A4988 drivers
- **Power**: Separate motor VMOT and logic 3.3V (common ground)
- **Optional**: LED matrix display (MAX7219) for status indication

Default pin mappings and constants are defined in `include/Config.h`.

---

## Quick Start

### Prerequisites

- [PlatformIO](https://platformio.org/) (VSCode extension or CLI)
- ESP32 USB driver installed
- Python 3.x (for development tools)

### Installation

```bash
# Clone repository
git clone https://github.com/OmarTebo/sbr.git
cd sbr

# Open in VSCode with PlatformIO extension, or use CLI:
pio run -t upload
```

### Initial Setup

1. **Configure hardware** (if needed): Edit `include/Config.h` for custom pin mappings
2. **Upload firmware**: `pio run -t upload`
3. **Open serial monitor**: `pio device monitor` (115200 baud)
4. **Calibrate IMU**: 
   - Place robot perfectly level
   - Send `CALIBRATE` via serial
   - Send `SAVE_CAL` to persist calibration

**⚠️ Safety**: Keep wheels off ground during initial tuning. Set A4988 current limits before enabling motors.

---

## Project Structure

```
sbr/
├── include/          # Public headers (API interfaces)
├── src/             # Implementation files
├── lib/MPU6050/     # MPU6050 driver and Kalman filter
├── test/            # Unit tests (Unity framework)
├── tools/           # Python development tools
│   ├── imu_telemetry_graph.py  # Real-time telemetry plotting
│   ├── pid_gui.py              # PID tuning GUI
│   └── dummy_imu_generator.py  # Test data generation
└── Documentation files:
    ├── DOCUMENTATION.md         # Technical API reference
    ├── DEVELOPER_GUIDE.md       # Development guide
    ├── PHASE_STATUS.md          # Implementation status
    └── TEST_FLOW_GUIDE.md       # Testing procedures
```

---

## Usage

### Serial Commands

```
# Calibration
CALIBRATE          # Run calibration (robot must be level!)
SAVE_CAL           # Save calibration to NVS
LOAD_CAL           # Load calibration from NVS
CLEAR_CAL          # Clear stored calibration
GET_CAL_INFO       # Show current calibration offsets

# System Info
GET_BOOT_TAG       # Show firmware version/tag
GET_STATUS         # Show system status
RUN_SELF_CHECKS    # Run diagnostic tests

# Test Mode
TEST_MODE_ON       # Enable test mode (disable motors)
TEST_MODE_OFF      # Disable test mode

# PID Control
GET PID            # Show current PID gains
SET PID <kp> <ki> <kd>  # Set PID gains
```

### BLE Interface

- **Service UUID**: `d1c6f3e0-9d3b-11ee-be56-0242ac120002`
- **Characteristics**: KP, KI, KD for PID parameter tuning
- **Device Name**: "SBR-Bot"

### Development Tools

```bash
# Real-time telemetry plotting
python tools/imu_telemetry_graph.py --port COM3

# Dummy data mode (algorithm verification)
python tools/imu_telemetry_graph.py --dummy

# PID tuning GUI
python tools/pid_gui.py
```

---

## PID Tuning

### Starting Values

- **Kp**: Start with `1.0` (proportional gain)
- **Ki**: Start with `0.0` (integral term - add later if needed)
- **Kd**: Start with `1.0` (derivative damping)

### Tuning Process

1. **Increase Kp** until robot responds to tilt
2. **Add Kd** to damp oscillations
3. **Add Ki** only if steady-state error persists (use sparingly)

### Units

- **Angles**: Degrees (°)
- **Angular velocity**: Degrees per second (°/s)
- **PID output**: Angular velocity in °/s (converted to motor steps/sec)

---

## Testing

### Unit Tests

```bash
# Run all unit tests
pio test -e esp32 --without-uploading

# Test results: 32/34 passing (94%)
# - PID tests: 10/10 ✅
# - Kalman tests: 8/10 ⚠️ (2 acceptable failures)
# - IMU tests: 14/14 ✅
```

### Hardware Testing

1. Run self-checks: `RUN_SELF_CHECKS`
2. Verify calibration: `GET_CAL_INFO`
3. Test motor directions (adjust flags in `Config.h` if needed)
4. Start with conservative PID gains
5. Monitor telemetry for stability

---

## Configuration

Key configuration options in `include/Config.h`:

- **Control Loop**: `CONTROL_LOOP_HZ` (default: 200Hz)
- **Motor Pins**: `LEFT_STEP_PIN`, `RIGHT_STEP_PIN`, etc.
- **Motor Direction**: `INVERT_LEFT_MOTOR`, `INVERT_RIGHT_MOTOR`, `SWAP_MOTORS`
- **PID Defaults**: `DEFAULT_PID_KP`, `DEFAULT_PID_KI`, `DEFAULT_PID_KD`
- **Test Mode**: `TEST_MODE_ENABLED` (compile-time flag)

---

## Documentation

- **[DOCUMENTATION.md](DOCUMENTATION.md)**: Complete API reference and technical details
- **[DEVELOPER_GUIDE.md](DEVELOPER_GUIDE.md)**: Comprehensive development guide (project state, workflow, planning)
- **[PHASE_STATUS.md](PHASE_STATUS.md)**: Implementation progress and roadmap
- **[TEST_FLOW_GUIDE.md](TEST_FLOW_GUIDE.md)**: Testing procedures and workflows

---

## Contributing

Contributions are welcome! Please:

1. Follow existing code style and structure
2. Add unit tests for new algorithms
3. Update documentation for API changes
4. Keep public headers minimal and well-documented
5. Document units clearly (degrees, deg/s, seconds)

---

## License

MIT License - see [LICENSE](LICENSE) file for details

---

## Acknowledgments

- MPU6050 driver and Kalman filter implementation
- AccelStepper library for motor control
- PlatformIO for build system
- Unity framework for unit testing

---

**Status**: Active development - See [PHASE_STATUS.md](PHASE_STATUS.md) for current progress
