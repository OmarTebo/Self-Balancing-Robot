# IMU Telemetry Grapher

Real-time plotting tool for SBR IMU telemetry data using matplotlib.

## Features

- **Real-time plotting** of PITCH, ROLL, and YAW angles
- **Expected vs Actual comparison** - overlay expected values from CSV file
- **Statistics display** - mean, standard deviation, and range for each axis
- **Auto-scaling time window** - configurable time window (default: 10 seconds)
- **Auto-detect serial port** - automatically finds ESP32 serial port
- **Save plots** - save graphs to PNG file on close

## Installation

```bash
pip install pyserial matplotlib numpy
```

## Usage

### Basic usage (auto-detect port):
```bash
python tools/imu_telemetry_graph.py
```

### Specify serial port:
```bash
python tools/imu_telemetry_graph.py --port COM3
# or on Linux/Mac:
python tools/imu_telemetry_graph.py --port /dev/ttyUSB0
```

### With expected values overlay:
```bash
python tools/imu_telemetry_graph.py --expected-file tools/expected_values_example.csv
```

### Custom time window:
```bash
python tools/imu_telemetry_graph.py --window 20  # 20 second window
```

### Save plot on close:
```bash
python tools/imu_telemetry_graph.py --save
```

### All options:
```bash
python tools/imu_telemetry_graph.py \
    --port COM3 \
    --expected-file expected.csv \
    --window 15 \
    --save
```

## Expected Values CSV Format

The expected values file should be a CSV with the following columns:
- `time` - Time in seconds (relative to start)
- `pitch` - Expected pitch angle in degrees
- `roll` - Expected roll angle in degrees
- `yaw` - Expected yaw angle in degrees

Example:
```csv
time,pitch,roll,yaw
0.0,0.0,0.0,0.0
1.0,5.0,0.0,0.0
2.0,10.0,0.0,0.0
```

See `tools/expected_values_example.csv` for a complete example.

## Telemetry Format

The tool parses telemetry lines in the format:
```
PITCH:12.34 ROLL:-5.67 YAW:89.01
```

This matches the telemetry output from `BotController::update()` which prints at 50Hz.

## Graph Features

- **Three subplots**: One for each axis (Pitch, Roll, Yaw)
- **Actual values**: Solid colored lines (blue=pitch, red=roll, green=yaw)
- **Expected values**: Dashed lines (if CSV file provided)
- **Statistics box**: Shows mean, std dev, and range for each axis
- **Auto-scaling**: X-axis scrolls with time window, Y-axis shows full range

## Tips

1. **First run**: Let the robot sit still to verify zero readings are correct
2. **Calibration check**: After calibration, verify offsets are applied correctly
3. **Expected values**: Use for comparing actual behavior vs expected test patterns
4. **Save plots**: Use `--save` to capture graphs for documentation
5. **Time window**: Adjust `--window` based on your test duration

## Troubleshooting

- **No data**: Check serial port is correct and robot is sending telemetry
- **Port not found**: Use `--port` to specify exact port name
- **Expected values not showing**: Check CSV format matches example (time, pitch, roll, yaw columns)
- **Plot not updating**: Ensure robot is sending telemetry at 50Hz (check serial monitor)

