# Phase 7: BLE Mobile App - Implementation Plan

**Status:** ⏳ **NOT STARTED**  
**Priority:** **HIGH**  
**Purpose:** Flutter mobile app for remote control with tank controls and telemetry

---

## Overview

Create a Flutter mobile application that communicates with the SBR robot via Bluetooth Low Energy (BLE). The app will provide:
1. **Tank controls** - Direct left/right motor speed control
2. **Control modes** - AUTO (PID only), MANUAL (tank only), MIXED (tank + PID)
3. **Real-time telemetry** - IMU angles, motor speeds, system status
4. **PID tuning** - Extend existing BLE PID characteristics
5. **System control** - Calibration, test mode, emergency stop

---

## Firmware Requirements

### 1. Extend BLEHandler

**New BLE Characteristics:**
```cpp
// Tank Control
#define CHAR_LEFT_MOTOR_UUID    "d1c6f3e4-9d3b-11ee-be56-0242ac120002"
#define CHAR_RIGHT_MOTOR_UUID   "d1c6f3e5-9d3b-11ee-be56-0242ac120002"
#define CHAR_CONTROL_MODE_UUID   "d1c6f3e6-9d3b-11ee-be56-0242ac120002"
#define CHAR_TANK_ENABLE_UUID    "d1c6f3e7-9d3b-11ee-be56-0242ac120002"

// Telemetry Streaming
#define CHAR_TELEMETRY_UUID      "d1c6f3e8-9d3b-11ee-be56-0242ac120002"

// System Control
#define CHAR_EMERGENCY_STOP_UUID "d1c6f3e9-9d3b-11ee-be56-0242ac120002"
#define CHAR_CALIBRATE_UUID      "d1c6f3ea-9d3b-11ee-be56-0242ac120002"
```

**BLEHandler.h additions:**
```cpp
enum ControlMode {
  MODE_AUTO = 0,   // Pure PID control
  MODE_MANUAL = 1, // Pure tank control
  MODE_MIXED = 2   // Tank + PID corrections
};

struct TankControl {
  float leftSpeed;   // -100 to +100 (%)
  float rightSpeed;  // -100 to +100 (%)
  ControlMode mode;
  bool enabled;
};

bool takePendingTankControl(TankControl &out);
bool hasEmergencyStop();
void clearEmergencyStop();
```

**BLEHandler.cpp changes:**
- Add characteristics for tank control
- Add telemetry notification characteristic
- Implement thread-safe tank control handling (similar to PID params)
- Add emergency stop flag
- Add calibration trigger

### 2. Extend BotController

**BotController.h additions:**
```cpp
enum ControlMode {
  MODE_AUTO = 0,
  MODE_MANUAL = 1,
  MODE_MIXED = 2
};

// In update() method, check for tank control commands
void update(float dt); // Modified to handle control modes
```

**BotController.cpp changes:**
```cpp
void BotController::update(float dt) {
  // ... existing IMU and BLE PID code ...
  
  // Check for tank control commands
  TankControl tank;
  if (ble.takePendingTankControl(tank)) {
    // Update control mode and tank commands
    currentControlMode = tank.mode;
    tankLeftSpeed = tank.leftSpeed;
    tankRightSpeed = tank.rightSpeed;
    tankControlEnabled = tank.enabled;
  }
  
  // Check emergency stop
  if (ble.hasEmergencyStop()) {
    leftMotor.setSpeedStepsPerSec(0);
    rightMotor.setSpeedStepsPerSec(0);
    return; // Skip rest of update
  }
  
  // ... existing PID computation ...
  
  // Apply control mode logic
  float leftFinalSpeed, rightFinalSpeed;
  
  switch (currentControlMode) {
    case MODE_AUTO:
      // Pure PID (current behavior)
      leftFinalSpeed = rollStepsPerSec * leftSign;
      rightFinalSpeed = rollStepsPerSec * rightSign;
      break;
      
    case MODE_MANUAL:
      // Pure tank control (no PID)
      leftFinalSpeed = (tankLeftSpeed / 100.0f) * maxMotorSpeed;
      rightFinalSpeed = (tankRightSpeed / 100.0f) * maxMotorSpeed;
      break;
      
    case MODE_MIXED:
      // Tank + PID corrections
      float tankLeftSteps = (tankLeftSpeed / 100.0f) * maxMotorSpeed;
      float tankRightSteps = (tankRightSpeed / 100.0f) * maxMotorSpeed;
      float pidLeftSteps = rollStepsPerSec * leftSign;
      float pidRightSteps = rollStepsPerSec * rightSign;
      
      // Mix: user command + PID correction
      leftFinalSpeed = tankLeftSteps + pidLeftSteps;
      rightFinalSpeed = tankRightSteps + pidRightSteps;
      
      // Clamp to max speed
      leftFinalSpeed = constrain(leftFinalSpeed, -maxMotorSpeed, maxMotorSpeed);
      rightFinalSpeed = constrain(rightFinalSpeed, -maxMotorSpeed, maxMotorSpeed);
      break;
  }
  
  // Apply motor speeds
  leftMotor.setSpeedStepsPerSec(leftFinalSpeed);
  rightMotor.setSpeedStepsPerSec(rightFinalSpeed);
  
  // ... rest of update ...
}
```

**Telemetry Streaming:**
```cpp
// In BotController::update(), periodically send telemetry
static unsigned long lastTelemetryMs = 0;
if (millis() - lastTelemetryMs >= 50) { // 20 Hz
  lastTelemetryMs = millis();
  
  TelemetryData telemetry;
  telemetry.roll = imu.getRoll();
  telemetry.pitch = imu.getPitch();
  telemetry.yaw = imu.getYaw();
  telemetry.leftSpeed = leftMotor.getCurrentSpeed();
  telemetry.rightSpeed = rightMotor.getCurrentSpeed();
  telemetry.controlMode = currentControlMode;
  
  ble.sendTelemetry(telemetry); // Notify all connected clients
}
```

### 3. Telemetry Data Structure

```cpp
struct TelemetryData {
  float roll;        // degrees
  float pitch;       // degrees
  float yaw;         // degrees
  float leftSpeed;   // steps/sec
  float rightSpeed;  // steps/sec
  uint8_t controlMode; // 0=AUTO, 1=MANUAL, 2=MIXED
  bool emergencyStop;  // true if emergency stop active
};

// Serialize to JSON string for BLE transmission
String serializeTelemetry(const TelemetryData &data) {
  return String("{") +
    "\"roll\":" + String(data.roll, 2) + "," +
    "\"pitch\":" + String(data.pitch, 2) + "," +
    "\"yaw\":" + String(data.yaw, 2) + "," +
    "\"leftSpeed\":" + String(data.leftSpeed, 1) + "," +
    "\"rightSpeed\":" + String(data.rightSpeed, 1) + "," +
    "\"mode\":" + String(data.controlMode) + "," +
    "\"emergency\":" + String(data.emergencyStop ? "true" : "false") +
    "}";
}
```

---

## Flutter App Requirements

### 1. Project Structure

```
mobile_app/
├── lib/
│   ├── main.dart                 # App entry point
│   ├── models/
│   │   ├── telemetry_data.dart   # Telemetry data model
│   │   ├── control_mode.dart     # Control mode enum
│   │   └── pid_params.dart        # PID parameters model
│   ├── services/
│   │   ├── ble_service.dart      # BLE communication
│   │   └── telemetry_parser.dart # Parse telemetry JSON
│   ├── ui/
│   │   ├── home_screen.dart      # Main screen
│   │   ├── tank_control.dart     # Tank control UI
│   │   ├── telemetry_view.dart   # Telemetry display
│   │   ├── pid_tuning.dart       # PID parameter adjustment
│   │   └── settings.dart        # App settings
│   └── widgets/
│       ├── joystick.dart         # Tank control joystick
│       └── telemetry_card.dart   # Telemetry display card
├── pubspec.yaml
└── README.md
```

### 2. Dependencies (pubspec.yaml)

```yaml
dependencies:
  flutter:
    sdk: flutter
  flutter_blue_plus: ^1.30.0  # BLE communication
  provider: ^6.1.1            # State management
  fl_chart: ^0.66.0           # Telemetry plotting (optional)
```

### 3. BLE Service Implementation

**lib/services/ble_service.dart:**
```dart
import 'package:flutter_blue_plus/flutter_blue_plus.dart';

class BLEService {
  static const String SERVICE_UUID = "d1c6f3e0-9d3b-11ee-be56-0242ac120002";
  static const String DEVICE_NAME = "SBR-Bot";
  
  // Characteristics
  static const String CHAR_KP_UUID = "d1c6f3e1-9d3b-11ee-be56-0242ac120002";
  static const String CHAR_KI_UUID = "d1c6f3e2-9d3b-11ee-be56-0242ac120002";
  static const String CHAR_KD_UUID = "d1c6f3e3-9d3b-11ee-be56-0242ac120002";
  static const String CHAR_LEFT_MOTOR_UUID = "d1c6f3e4-9d3b-11ee-be56-0242ac120002";
  static const String CHAR_RIGHT_MOTOR_UUID = "d1c6f3e5-9d3b-11ee-be56-0242ac120002";
  static const String CHAR_CONTROL_MODE_UUID = "d1c6f3e6-9d3b-11ee-be56-0242ac120002";
  static const String CHAR_TELEMETRY_UUID = "d1c6f3e8-9d3b-11ee-be56-0242ac120002";
  static const String CHAR_EMERGENCY_STOP_UUID = "d1c6f3e9-9d3b-11ee-be56-0242ac120002";
  
  BluetoothDevice? _device;
  BluetoothCharacteristic? _kpChar;
  BluetoothCharacteristic? _kiChar;
  BluetoothCharacteristic? _kdChar;
  BluetoothCharacteristic? _leftMotorChar;
  BluetoothCharacteristic? _rightMotorChar;
  BluetoothCharacteristic? _controlModeChar;
  BluetoothCharacteristic? _telemetryChar;
  BluetoothCharacteristic? _emergencyStopChar;
  
  Future<bool> connect() async {
    // Scan for device
    await FlutterBluePlus.startScan(timeout: Duration(seconds: 4));
    FlutterBluePlus.scanResults.listen((results) {
      for (ScanResult result in results) {
        if (result.device.name == DEVICE_NAME) {
          _device = result.device;
          FlutterBluePlus.stopScan();
        }
      }
    });
    
    // Connect
    if (_device != null) {
      await _device!.connect();
      await _discoverServices();
      return true;
    }
    return false;
  }
  
  Future<void> _discoverServices() async {
    List<BluetoothService> services = await _device!.discoverServices();
    for (BluetoothService service in services) {
      if (service.uuid.toString() == SERVICE_UUID) {
        for (BluetoothCharacteristic char in service.characteristics) {
          String uuid = char.uuid.toString();
          if (uuid == CHAR_KP_UUID) _kpChar = char;
          else if (uuid == CHAR_KI_UUID) _kiChar = char;
          else if (uuid == CHAR_KD_UUID) _kdChar = char;
          else if (uuid == CHAR_LEFT_MOTOR_UUID) _leftMotorChar = char;
          else if (uuid == CHAR_RIGHT_MOTOR_UUID) _rightMotorChar = char;
          else if (uuid == CHAR_CONTROL_MODE_UUID) _controlModeChar = char;
          else if (uuid == CHAR_TELEMETRY_UUID) {
            _telemetryChar = char;
            await char.setNotifyValue(true);
            char.onValueReceived.listen((value) {
              // Parse telemetry JSON
              _onTelemetryReceived(value);
            });
          }
          else if (uuid == CHAR_EMERGENCY_STOP_UUID) _emergencyStopChar = char;
        }
      }
    }
  }
  
  // Send tank control commands
  Future<void> setTankControl(double leftSpeed, double rightSpeed) async {
    if (_leftMotorChar != null && _rightMotorChar != null) {
      await _leftMotorChar!.write([leftSpeed.round()]);
      await _rightMotorChar!.write([rightSpeed.round()]);
    }
  }
  
  // Set control mode
  Future<void> setControlMode(int mode) async {
    if (_controlModeChar != null) {
      await _controlModeChar!.write([mode]);
    }
  }
  
  // Emergency stop
  Future<void> emergencyStop() async {
    if (_emergencyStopChar != null) {
      await _emergencyStopChar!.write([1]);
    }
  }
  
  // PID tuning (extend existing)
  Future<void> setPID(double kp, double ki, double kd) async {
    if (_kpChar != null && _kiChar != null && _kdChar != null) {
      await _kpChar!.write(kp.toString().codeUnits);
      await _kiChar!.write(ki.toString().codeUnits);
      await _kdChar!.write(kd.toString().codeUnits);
    }
  }
  
  void _onTelemetryReceived(List<int> value) {
    // Parse JSON telemetry
    String json = String.fromCharCodes(value);
    // Emit to UI via stream or provider
  }
}
```

### 4. UI Components

**Tank Control UI:**
- Dual joysticks or sliders for left/right motor speed
- Range: -100% to +100%
- Control mode selector (AUTO/MANUAL/MIXED)
- Emergency stop button (large, prominent)

**Telemetry Display:**
- Real-time roll/pitch/yaw angles
- Motor speeds (left/right)
- Control mode indicator
- Connection status
- Optional: Plotting with fl_chart

**PID Tuning Panel:**
- Sliders for kp, ki, kd
- Current values display
- Apply button

---

## Control Mode Details

### AUTO Mode (Pure PID)
- User provides no input
- Robot balances using PID only
- Target: 0° roll (balanced)

### MANUAL Mode (Pure Tank)
- User controls left/right motors directly
- No PID corrections
- Robot can fall over if not balanced manually
- Useful for: Testing motors, manual driving practice

### MIXED Mode (Tank + PID)
- User provides base speed via tank controls
- PID adds corrections to maintain balance
- Example:
  - User drives forward at 50% speed
  - PID adds ±5% corrections to keep balanced
  - Result: Robot moves forward while staying balanced
- **This is the key feature** - allows driving while auto-balancing

---

## Implementation Steps

### Step 1: Firmware BLE Extensions
1. Add new BLE characteristics to `BLEHandler.h/cpp`
2. Implement tank control command handling
3. Add telemetry streaming
4. Test with BLE scanner app

### Step 2: BotController Control Modes
1. Add control mode enum
2. Implement mode switching logic
3. Implement mixing algorithm (MIXED mode)
4. Test with serial commands (mock BLE)

### Step 3: Flutter App Foundation
1. Set up Flutter project
2. Implement BLE service
3. Create basic UI structure
4. Test connection and basic commands

### Step 4: Tank Control UI
1. Implement joystick/slider controls
2. Add control mode selector
3. Test tank control in MANUAL mode

### Step 5: Telemetry Display
1. Parse telemetry JSON
2. Display real-time data
3. Optional: Add plotting

### Step 6: Integration & Testing
1. Test all control modes
2. Verify MIXED mode works correctly
3. Test emergency stop
4. Polish UI/UX

---

## Testing Strategy

### Firmware Testing
1. **Unit test:** Control mode logic
2. **Integration test:** BLE characteristics work correctly
3. **Hardware test:** Tank control moves motors correctly
4. **Hardware test:** MIXED mode balances while driving

### App Testing
1. **Connection test:** App connects to robot
2. **Command test:** Tank controls work
3. **Telemetry test:** Data updates in real-time
4. **Mode test:** Switching modes works
5. **Emergency stop test:** Stops motors immediately

---

## Success Criteria

- [ ] App connects to robot via BLE
- [ ] Tank controls move motors correctly
- [ ] AUTO mode works (pure PID)
- [ ] MANUAL mode works (pure tank)
- [ ] MIXED mode works (tank + PID corrections)
- [ ] Telemetry streams in real-time
- [ ] PID tuning works via BLE
- [ ] Emergency stop works
- [ ] UI is intuitive and responsive

---

## Future Enhancements (Post-Phase 7)

- Battery level monitoring
- Waypoint navigation
- Recording/playback of control sequences
- Advanced PID tuning (auto-tuning)
- Camera feed (if ESP32-CAM added)
- Multi-robot support

---

**Priority:** **HIGH**  
**Estimated Effort:** 
- Firmware: 2-3 days
- Flutter app: 3-5 days
- Testing: 1-2 days
**Total:** ~1-2 weeks

**Dependencies:** None (can start immediately)

---

**Last Updated:** After Phase 4 completion  
**Next Steps:** Begin firmware BLE extensions

