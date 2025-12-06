#pragma once
#include <Arduino.h>

// Forward declarations
class IMU;
class MotorDriver;
class Display;
class BLEHandler;
class PIDController;

// Main self-check function
// Runs all self-checks and returns true if all pass
bool runSelfChecks(IMU &imu, MotorDriver &leftMotor, MotorDriver &rightMotor,
                   Display &display, BLEHandler &ble, PIDController &pid);

