#!/usr/bin/env python3
"""
Dummy IMU Data Generator for SBR Testing
Generates realistic MPU6050 sensor data (acceleration and gyroscope) for testing
without physical IMU hardware.

This simulates:
- 3-axis acceleration (ax, ay, az) in g-force units
- 3-axis gyroscope (gx, gy, gz) in degrees/second
- Rotational velocity around x-axis (for roll control)

Usage:
    python dummy_imu_generator.py [--duration 10] [--pattern sine|step|noise|static]
    
Outputs to serial port in format compatible with MPU6050 library.
"""

import serial
import serial.tools.list_ports
import time
import math
import argparse
import numpy as np

# MPU6050 sensor characteristics
ACCEL_SCALE = 16384.0  # LSB per g (for ±2g range)
GYRO_SCALE = 131.0     # LSB per deg/s (for ±250 deg/s range)
UPDATE_RATE_HZ = 200   # Control loop frequency
DT = 1.0 / UPDATE_RATE_HZ

# Default patterns
PATTERNS = {
    'sine': 'Sine wave oscillation',
    'step': 'Step response',
    'noise': 'Random noise',
    'static': 'Static (zero)',
    'tilt': 'Gradual tilt and return'
}


def generate_sine_pattern(t, amplitude=10.0, frequency=0.5):
    """Generate sine wave pattern for roll angle"""
    roll_angle = amplitude * math.sin(2 * math.pi * frequency * t)
    # Convert angle to acceleration (assuming gravity)
    ax = 0.0  # No acceleration in x
    ay = 9.81 * math.sin(math.radians(roll_angle))  # Gravity component in y
    az = 9.81 * math.cos(math.radians(roll_angle))  # Gravity component in z
    
    # Gyroscope: derivative of angle (angular velocity)
    roll_rate = amplitude * 2 * math.pi * frequency * math.cos(2 * math.pi * frequency * t)
    
    return {
        'ax': ax,
        'ay': ay,
        'az': az,
        'gx': roll_rate,  # Roll rate around x-axis
        'gy': 0.0,
        'gz': 0.0,
        'roll': roll_angle,
        'pitch': 0.0,
        'yaw': 0.0
    }


def generate_step_pattern(t, step_time=2.0, step_angle=15.0):
    """Generate step response pattern"""
    if t < step_time:
        roll_angle = 0.0
        roll_rate = 0.0
    else:
        roll_angle = step_angle
        roll_rate = 0.0  # Step has no rate after settling
    
    ax = 0.0
    ay = 9.81 * math.sin(math.radians(roll_angle))
    az = 9.81 * math.cos(math.radians(roll_angle))
    
    return {
        'ax': ax,
        'ay': ay,
        'az': az,
        'gx': roll_rate,
        'gy': 0.0,
        'gz': 0.0,
        'roll': roll_angle,
        'pitch': 0.0,
        'yaw': 0.0
    }


def generate_noise_pattern(t, noise_level=2.0):
    """Generate random noise pattern"""
    roll_angle = np.random.normal(0, noise_level)
    roll_rate = np.random.normal(0, noise_level * 0.1)
    
    ax = np.random.normal(0, 0.1)
    ay = 9.81 * math.sin(math.radians(roll_angle)) + np.random.normal(0, 0.05)
    az = 9.81 * math.cos(math.radians(roll_angle)) + np.random.normal(0, 0.05)
    
    return {
        'ax': ax,
        'ay': ay,
        'az': az,
        'gx': roll_rate,
        'gy': np.random.normal(0, 0.1),
        'gz': np.random.normal(0, 0.1),
        'roll': roll_angle,
        'pitch': 0.0,
        'yaw': 0.0
    }


def generate_static_pattern(t):
    """Generate static (zero) pattern"""
    return {
        'ax': 0.0,
        'ay': 0.0,
        'az': 9.81,  # Gravity pointing down
        'gx': 0.0,
        'gy': 0.0,
        'gz': 0.0,
        'roll': 0.0,
        'pitch': 0.0,
        'yaw': 0.0
    }


def generate_tilt_pattern(t, tilt_duration=3.0, max_tilt=20.0):
    """Generate gradual tilt and return pattern"""
    if t < tilt_duration:
        # Gradual tilt
        roll_angle = (t / tilt_duration) * max_tilt
        roll_rate = max_tilt / tilt_duration
    elif t < tilt_duration * 2:
        # Hold
        roll_angle = max_tilt
        roll_rate = 0.0
    elif t < tilt_duration * 3:
        # Return
        roll_angle = max_tilt * (1 - (t - tilt_duration * 2) / tilt_duration)
        roll_rate = -max_tilt / tilt_duration
    else:
        # Settle
        roll_angle = 0.0
        roll_rate = 0.0
    
    ax = 0.0
    ay = 9.81 * math.sin(math.radians(roll_angle))
    az = 9.81 * math.cos(math.radians(roll_angle))
    
    return {
        'ax': ax,
        'ay': ay,
        'az': az,
        'gx': roll_rate,
        'gy': 0.0,
        'gz': 0.0,
        'roll': roll_angle,
        'pitch': 0.0,
        'yaw': 0.0
    }


def convert_to_raw(data):
    """Convert physical units to MPU6050 raw values"""
    # Add some sensor noise
    noise_scale = 0.02
    
    raw_ax = int((data['ax'] + np.random.normal(0, noise_scale)) * ACCEL_SCALE)
    raw_ay = int((data['ay'] + np.random.normal(0, noise_scale)) * ACCEL_SCALE)
    raw_az = int((data['az'] + np.random.normal(0, noise_scale)) * ACCEL_SCALE)
    
    raw_gx = int((data['gx'] + np.random.normal(0, noise_scale * 0.1)) * GYRO_SCALE)
    raw_gy = int((data['gy'] + np.random.normal(0, noise_scale * 0.1)) * GYRO_SCALE)
    raw_gz = int((data['gz'] + np.random.normal(0, noise_scale * 0.1)) * GYRO_SCALE)
    
    return {
        'ax': raw_ax,
        'ay': raw_ay,
        'az': raw_az,
        'gx': raw_gx,
        'gy': raw_gy,
        'gz': raw_gz
    }


def find_serial_port():
    """Auto-detect serial port"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'USB' in port.description.upper() or 'CH340' in port.description or 'CP210' in port.description:
            return port.device
    if ports:
        return ports[0].device
    return None


def main():
    parser = argparse.ArgumentParser(description='Dummy IMU Data Generator for SBR')
    parser.add_argument('--port', type=str, default=None,
                       help='Serial port (auto-detect if not specified)')
    parser.add_argument('--duration', type=float, default=10.0,
                       help='Duration in seconds (default: 10)')
    parser.add_argument('--pattern', type=str, default='sine',
                       choices=['sine', 'step', 'noise', 'static', 'tilt'],
                       help='Pattern type (default: sine)')
    parser.add_argument('--baud', type=int, default=115200,
                       help='Serial baud rate (default: 115200)')
    parser.add_argument('--output-csv', type=str, default=None,
                       help='Also save expected values to CSV file')
    
    args = parser.parse_args()
    
    # Select pattern generator
    pattern_generators = {
        'sine': lambda t: generate_sine_pattern(t, amplitude=10.0, frequency=0.5),
        'step': generate_step_pattern,
        'noise': generate_noise_pattern,
        'static': generate_static_pattern,
        'tilt': generate_tilt_pattern
    }
    
    generate = pattern_generators[args.pattern]
    
    # Find serial port
    if not args.port:
        args.port = find_serial_port()
        if not args.port:
            print("Error: No serial port found. Please specify --port")
            return
    
    print(f"Pattern: {args.pattern} ({PATTERNS[args.pattern]})")
    print(f"Duration: {args.duration} seconds")
    print(f"Update rate: {UPDATE_RATE_HZ} Hz")
    print(f"Connecting to {args.port}...")
    
    # Open serial port
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
        time.sleep(2)  # Wait for connection
        print("Connected! Generating data...")
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return
    
    # CSV output
    csv_data = []
    if args.output_csv:
        csv_data.append(['time', 'pitch', 'roll', 'yaw'])
    
    start_time = time.time()
    t = 0.0
    
    try:
        while t < args.duration:
            # Generate pattern data
            data = generate(t)
            
            # Convert to raw sensor values
            raw = convert_to_raw(data)
            
            # Send raw data (simulating MPU6050 I2C read)
            # Note: This is a simplified simulation. In reality, the ESP32 would
            # read from I2C, but for testing we can inject data differently.
            # For now, we'll just print what would be expected telemetry output.
            
            # Calculate expected angles (using same algorithm as firmware)
            # This matches the atan2 calculation from MPU6050.cpp
            roll_expected = math.atan2(data['ay'], data['az']) * 180.0 / math.pi
            pitch_expected = math.atan2(-data['ax'], math.sqrt(data['ay']**2 + data['az']**2)) * 180.0 / math.pi
            
            # Print telemetry format (what BotController outputs)
            telemetry = f"PITCH:{pitch_expected:.2f} ROLL:{roll_expected:.2f} YAW:0.00\n"
            ser.write(telemetry.encode('utf-8'))
            
            # Save to CSV if requested
            if args.output_csv:
                csv_data.append([t, pitch_expected, roll_expected, 0.0])
            
            # Wait for next update
            time.sleep(DT)
            t += DT
            
            # Progress indicator
            if int(t) % 1 == 0 and t > 0:
                print(f"Time: {t:.1f}s / {args.duration:.1f}s", end='\r')
    
    except KeyboardInterrupt:
        print("\nStopped by user")
    finally:
        ser.close()
        print("\nDisconnected")
        
        # Save CSV if requested
        if args.output_csv and csv_data:
            import csv
            with open(args.output_csv, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerows(csv_data)
            print(f"Expected values saved to {args.output_csv}")


if __name__ == "__main__":
    main()

