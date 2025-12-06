#!/usr/bin/env python3
"""
SBR IMU Telemetry Grapher
- Real-time plotting of PITCH, ROLL, YAW values from serial telemetry
- Dummy data mode for algorithm verification (like MATLAB simulation)
- Optional expected value overlay for comparison
- Uses matplotlib for high-quality graphs
- Parses telemetry format: "PITCH:%.2f ROLL:%.2f YAW:%.2f"

Requirements: pip install pyserial matplotlib numpy

Usage:
    # Real MPU data mode (default)
    python imu_telemetry_graph.py [--port COM3] [--expected-file expected.csv]
    
    # Dummy data mode (algorithm verification)
    python imu_telemetry_graph.py --dummy [--pattern sine|step|tilt] [--expected-file expected.csv]
    
    --dummy: Use dummy data instead of real serial data (for algorithm testing)
    --pattern: Pattern type for dummy data (sine, step, tilt) - only with --dummy
    --port: Serial port (default: auto-detect, ignored in dummy mode)
    --expected-file: CSV file with expected values (time, pitch, roll, yaw columns)
    --window: Time window in seconds (default: 10)
    --save: Save plot to file (default: False)
"""

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import re
import time
import argparse
import csv
import math
from collections import deque
from threading import Thread, Event
import queue

# Configuration
DEFAULT_BAUD = 115200
DEFAULT_WINDOW_SEC = 10  # Time window to display
UPDATE_INTERVAL_MS = 50  # Update plot every 50ms
TELEMETRY_FREQ_HZ = 50   # Expected telemetry frequency (from BotController)

# Telemetry regex: matches "PITCH:%.2f ROLL:%.2f YAW:%.2f"
TELEMETRY_REGEX = re.compile(r'PITCH[:=]\s*([-\d.]+)\s+ROLL[:=]\s*([-\d.]+)\s+YAW[:=]\s*([-\d.]+)', re.IGNORECASE)

# Update rate for dummy data (matches control loop: 200 Hz)
DUMMY_UPDATE_RATE_HZ = 200
DUMMY_DT = 1.0 / DUMMY_UPDATE_RATE_HZ


class DummyDataGenerator:
    """Generates dummy IMU sensor data for algorithm verification"""
    
    def __init__(self, pattern='sine'):
        self.pattern = pattern
        self.t = 0.0
        
    def generate_sensor_data(self, t):
        """Generate acceleration data in g-force units (1.0 = 1g)"""
        if self.pattern == 'sine':
            # Sine wave: 10 degree amplitude, 0.5 Hz
            roll_angle = 10.0 * math.sin(2 * math.pi * 0.5 * t)
            ax = 0.0
            ay = math.sin(math.radians(roll_angle))
            az = math.cos(math.radians(roll_angle))
            
        elif self.pattern == 'step':
            # Step response: 15 degrees at t=2s
            if t < 2.0:
                roll_angle = 0.0
            else:
                roll_angle = 15.0
            ax = 0.0
            ay = math.sin(math.radians(roll_angle))
            az = math.cos(math.radians(roll_angle))
            
        elif self.pattern == 'tilt':
            # Gradual tilt: 0->20->0 degrees
            if t < 3.0:
                roll_angle = (t / 3.0) * 20.0
            elif t < 6.0:
                roll_angle = 20.0
            elif t < 9.0:
                roll_angle = 20.0 * (1 - (t - 6.0) / 3.0)
            else:
                roll_angle = 0.0
            ax = 0.0
            ay = math.sin(math.radians(roll_angle))
            az = math.cos(math.radians(roll_angle))
            
        else:  # static
            ax = 0.0
            ay = 0.0
            az = 1.0  # 1g pointing down
            
        return {'ax': ax, 'ay': ay, 'az': az}
    
    @staticmethod
    def calculate_angles(ax_g, ay_g, az_g, pitch_offset=0.0, roll_offset=0.0):
        """
        Calculate angles from acceleration using same algorithm as firmware.
        This matches the atan2 calculation from MPU6050.cpp
        """
        # Same algorithm as firmware (simple atan2 method)
        roll = math.atan2(ay_g, az_g) * 180.0 / math.pi
        pitch = math.atan2(-ax_g, math.sqrt(ay_g**2 + az_g**2)) * 180.0 / math.pi
        yaw = 0.0
        
        # Apply calibration offsets (same as firmware)
        pitch -= pitch_offset
        roll -= roll_offset
        
        return (pitch, roll, yaw)


class SerialReader(Thread):
    """Background thread to read serial data"""
    def __init__(self, ser, data_queue, stop_event):
        super().__init__(daemon=True)
        self.ser = ser
        self.data_queue = data_queue
        self.stop_event = stop_event
        
    def run(self):
        buffer = b""
        while not self.stop_event.is_set():
            try:
                if self.ser.in_waiting:
                    data = self.ser.read(self.ser.in_waiting)
                    buffer += data
                    
                    # Process complete lines
                    while b"\n" in buffer:
                        line, buffer = buffer.split(b"\n", 1)
                        try:
                            text = line.decode('utf-8', errors='ignore').strip()
                            match = TELEMETRY_REGEX.search(text)
                            if match:
                                pitch = float(match.group(1))
                                roll = float(match.group(2))
                                yaw = float(match.group(3))
                                self.data_queue.put((time.time(), pitch, roll, yaw))
                        except (ValueError, UnicodeDecodeError):
                            pass
                else:
                    time.sleep(0.01)
            except Exception as e:
                print(f"Serial error: {e}")
                break


class TelemetryGrapher:
    def __init__(self, port=None, expected_file=None, window_sec=10, save_plot=False, 
                 dummy_mode=False, dummy_pattern='sine'):
        self.port = port
        self.expected_file = expected_file
        self.window_sec = window_sec
        self.save_plot = save_plot
        self.dummy_mode = dummy_mode
        self.dummy_pattern = dummy_pattern
        
        # Data storage (time, pitch, roll, yaw)
        update_freq = DUMMY_UPDATE_RATE_HZ if dummy_mode else TELEMETRY_FREQ_HZ
        maxlen = int(window_sec * update_freq * 2)
        self.times = deque(maxlen=maxlen)
        self.pitch = deque(maxlen=maxlen)
        self.roll = deque(maxlen=maxlen)
        self.yaw = deque(maxlen=maxlen)
        
        # Dummy mode: also store sensor data
        if dummy_mode:
            self.ax_data = deque(maxlen=maxlen)
            self.ay_data = deque(maxlen=maxlen)
            self.az_data = deque(maxlen=maxlen)
            self.dummy_generator = DummyDataGenerator(pattern=dummy_pattern)
        else:
            self.ax_data = None
            self.ay_data = None
            self.az_data = None
            self.dummy_generator = None
        
        # Expected values (if provided)
        self.expected_times = []
        self.expected_pitch = []
        self.expected_roll = []
        self.expected_yaw = []
        
        # Serial connection
        self.ser = None
        self.reader = None
        self.data_queue = queue.Queue()
        self.stop_event = Event()
        
        # Matplotlib setup
        if dummy_mode:
            # In dummy mode, show sensor data + angles (6 subplots)
            self.fig, self.axes = plt.subplots(6, 1, figsize=(12, 14))
            self.fig.suptitle(f'SBR IMU Algorithm Verification - Dummy Data ({dummy_pattern} pattern)', 
                            fontsize=14, fontweight='bold')
        else:
            # Real mode: just angles (3 subplots)
            self.fig, self.axes = plt.subplots(3, 1, figsize=(12, 10))
            self.fig.suptitle('SBR IMU Telemetry - Real-time', fontsize=14, fontweight='bold')
        
        # Setup subplots
        if dummy_mode:
            # Dummy mode: sensor data + angles
            self.ax_ax = self.axes[0]
            self.ax_ay = self.axes[1]
            self.ax_az = self.axes[2]
            self.ax_pitch = self.axes[3]
            self.ax_roll = self.axes[4]
            self.ax_yaw = self.axes[5]
            
            # Sensor data plots
            self.ax_ax.set_ylabel('ax (g)', fontweight='bold')
            self.ax_ax.set_title('Acceleration X (g-force)', fontsize=11)
            self.ax_ax.grid(True, alpha=0.3)
            self.ax_ax.set_ylim(-2, 2)
            
            self.ax_ay.set_ylabel('ay (g)', fontweight='bold')
            self.ax_ay.set_title('Acceleration Y (g-force)', fontsize=11)
            self.ax_ay.grid(True, alpha=0.3)
            self.ax_ay.set_ylim(-2, 2)
            
            self.ax_az.set_ylabel('az (g)', fontweight='bold')
            self.ax_az.set_title('Acceleration Z (g-force)', fontsize=11)
            self.ax_az.grid(True, alpha=0.3)
            self.ax_az.set_ylim(-2, 2)
            
            # Angle plots
            self.ax_pitch.set_ylabel('Pitch (deg)', fontweight='bold')
            self.ax_pitch.set_title('Pitch Angle (Calculated)', fontsize=11)
            self.ax_pitch.grid(True, alpha=0.3)
            self.ax_pitch.set_ylim(-90, 90)
            
            self.ax_roll.set_ylabel('Roll (deg)', fontweight='bold')
            self.ax_roll.set_title('Roll Angle (Calculated)', fontsize=11)
            self.ax_roll.grid(True, alpha=0.3)
            self.ax_roll.set_ylim(-90, 90)
            
            self.ax_yaw.set_ylabel('Yaw (deg)', fontweight='bold')
            self.ax_yaw.set_xlabel('Time (seconds)', fontweight='bold')
            self.ax_yaw.set_title('Yaw Angle (Calculated)', fontsize=11)
            self.ax_yaw.grid(True, alpha=0.3)
            self.ax_yaw.set_ylim(-180, 180)
        else:
            # Real mode: just angles
            self.ax_pitch = self.axes[0]
            self.ax_roll = self.axes[1]
            self.ax_yaw = self.axes[2]
            
            self.ax_pitch.set_ylabel('Pitch (deg)', fontweight='bold')
            self.ax_pitch.set_title('Pitch Angle', fontsize=11)
            self.ax_pitch.grid(True, alpha=0.3)
            self.ax_pitch.set_ylim(-90, 90)
            
            self.ax_roll.set_ylabel('Roll (deg)', fontweight='bold')
            self.ax_roll.set_title('Roll Angle', fontsize=11)
            self.ax_roll.grid(True, alpha=0.3)
            self.ax_roll.set_ylim(-90, 90)
            
            self.ax_yaw.set_ylabel('Yaw (deg)', fontweight='bold')
            self.ax_yaw.set_xlabel('Time (seconds)', fontweight='bold')
            self.ax_yaw.set_title('Yaw Angle', fontsize=11)
            self.ax_yaw.grid(True, alpha=0.3)
            self.ax_yaw.set_ylim(-180, 180)
        
        # Plot lines
        if dummy_mode:
            # Sensor data lines
            self.line_ax, = self.ax_ax.plot([], [], 'b-', linewidth=1.5, label='ax')
            self.line_ay, = self.ax_ay.plot([], [], 'r-', linewidth=1.5, label='ay')
            self.line_az, = self.ax_az.plot([], [], 'g-', linewidth=1.5, label='az')
            # Angle lines (calculated from sensor data)
            self.line_pitch, = self.ax_pitch.plot([], [], 'b-', linewidth=1.5, label='Calculated')
            self.line_roll, = self.ax_roll.plot([], [], 'r-', linewidth=1.5, label='Calculated')
            self.line_yaw, = self.ax_yaw.plot([], [], 'g-', linewidth=1.5, label='Calculated')
        else:
            # Real mode: just angles
            self.line_pitch, = self.ax_pitch.plot([], [], 'b-', linewidth=1.5, label='Actual')
            self.line_roll, = self.ax_roll.plot([], [], 'r-', linewidth=1.5, label='Actual')
            self.line_yaw, = self.ax_yaw.plot([], [], 'g-', linewidth=1.5, label='Actual')
        
        # Expected value lines (if provided)
        self.line_pitch_exp = None
        self.line_roll_exp = None
        self.line_yaw_exp = None
        
        # Statistics text
        if dummy_mode:
            # Sensor data statistics
            self.stats_text_ax = self.ax_ax.text(0.02, 0.95, '', transform=self.ax_ax.transAxes,
                                                 verticalalignment='top', fontsize=9,
                                                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
            self.stats_text_ay = self.ax_ay.text(0.02, 0.95, '', transform=self.ax_ay.transAxes,
                                                 verticalalignment='top', fontsize=9,
                                                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
            self.stats_text_az = self.ax_az.text(0.02, 0.95, '', transform=self.ax_az.transAxes,
                                                 verticalalignment='top', fontsize=9,
                                                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        self.stats_text_pitch = self.ax_pitch.text(0.02, 0.95, '', transform=self.ax_pitch.transAxes,
                                                    verticalalignment='top', fontsize=9,
                                                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        self.stats_text_roll = self.ax_roll.text(0.02, 0.95, '', transform=self.ax_roll.transAxes,
                                                   verticalalignment='top', fontsize=9,
                                                   bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        self.stats_text_yaw = self.ax_yaw.text(0.02, 0.95, '', transform=self.ax_yaw.transAxes,
                                                verticalalignment='top', fontsize=9,
                                                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # Load expected values if file provided
        if expected_file:
            self.load_expected_values(expected_file)
        
        # Start time reference
        self.start_time = None
        self.dummy_t = 0.0  # Dummy mode time counter
        
    def load_expected_values(self, filename):
        """Load expected values from CSV file"""
        try:
            with open(filename, 'r') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    try:
                        t = float(row.get('time', 0))
                        p = float(row.get('pitch', 0))
                        r = float(row.get('roll', 0))
                        y = float(row.get('yaw', 0))
                        self.expected_times.append(t)
                        self.expected_pitch.append(p)
                        self.expected_roll.append(r)
                        self.expected_yaw.append(y)
                    except (ValueError, KeyError):
                        # Skip invalid rows
                        continue
            print(f"Loaded {len(self.expected_times)} expected value points from {filename}")
            
            # Create expected value lines
            if self.expected_times:
                self.line_pitch_exp, = self.ax_pitch.plot([], [], 'b--', linewidth=1, alpha=0.6, label='Expected')
                self.line_roll_exp, = self.ax_roll.plot([], [], 'r--', linewidth=1, alpha=0.6, label='Expected')
                self.line_yaw_exp, = self.ax_yaw.plot([], [], 'g--', linewidth=1, alpha=0.6, label='Expected')
                
                # Add legends
                self.ax_pitch.legend(loc='upper right')
                self.ax_roll.legend(loc='upper right')
                self.ax_yaw.legend(loc='upper right')
        except Exception as e:
            print(f"Error loading expected values: {e}")
    
    def find_serial_port(self):
        """Auto-detect serial port"""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Prefer ports that look like ESP32/Arduino
            if 'USB' in port.description.upper() or 'CH340' in port.description or 'CP210' in port.description:
                return port.device
        # Fallback to first available port
        if ports:
            return ports[0].device
        return None
    
    def connect_serial(self):
        """Connect to serial port"""
        if not self.port:
            self.port = self.find_serial_port()
            if not self.port:
                raise Exception("No serial port found. Please specify --port")
        
        print(f"Connecting to {self.port} at {DEFAULT_BAUD} baud...")
        self.ser = serial.Serial(self.port, DEFAULT_BAUD, timeout=0.1)
        time.sleep(2)  # Wait for connection to stabilize
        
        # Start reader thread
        self.reader = SerialReader(self.ser, self.data_queue, self.stop_event)
        self.reader.start()
        print("Connected! Waiting for telemetry data...")
        
    def disconnect_serial(self):
        """Disconnect from serial port"""
        self.stop_event.set()
        if self.reader:
            self.reader.join(timeout=1)
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("Disconnected")
    
    def update_plot(self, frame):
        """Update plot with new data (called by matplotlib animation)"""
        new_data = False
        
        if self.dummy_mode:
            # Generate dummy sensor data
            if self.start_time is None:
                self.start_time = time.time()
            
            current_time = time.time()
            rel_time = current_time - self.start_time
            
            # Generate sensor data
            sensor_data = self.dummy_generator.generate_sensor_data(rel_time)
            
            # Calculate angles using same algorithm as firmware
            p, r, y = DummyDataGenerator.calculate_angles(
                sensor_data['ax'], sensor_data['ay'], sensor_data['az']
            )
            
            self.times.append(rel_time)
            self.ax_data.append(sensor_data['ax'])
            self.ay_data.append(sensor_data['ay'])
            self.az_data.append(sensor_data['az'])
            self.pitch.append(p)
            self.roll.append(r)
            self.yaw.append(y)
            new_data = True
            
            # Sleep to match update rate
            time.sleep(DUMMY_DT)
        else:
            # Process serial data
            while True:
                try:
                    t, p, r, y = self.data_queue.get_nowait()
                    if self.start_time is None:
                        self.start_time = t
                    
                    # Convert to relative time
                    rel_time = t - self.start_time
                    
                    self.times.append(rel_time)
                    self.pitch.append(p)
                    self.roll.append(r)
                    self.yaw.append(y)
                    new_data = True
                except queue.Empty:
                    break
        
        if not new_data and len(self.times) == 0:
            return
        
            # Update plot data
        if len(self.times) > 0:
            times_array = np.array(self.times)
            
            if self.dummy_mode:
                # Update sensor data lines
                self.line_ax.set_data(times_array, np.array(self.ax_data))
                self.line_ay.set_data(times_array, np.array(self.ay_data))
                self.line_az.set_data(times_array, np.array(self.az_data))
            
            # Update angle lines
            self.line_pitch.set_data(times_array, np.array(self.pitch))
            self.line_roll.set_data(times_array, np.array(self.roll))
            self.line_yaw.set_data(times_array, np.array(self.yaw))
            
            # Update expected value lines if available
            if self.expected_times and len(self.times) > 0:
                # Find expected values in current time window
                current_time = times_array[-1] if times_array.size > 0 else 0
                window_start = max(0, current_time - self.window_sec)
                
                exp_mask = (np.array(self.expected_times) >= window_start) & \
                          (np.array(self.expected_times) <= current_time)
                
                if np.any(exp_mask):
                    exp_times = np.array(self.expected_times)[exp_mask]
                    exp_pitch = np.array(self.expected_pitch)[exp_mask]
                    exp_roll = np.array(self.expected_roll)[exp_mask]
                    exp_yaw = np.array(self.expected_yaw)[exp_mask]
                    
                    self.line_pitch_exp.set_data(exp_times, exp_pitch)
                    self.line_roll_exp.set_data(exp_times, exp_roll)
                    self.line_yaw_exp.set_data(exp_times, exp_yaw)
            
            # Update axes limits
            if times_array.size > 0:
                time_min = max(0, times_array[-1] - self.window_sec)
                time_max = times_array[-1] + 0.5
                
                for ax in self.axes:
                    ax.set_xlim(time_min, time_max)
            
            # Update statistics
            if self.dummy_mode:
                # Sensor data statistics
                if len(self.ax_data) > 0:
                    self.update_stats(self.ax_ax, self.stats_text_ax,
                                     np.array(self.ax_data), 'ax', unit='g')
                if len(self.ay_data) > 0:
                    self.update_stats(self.ax_ay, self.stats_text_ay,
                                     np.array(self.ay_data), 'ay', unit='g')
                if len(self.az_data) > 0:
                    self.update_stats(self.ax_az, self.stats_text_az,
                                     np.array(self.az_data), 'az', unit='g')
            
            # Angle statistics
            if len(self.pitch) > 0:
                self.update_stats(self.ax_pitch, self.stats_text_pitch, 
                                 np.array(self.pitch), 'Pitch')
            if len(self.roll) > 0:
                self.update_stats(self.ax_roll, self.stats_text_roll,
                                 np.array(self.roll), 'Roll')
            if len(self.yaw) > 0:
                self.update_stats(self.ax_yaw, self.stats_text_yaw,
                                 np.array(self.yaw), 'Yaw')
        
        if self.dummy_mode:
            return [self.line_ax, self.line_ay, self.line_az,
                    self.line_pitch, self.line_roll, self.line_yaw,
                    self.stats_text_ax, self.stats_text_ay, self.stats_text_az,
                    self.stats_text_pitch, self.stats_text_roll, self.stats_text_yaw]
        else:
            return [self.line_pitch, self.line_roll, self.line_yaw,
                    self.stats_text_pitch, self.stats_text_roll, self.stats_text_yaw]
    
    def update_stats(self, ax, text_obj, values, label, unit='°'):
        """Update statistics text box"""
        if values.size == 0:
            return
        
        mean_val = np.mean(values)
        std_val = np.std(values)
        min_val = np.min(values)
        max_val = np.max(values)
        
        stats = f"{label}:\n"
        stats += f"Mean: {mean_val:.3f}{unit}\n"
        stats += f"Std: {std_val:.3f}{unit}\n"
        stats += f"Range: [{min_val:.3f}{unit}, {max_val:.3f}{unit}]"
        
        text_obj.set_text(stats)
    
    def run(self):
        """Start the grapher"""
        try:
            if not self.dummy_mode:
                self.connect_serial()
            else:
                print(f"Dummy mode: Generating {self.dummy_pattern} pattern data...")
                print("Algorithm: atan2(ay, az) for roll, atan2(-ax, sqrt(ay²+az²)) for pitch")
            
            # Start animation
            ani = animation.FuncAnimation(self.fig, self.update_plot, interval=UPDATE_INTERVAL_MS,
                                         blit=False, cache_frame_data=False)
            
            # Save plot if requested
            if self.save_plot:
                def save_on_close(event):
                    mode = "dummy" if self.dummy_mode else "real"
                    filename = f"imu_telemetry_{mode}_{int(time.time())}.png"
                    self.fig.savefig(filename, dpi=150, bbox_inches='tight')
                    print(f"Plot saved to {filename}")
                
                self.fig.canvas.mpl_connect('close_event', save_on_close)
            
            plt.tight_layout()
            plt.show()
            
        except KeyboardInterrupt:
            print("\nStopping...")
        except Exception as e:
            print(f"Error: {e}")
        finally:
            if not self.dummy_mode:
                self.disconnect_serial()


def main():
    parser = argparse.ArgumentParser(description='SBR IMU Telemetry Grapher')
    parser.add_argument('--dummy', action='store_true',
                       help='Use dummy data instead of real serial data (for algorithm verification)')
    parser.add_argument('--pattern', type=str, default='sine',
                       choices=['sine', 'step', 'tilt', 'static'],
                       help='Pattern type for dummy data (default: sine, only with --dummy)')
    parser.add_argument('--port', type=str, default=None,
                       help='Serial port (e.g., COM3 or /dev/ttyUSB0). Auto-detect if not specified (ignored in dummy mode)')
    parser.add_argument('--expected-file', type=str, default=None,
                       help='CSV file with expected values (columns: time, pitch, roll, yaw)')
    parser.add_argument('--window', type=float, default=10,
                       help='Time window in seconds (default: 10)')
    parser.add_argument('--save', action='store_true',
                       help='Save plot to file on close')
    
    args = parser.parse_args()
    
    grapher = TelemetryGrapher(
        port=args.port,
        expected_file=args.expected_file,
        window_sec=args.window,
        save_plot=args.save,
        dummy_mode=args.dummy,
        dummy_pattern=args.pattern
    )
    
    grapher.run()


if __name__ == "__main__":
    main()

