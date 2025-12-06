#!/usr/bin/env python3
"""
SBR IMU Telemetry Grapher
- Real-time plotting of PITCH, ROLL, YAW values from serial telemetry
- Optional expected value overlay for comparison
- Uses matplotlib for high-quality graphs
- Parses telemetry format: "PITCH:%.2f ROLL:%.2f YAW:%.2f"

Requirements: pip install pyserial matplotlib numpy

Usage:
    python imu_telemetry_graph.py [--port COM3] [--expected-file expected.csv]
    
    --port: Serial port (default: auto-detect)
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
    def __init__(self, port=None, expected_file=None, window_sec=10, save_plot=False):
        self.port = port
        self.expected_file = expected_file
        self.window_sec = window_sec
        self.save_plot = save_plot
        
        # Data storage (time, pitch, roll, yaw)
        self.times = deque(maxlen=int(window_sec * TELEMETRY_FREQ_HZ * 2))
        self.pitch = deque(maxlen=int(window_sec * TELEMETRY_FREQ_HZ * 2))
        self.roll = deque(maxlen=int(window_sec * TELEMETRY_FREQ_HZ * 2))
        self.yaw = deque(maxlen=int(window_sec * TELEMETRY_FREQ_HZ * 2))
        
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
        self.fig, self.axes = plt.subplots(3, 1, figsize=(12, 10))
        self.fig.suptitle('SBR IMU Telemetry - Real-time', fontsize=14, fontweight='bold')
        
        # Setup subplots
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
        self.line_pitch, = self.ax_pitch.plot([], [], 'b-', linewidth=1.5, label='Actual')
        self.line_roll, = self.ax_roll.plot([], [], 'r-', linewidth=1.5, label='Actual')
        self.line_yaw, = self.ax_yaw.plot([], [], 'g-', linewidth=1.5, label='Actual')
        
        # Expected value lines (if provided)
        self.line_pitch_exp = None
        self.line_roll_exp = None
        self.line_yaw_exp = None
        
        # Statistics text
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
        # Process all queued data
        new_data = False
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
            
            # Update actual value lines
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
            if len(self.pitch) > 0:
                self.update_stats(self.ax_pitch, self.stats_text_pitch, 
                                 np.array(self.pitch), 'Pitch')
            if len(self.roll) > 0:
                self.update_stats(self.ax_roll, self.stats_text_roll,
                                 np.array(self.roll), 'Roll')
            if len(self.yaw) > 0:
                self.update_stats(self.ax_yaw, self.stats_text_yaw,
                                 np.array(self.yaw), 'Yaw')
        
        return [self.line_pitch, self.line_roll, self.line_yaw,
                self.stats_text_pitch, self.stats_text_roll, self.stats_text_yaw]
    
    def update_stats(self, ax, text_obj, values, label):
        """Update statistics text box"""
        if values.size == 0:
            return
        
        mean_val = np.mean(values)
        std_val = np.std(values)
        min_val = np.min(values)
        max_val = np.max(values)
        
        stats = f"{label}:\n"
        stats += f"Mean: {mean_val:.2f}°\n"
        stats += f"Std: {std_val:.2f}°\n"
        stats += f"Range: [{min_val:.2f}°, {max_val:.2f}°]"
        
        text_obj.set_text(stats)
    
    def run(self):
        """Start the grapher"""
        try:
            self.connect_serial()
            
            # Start animation
            ani = animation.FuncAnimation(self.fig, self.update_plot, interval=UPDATE_INTERVAL_MS,
                                         blit=False, cache_frame_data=False)
            
            # Save plot if requested
            if self.save_plot:
                def save_on_close(event):
                    filename = f"imu_telemetry_{int(time.time())}.png"
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
            self.disconnect_serial()


def main():
    parser = argparse.ArgumentParser(description='SBR IMU Telemetry Grapher')
    parser.add_argument('--port', type=str, default=None,
                       help='Serial port (e.g., COM3 or /dev/ttyUSB0). Auto-detect if not specified')
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
        save_plot=args.save
    )
    
    grapher.run()


if __name__ == "__main__":
    main()

