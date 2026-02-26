#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import re

class CraneJoystickNode(Node):
    def __init__(self):
        super().__init__('crane_joystick_driver')
        
        # === Debug mode ===
        self.debug_mode = self.declare_parameter('debug', False).value
        self.data_count = 0  # Receive data count
        self.publish_count = {'hoist': 0, 'trolley': 0, 'slewing': 0}  # Publish count

        # === Configure serial port (please modify port number according to actual situation, usually /dev/ttyUSB0) ===
        serial_port = self.declare_parameter('serial_port', '/dev/ttyUSB0').value
        self.ser = None
        try:
            self.ser = serial.Serial(serial_port, 115200, timeout=1)
            self.get_logger().info(f'✅ Serial port connected successfully: {serial_port}')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to open serial port {serial_port}: {e}')
            self.get_logger().error('Node will continue running, but cannot read serial port data')
            # Don't return, let node continue initialization, but serial port is None

        # === Create publishers ===
        self.pub_hoist = self.create_publisher(Float32, '/hoist/target_position', 10)
        self.pub_trolley = self.create_publisher(Float32, '/trolley/target_position', 10)
        self.pub_slewing = self.create_publisher(Float32, '/slewing/target_position', 10)

        # === Internal state recording ===
        # Assume initial position (please adjust according to your motor's actual zero position)
        self.pos_hoist = 0.0
        self.pos_trolley = 0.0
        self.pos_slewing = 0.0

        self.mode = 0 # 0: Control Trolley/Slewing, 1: Control Hoist
        self.last_sw = 1
        
        # === Control parameters ===
        # Current joystick ADC range is approximately 0-1023, center value about 512
        self.dead_zone = 40      # Dead zone, prevent slight jitter
        self.center_val = 512    # Joystick ADC center value
        self.speed_factor = 5.0  # Sensitivity adjustment (larger is more sensitive, increase to make movement more obvious)
        
        # === Position limits (prevent cumulative drift) ===
        self.max_pos = 360.0  # Maximum position limit
        self.min_pos = -360.0  # Minimum position limit

        # Create timer to read serial port
        self.create_timer(0.05, self.timer_callback) # 20Hz
        
        # Periodically output status (only useful in debug_mode)
        self.create_timer(1.0, self.status_callback) # 1Hz

    def map_speed(self, val):
        """Map joystick value (0-1023) to velocity increment"""
        diff = val - self.center_val
        if abs(diff) < self.dead_zone:
            return 0.0
        # Normalize: map offset (-512 to +512) to (-1 to +1), then multiply by speed factor
        # Use 512.0 as normalization base (because range is 0-1023, center 512, max offset ±512)
        normalized = diff / 512.0
        return normalized * self.speed_factor

    def clamp_position(self, pos):
        """Limit position within reasonable range"""
        return max(self.min_pos, min(self.max_pos, pos))
    
    def status_callback(self):
        """Periodically report status"""
        if self.debug_mode:
            self.get_logger().info(
                f'📊 Status: Received data={self.data_count} times, '
                f'Published[Hoist={self.publish_count["hoist"]}, '
                f'Trolley={self.publish_count["trolley"]}, '
                f'Slewing={self.publish_count["slewing"]}], '
                f'Position[H={self.pos_hoist:.2f}°, T={self.pos_trolley:.2f}°, S={self.pos_slewing:.2f}°]'
            )

    def timer_callback(self):
        # Check if serial port is available
        if self.ser is None or not self.ser.is_open:
            # Warn every 5 seconds
            if hasattr(self, '_last_warning_time'):
                import time
                if time.time() - self._last_warning_time > 5.0:
                    self.get_logger().warn('⚠️  Serial port not connected, cannot read data')
                    self._last_warning_time = time.time()
            else:
                import time
                self._last_warning_time = time.time()
            return

        try:
            if self.ser.in_waiting > 0:
                # Read one line of data, use errors='ignore' to prevent decode errors
                raw_data = self.ser.readline()
                if len(raw_data) == 0:
                    # Device reports readable but no data, may be disconnected
                    self.get_logger().debug('Serial port reports readable but no data, may be disconnected')
                    return
                line = raw_data.decode('utf-8', errors='ignore').strip()
                
                # Skip empty lines
                if not line:
                    return
                
                # Clean data: remove possible label prefixes (e.g., "X:", "Y:", "SW:", etc.)
                # Handle formats like "X:496,Y:509,SW:1" or ":495494,Y:495,Y" or "495494,495,1"
                cleaned_line = line
                # Remove all alphanumeric combinations with colon labels (e.g., "X:", "Y:", "SW:", etc.)
                cleaned_line = re.sub(r'\w+:', '', cleaned_line)
                # Remove colon at line start (handle ":495494" case)
                cleaned_line = cleaned_line.lstrip(':')
                
                data = cleaned_line.split(',')
                
                if len(data) < 3:
                    self.get_logger().debug(f'Data format error, expected at least 3 values, got {len(data)}: {line}')
                    return

                # Read raw data, add error handling
                try:
                    # Extract numeric part from each field
                    def extract_number(s):
                        # Remove all non-numeric characters (except minus sign), keep only numbers
                        s = s.strip()
                        # If string starts with colon, remove it
                        s = s.lstrip(':')
                        # Extract first continuous numeric sequence
                        match = re.search(r'-?\d+', s)
                        if match:
                            return int(match.group())
                        raise ValueError(f"Cannot extract number from '{s}'")
                    
                    raw_x = extract_number(data[0])
                    raw_y = extract_number(data[1])
                    raw_sw = extract_number(data[2])
                except (ValueError, IndexError) as e:
                    self.get_logger().warn(f'Data parsing failed: {line}, error: {e}')
                    return

                # Successfully parsed one data entry
                self.data_count += 1
                if self.debug_mode:
                    self.get_logger().info(
                        f'📥 Received data: raw="{line}", cleaned="{cleaned_line}", '
                        f'X={raw_x}, Y={raw_y}, SW={raw_sw}, mode={self.mode}'
                    )

                # === Button handling: mode switch (rising edge trigger) ===
                if raw_sw == 0 and self.last_sw == 1:
                    self.mode = 1 - self.mode # 0 becomes 1, 1 becomes 0
                    mode_name = "Hoist" if self.mode == 1 else "XY (Slewing/Trolley)"
                    self.get_logger().info(f'Mode switched to: {mode_name}')
                self.last_sw = raw_sw

                # === Motion calculation ===
                speed_x = self.map_speed(raw_x) # Left/Right
                speed_y = self.map_speed(raw_y) # Up/Down

                # Record old position, used to determine if publishing is needed
                old_pos_hoist = self.pos_hoist
                old_pos_trolley = self.pos_trolley
                old_pos_slewing = self.pos_slewing

                # Assign control based on mode
                if self.mode == 0:
                    # Mode 0: X-axis controls rotation (Slewing), Y-axis controls trolley (Trolley)
                    self.pos_slewing = self.clamp_position(self.pos_slewing + speed_x)
                    self.pos_trolley = self.clamp_position(self.pos_trolley + speed_y)
                else:
                    # Mode 1: Y-axis controls hoist (Hoist)
                    self.pos_hoist = self.clamp_position(self.pos_hoist + speed_y)
                    # X-axis can be left empty in hoist mode, or used for fine adjustment

                # === Publish topics (only publish when value changes) ===
                if abs(self.pos_hoist - old_pos_hoist) > 0.01:  # 0.01 degree threshold
                    msg_hoist = Float32()
                    msg_hoist.data = float(self.pos_hoist)
                    self.pub_hoist.publish(msg_hoist)
                    self.publish_count['hoist'] += 1
                    if self.debug_mode:
                        self.get_logger().info(f'📤 Published Hoist target position: {self.pos_hoist:.2f}°')
                
                if abs(self.pos_trolley - old_pos_trolley) > 0.01:
                    msg_trolley = Float32()
                    msg_trolley.data = float(self.pos_trolley)
                    self.pub_trolley.publish(msg_trolley)
                    self.publish_count['trolley'] += 1
                    if self.debug_mode:
                        self.get_logger().info(f'📤 Published Trolley target position: {self.pos_trolley:.2f}°')
                
                if abs(self.pos_slewing - old_pos_slewing) > 0.01:
                    msg_slewing = Float32()
                    msg_slewing.data = float(self.pos_slewing)
                    self.pub_slewing.publish(msg_slewing)
                    self.publish_count['slewing'] += 1
                    if self.debug_mode:
                        self.get_logger().info(f'📤 Published Slewing target position: {self.pos_slewing:.2f}°')

        except serial.SerialException as e:
            self.get_logger().error(f'Serial port communication error: {e}')
        except Exception as e:
            self.get_logger().warn(f'Unexpected error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CraneJoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Received interrupt signal, shutting down node...')
    finally:
        # Close serial port
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
            node.get_logger().info('Serial port closed')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()