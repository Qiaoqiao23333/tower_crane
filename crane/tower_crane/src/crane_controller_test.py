#!/usr/bin/env python3
"""
Test script for controlling the spherical pendulum crane simulation.

This script demonstrates different control scenarios:
1. Step acceleration input
2. Sinusoidal motion
3. Trapezoidal velocity profile

Usage:
    ros2 run tower_crane crane_controller_test.py --ros-args -p mode:=step
    ros2 run tower_crane crane_controller_test.py --ros-args -p mode:=sine
    ros2 run tower_crane crane_controller_test.py --ros-args -p mode:=trapezoid
"""

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class CraneControllerTest(Node):
    """Test controller that sends various command profiles to the crane."""
    
    def __init__(self):
        super().__init__('crane_controller_test')
        
        # Parameters
        self.declare_parameter('mode', 'step')  # step, sine, trapezoid, manual
        self.declare_parameter('amplitude', 0.5)  # m/s² for acceleration commands
        self.declare_parameter('frequency', 0.2)  # Hz for sinusoidal motion
        self.declare_parameter('control_rate', 20.0)  # Hz
        
        self.mode = self.get_parameter('mode').value
        self.amplitude = self.get_parameter('amplitude').value
        self.frequency = self.get_parameter('frequency').value
        control_rate = self.get_parameter('control_rate').value
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.accel_pub = self.create_publisher(Float64MultiArray, '/target_acceleration', 10)
        
        # Subscriber for state monitoring
        self.create_subscription(
            Float64MultiArray,
            '/crane_state',
            self.state_callback,
            10
        )
        
        # Timer for control loop
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)
        
        # Internal state
        self.current_state = None
        self.start_time = self.get_clock().now()
        self.phase = 0  # For state machine in trapezoid mode
        
        self.get_logger().info(f'Crane Controller Test Node Started')
        self.get_logger().info(f'Mode: {self.mode}')
        self.get_logger().info(f'Amplitude: {self.amplitude} m/s²')
        self.get_logger().info(f'Frequency: {self.frequency} Hz')
        self.get_logger().info(f'Control Rate: {control_rate} Hz')
        self.get_logger().info('=' * 70)
        
        if self.mode == 'step':
            self.get_logger().info('STEP MODE: Will apply step acceleration command')
        elif self.mode == 'sine':
            self.get_logger().info('SINE MODE: Will apply sinusoidal acceleration')
        elif self.mode == 'trapezoid':
            self.get_logger().info('TRAPEZOID MODE: Trapezoidal velocity profile')
        else:
            self.get_logger().info('MANUAL MODE: Waiting for external commands')
    
    def state_callback(self, msg):
        """Receive current crane state."""
        self.current_state = msg.data
    
    def control_loop(self):
        """Main control loop - generates and publishes commands."""
        current_time = self.get_clock().now()
        t = (current_time - self.start_time).nanoseconds / 1e9
        
        if self.mode == 'step':
            # Step input: accelerate for 2 seconds, then decelerate
            if t < 2.0:
                accel = self.amplitude
            elif t < 4.0:
                accel = -self.amplitude
            elif t < 6.0:
                accel = 0.0
            else:
                # Reset
                self.start_time = current_time
                accel = 0.0
        
        elif self.mode == 'sine':
            # Sinusoidal acceleration: a(t) = A * sin(2π*f*t)
            accel = self.amplitude * np.sin(2 * np.pi * self.frequency * t)
        
        elif self.mode == 'trapezoid':
            # Trapezoidal profile: accelerate -> constant -> decelerate
            T_accel = 2.0
            T_const = 3.0
            T_decel = 2.0
            T_total = T_accel + T_const + T_decel
            
            t_mod = t % T_total
            
            if t_mod < T_accel:
                accel = self.amplitude
            elif t_mod < T_accel + T_const:
                accel = 0.0
            elif t_mod < T_total:
                accel = -self.amplitude
            else:
                accel = 0.0
        
        else:  # manual mode
            accel = 0.0
        
        # Publish acceleration command
        msg = Float64MultiArray()
        msg.data = [accel]
        self.accel_pub.publish(msg)
        
        # Also publish as Twist for compatibility
        twist = Twist()
        twist.linear.x = accel
        self.cmd_vel_pub.publish(twist)
        
        # Log periodically
        if int(t * 10) % 10 == 0:  # Every 1 second
            if self.current_state is not None and len(self.current_state) >= 6:
                self.get_logger().info(
                    f't={t:.1f}s, cmd_a={accel:.3f} m/s², '
                    f'x={self.current_state[0]:.3f}m, '
                    f'θ_x={np.degrees(self.current_state[2]):.2f}°, '
                    f'θ_y={np.degrees(self.current_state[4]):.2f}°'
                )


def main(args=None):
    rclpy.init(args=args)
    node = CraneControllerTest()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

