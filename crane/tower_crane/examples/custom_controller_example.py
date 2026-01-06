#!/usr/bin/env python3
"""
Example: Custom Controller for Tower Crane Spherical Pendulum

This example demonstrates how to create a custom controller that:
1. Subscribes to crane state
2. Implements a simple feedback control law
3. Publishes control commands

The controller implements a basic PD (Proportional-Derivative) control
to move the trolley to a target position while minimizing sway.

Usage:
    ros2 run tower_crane custom_controller_example.py --ros-args -p target_position:=1.0
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class CustomCraneController(Node):
    """
    Simple PD controller with sway compensation for tower crane.
    
    Control Law:
        F = Kp*(x_target - x) - Kd*ẋ - Ks*θ_x - Ksd*θ̇_x
    
    Where:
        - Kp: Position gain
        - Kd: Velocity damping
        - Ks: Sway compensation gain
        - Ksd: Sway velocity damping
    """
    
    def __init__(self):
        super().__init__('custom_crane_controller')
        
        # Parameters
        self.declare_parameter('target_position', 0.0)
        self.declare_parameter('Kp', 10.0)   # Position gain
        self.declare_parameter('Kd', 5.0)    # Velocity damping
        self.declare_parameter('Ks', 15.0)   # Sway compensation
        self.declare_parameter('Ksd', 3.0)   # Sway velocity damping
        self.declare_parameter('control_rate', 50.0)
        
        self.target_pos = self.get_parameter('target_position').value
        self.Kp = self.get_parameter('Kp').value
        self.Kd = self.get_parameter('Kd').value
        self.Ks = self.get_parameter('Ks').value
        self.Ksd = self.get_parameter('Ksd').value
        control_rate = self.get_parameter('control_rate').value
        
        # Current state
        self.current_state = np.zeros(6)
        self.state_received = False
        
        # Publishers & Subscribers
        self.force_pub = self.create_publisher(
            Float64MultiArray,
            '/control_force',
            10
        )
        
        self.create_subscription(
            Float64MultiArray,
            '/crane_state',
            self.state_callback,
            10
        )
        
        # Control timer
        self.timer = self.create_timer(1.0 / control_rate, self.control_loop)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('Custom Crane Controller Started')
        self.get_logger().info(f'Target Position: {self.target_pos:.3f} m')
        self.get_logger().info(f'Control Gains: Kp={self.Kp}, Kd={self.Kd}, '
                             f'Ks={self.Ks}, Ksd={self.Ksd}')
        self.get_logger().info('=' * 70)
    
    def state_callback(self, msg):
        """Receive current crane state."""
        if len(msg.data) >= 6:
            self.current_state = np.array(msg.data[:6])
            self.state_received = True
    
    def control_loop(self):
        """Main control loop - compute and publish control force."""
        if not self.state_received:
            return
        
        # Unpack state
        x = self.current_state[0]
        x_dot = self.current_state[1]
        theta_x = self.current_state[2]
        theta_x_dot = self.current_state[3]
        theta_y = self.current_state[4]
        theta_y_dot = self.current_state[5]
        
        # PD control with sway compensation
        # F = Kp*(x_target - x) - Kd*ẋ - Ks*θ_x - Ksd*θ̇_x
        position_error = self.target_pos - x
        
        F_control = (
            self.Kp * position_error -
            self.Kd * x_dot -
            self.Ks * theta_x -
            self.Ksd * theta_x_dot
        )
        
        # Optional: Add Y-axis sway compensation
        # F_control -= self.Ks * theta_y + self.Ksd * theta_y_dot
        
        # Publish control force
        msg = Float64MultiArray()
        msg.data = [F_control]
        self.force_pub.publish(msg)
        
        # Log periodically
        if not hasattr(self, '_log_counter'):
            self._log_counter = 0
        
        self._log_counter += 1
        if self._log_counter % 100 == 0:  # Every 2 seconds at 50 Hz
            self.get_logger().info(
                f'x={x:.3f}m (err={position_error:.3f}m), '
                f'θ_x={np.degrees(theta_x):.2f}°, '
                f'θ_y={np.degrees(theta_y):.2f}°, '
                f'F={F_control:.2f}N'
            )


def main(args=None):
    rclpy.init(args=args)
    node = CustomCraneController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Controller stopped by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

