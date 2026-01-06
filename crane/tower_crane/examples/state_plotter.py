#!/usr/bin/env python3
"""
Real-time State Plotter for Tower Crane Dynamics

This script subscribes to the crane state and plots:
1. Time history of position and angles
2. Phase portraits (position vs velocity, angle vs angular velocity)
3. Coupling visualization

Requires matplotlib.

Usage:
    ros2 run tower_crane state_plotter.py
"""

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend for ROS compatibility
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque


class StatePlotter(Node):
    """Real-time plotter for crane state variables."""
    
    def __init__(self):
        super().__init__('state_plotter')
        
        # Parameters
        self.declare_parameter('history_length', 500)  # Number of samples to plot
        self.declare_parameter('update_rate', 20.0)  # Plot update rate (Hz)
        
        history_len = self.get_parameter('history_length').value
        update_rate = self.get_parameter('update_rate').value
        
        # Data storage (circular buffers)
        self.time_data = deque(maxlen=history_len)
        self.x_data = deque(maxlen=history_len)
        self.x_dot_data = deque(maxlen=history_len)
        self.theta_x_data = deque(maxlen=history_len)
        self.theta_x_dot_data = deque(maxlen=history_len)
        self.theta_y_data = deque(maxlen=history_len)
        self.theta_y_dot_data = deque(maxlen=history_len)
        self.force_data = deque(maxlen=history_len)
        
        self.start_time = None
        
        # Subscribe to crane state
        self.create_subscription(
            Float64MultiArray,
            '/crane_state',
            self.state_callback,
            10
        )
        
        # Setup plot
        self.setup_plot()
        
        # Animation timer
        self.ani = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=int(1000.0 / update_rate),
            blit=False
        )
        
        self.get_logger().info('State Plotter Started')
        self.get_logger().info('Subscribing to /crane_state topic...')
        self.get_logger().info('Close the plot window to exit.')
    
    def state_callback(self, msg):
        """Receive and store crane state."""
        if len(msg.data) < 7:
            return
        
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        current_time = self.get_clock().now()
        t = (current_time - self.start_time).nanoseconds / 1e9
        
        # Store data
        self.time_data.append(t)
        self.x_data.append(msg.data[0])
        self.x_dot_data.append(msg.data[1])
        self.theta_x_data.append(np.degrees(msg.data[2]))  # Convert to degrees
        self.theta_x_dot_data.append(np.degrees(msg.data[3]))
        self.theta_y_data.append(np.degrees(msg.data[4]))
        self.theta_y_dot_data.append(np.degrees(msg.data[5]))
        self.force_data.append(msg.data[6])
    
    def setup_plot(self):
        """Initialize matplotlib figure and axes."""
        self.fig = plt.figure(figsize=(14, 10))
        self.fig.suptitle('Tower Crane Dynamics - Real-Time State Visualization', 
                         fontsize=14, fontweight='bold')
        
        # Create subplots
        # Row 1: Time histories
        self.ax1 = plt.subplot(3, 3, 1)  # Position
        self.ax2 = plt.subplot(3, 3, 2)  # Sway X
        self.ax3 = plt.subplot(3, 3, 3)  # Sway Y
        
        # Row 2: More time histories
        self.ax4 = plt.subplot(3, 3, 4)  # Velocity
        self.ax5 = plt.subplot(3, 3, 5)  # Angular velocity X
        self.ax6 = plt.subplot(3, 3, 6)  # Angular velocity Y
        
        # Row 3: Phase portraits and force
        self.ax7 = plt.subplot(3, 3, 7)  # Position phase portrait
        self.ax8 = plt.subplot(3, 3, 8)  # Sway X phase portrait
        self.ax9 = plt.subplot(3, 3, 9)  # Control force
        
        # Configure axes
        self.ax1.set_title('Trolley Position')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('x (m)')
        self.ax1.grid(True, alpha=0.3)
        
        self.ax2.set_title('Sway Angle X')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.set_ylabel('θ_x (deg)')
        self.ax2.grid(True, alpha=0.3)
        
        self.ax3.set_title('Sway Angle Y')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('θ_y (deg)')
        self.ax3.grid(True, alpha=0.3)
        
        self.ax4.set_title('Trolley Velocity')
        self.ax4.set_xlabel('Time (s)')
        self.ax4.set_ylabel('ẋ (m/s)')
        self.ax4.grid(True, alpha=0.3)
        
        self.ax5.set_title('Sway Angular Velocity X')
        self.ax5.set_xlabel('Time (s)')
        self.ax5.set_ylabel('θ̇_x (deg/s)')
        self.ax5.grid(True, alpha=0.3)
        
        self.ax6.set_title('Sway Angular Velocity Y')
        self.ax6.set_xlabel('Time (s)')
        self.ax6.set_ylabel('θ̇_y (deg/s)')
        self.ax6.grid(True, alpha=0.3)
        
        self.ax7.set_title('Position Phase Portrait')
        self.ax7.set_xlabel('x (m)')
        self.ax7.set_ylabel('ẋ (m/s)')
        self.ax7.grid(True, alpha=0.3)
        
        self.ax8.set_title('Sway X Phase Portrait')
        self.ax8.set_xlabel('θ_x (deg)')
        self.ax8.set_ylabel('θ̇_x (deg/s)')
        self.ax8.grid(True, alpha=0.3)
        
        self.ax9.set_title('Control Force')
        self.ax9.set_xlabel('Time (s)')
        self.ax9.set_ylabel('F (N)')
        self.ax9.grid(True, alpha=0.3)
        
        # Initialize line objects
        self.line1, = self.ax1.plot([], [], 'b-', linewidth=1.5)
        self.line2, = self.ax2.plot([], [], 'r-', linewidth=1.5)
        self.line3, = self.ax3.plot([], [], 'g-', linewidth=1.5)
        self.line4, = self.ax4.plot([], [], 'b-', linewidth=1.5)
        self.line5, = self.ax5.plot([], [], 'r-', linewidth=1.5)
        self.line6, = self.ax6.plot([], [], 'g-', linewidth=1.5)
        self.line7, = self.ax7.plot([], [], 'b-', linewidth=1.5, alpha=0.6)
        self.line8, = self.ax8.plot([], [], 'r-', linewidth=1.5, alpha=0.6)
        self.line9, = self.ax9.plot([], [], 'm-', linewidth=1.5)
        
        plt.tight_layout()
    
    def update_plot(self, frame):
        """Update plot with new data."""
        if len(self.time_data) == 0:
            return
        
        # Convert deques to arrays for plotting
        t = np.array(self.time_data)
        x = np.array(self.x_data)
        x_dot = np.array(self.x_dot_data)
        theta_x = np.array(self.theta_x_data)
        theta_x_dot = np.array(self.theta_x_dot_data)
        theta_y = np.array(self.theta_y_data)
        theta_y_dot = np.array(self.theta_y_dot_data)
        force = np.array(self.force_data)
        
        # Update time series
        self.line1.set_data(t, x)
        self.line2.set_data(t, theta_x)
        self.line3.set_data(t, theta_y)
        self.line4.set_data(t, x_dot)
        self.line5.set_data(t, theta_x_dot)
        self.line6.set_data(t, theta_y_dot)
        self.line9.set_data(t, force)
        
        # Update phase portraits
        self.line7.set_data(x, x_dot)
        self.line8.set_data(theta_x, theta_x_dot)
        
        # Auto-scale axes
        for ax, data_y in [
            (self.ax1, x), (self.ax2, theta_x), (self.ax3, theta_y),
            (self.ax4, x_dot), (self.ax5, theta_x_dot), (self.ax6, theta_y_dot),
            (self.ax9, force)
        ]:
            if len(data_y) > 0:
                ax.relim()
                ax.autoscale_view()
        
        # Phase portrait limits
        if len(x) > 0:
            self.ax7.relim()
            self.ax7.autoscale_view()
        if len(theta_x) > 0:
            self.ax8.relim()
            self.ax8.autoscale_view()
        
        return (self.line1, self.line2, self.line3, self.line4, 
                self.line5, self.line6, self.line7, self.line8, self.line9)


def main(args=None):
    rclpy.init(args=args)
    node = StatePlotter()
    
    # Use plt.show() to display the plot
    # This blocks, but we need a separate thread for ROS spinning
    import threading
    
    def spin_thread():
        rclpy.spin(node)
    
    thread = threading.Thread(target=spin_thread, daemon=True)
    thread.start()
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

