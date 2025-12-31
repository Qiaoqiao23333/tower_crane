#!/usr/bin/env python3
"""
Tower Crane Spherical Pendulum Dynamics Simulator
Based on Lagrangian Mechanics for Nonlinear Optimal Control

This node simulates the coupled dynamics of a tower crane with a spherical pendulum,
where the trolley's acceleration directly induces payload sway in both X and Y directions.

Author: Generated for Tower Crane Project
License: Apache 2.0
"""

import numpy as np
from scipy.integrate import odeint
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Time


class SphericalPendulumDynamics(Node):
    """
    Simulates tower crane dynamics with spherical pendulum using Lagrangian mechanics.
    
    ============================================================================
    MATHEMATICAL MODEL - LAGRANGIAN MECHANICS
    ============================================================================
    
    System Configuration:
    - Trolley position: x (along the boom, prismatic joint)
    - Payload sway angles: θ_x, θ_y (spherical pendulum)
    - Rope length: L (can be varied via hook_joint)
    - Payload mass: m
    - Trolley mass: M
    
    Generalized coordinates: q = [x, θ_x, θ_y]
    
    Position of payload in Cartesian coordinates:
        X_payload = x + L*sin(θ_x)
        Y_payload = L*sin(θ_y)*cos(θ_x)
        Z_payload = -L*cos(θ_x)*cos(θ_y)
    
    Lagrangian: L = T - V
    
    Kinetic Energy (T):
        T_trolley = (1/2)*M*ẋ²
        T_payload = (1/2)*m*[(ẋ + L*θ̇_x*cos(θ_x))² + (L*θ̇_y*cos(θ_y)*cos(θ_x) - L*θ̇_x*sin(θ_y)*sin(θ_x))² + 
                              (L*θ̇_x*sin(θ_x)*cos(θ_y) + L*θ̇_y*sin(θ_y)*cos(θ_x))²]
        
        Simplified (small angle approximation for clearer coupling):
        T ≈ (1/2)*(M+m)*ẋ² + m*L*ẋ*θ̇_x + (1/2)*m*L²*(θ̇_x² + θ̇_y²)
    
    Potential Energy (V):
        V = -m*g*L*cos(θ_x)*cos(θ_y)
        V ≈ m*g*L*(1 - (θ_x² + θ_y²)/2)  [small angle approximation]
    
    ============================================================================
    EQUATIONS OF MOTION (Euler-Lagrange: d/dt(∂L/∂q̇ᵢ) - ∂L/∂qᵢ = Qᵢ)
    ============================================================================
    
    For TROLLEY (x-direction):
        (M + m)*ẍ + m*L*θ̈_x*cos(θ_x) - m*L*θ̇_x²*sin(θ_x) = F_control
    
    For SWAY X (θ_x):
        m*L²*θ̈_x + m*L*ẍ*cos(θ_x) + m*g*L*sin(θ_x) = -c_x*θ̇_x
    
    For SWAY Y (θ_y):
        m*L²*θ̈_y*cos²(θ_x) + m*g*L*sin(θ_y)*cos(θ_x) = -c_y*θ̇_y
    
    LINEARIZED VERSION (small angles: sin(θ)≈θ, cos(θ)≈1):
    
        (M + m)*ẍ + m*L*θ̈_x = F_control
        m*L²*θ̈_x + m*L*ẍ + m*g*L*θ_x = -c_x*θ̇_x
        m*L²*θ̈_y + m*g*L*θ_y = -c_y*θ̇_y
    
    COUPLING INSIGHT:
    The trolley acceleration ẍ appears in the sway equation for θ_x.
    This means any trolley movement DIRECTLY induces payload sway - the system is COUPLED.
    
    Solving for accelerations:
        θ̈_x = -(g/L)*θ_x - (ẍ/L) - (c_x/(m*L²))*θ̇_x
        θ̈_y = -(g/L)*θ_y - (c_y/(m*L²))*θ̇_y
        ẍ = (F_control - m*L*θ̈_x)/(M + m)
    
    This can be rearranged to:
        θ̈_x = -[(M+m)*g + c_x*ẍ]/(L*(M+m) + m*L)*θ_x - [c_x/(m*L²)]*θ̇_x + [M/(m*L(M+m))]*F_control
    
    Simplified with substitution:
        θ̈_x ≈ -(g/L)*θ_x - (1/L)*ẍ - (c_x/(m*L²))*θ̇_x
        θ̈_y ≈ -(g/L)*θ_y - (c_y/(m*L²))*θ̇_y
    
    ============================================================================
    STATE SPACE REPRESENTATION
    ============================================================================
    
    State vector: s = [x, ẋ, θ_x, θ̇_x, θ_y, θ̇_y]ᵀ
    
    State equations:
        ṡ₁ = s₂                           (trolley velocity)
        ṡ₂ = F_control/(M+m) - m*L*θ̈_x/(M+m)  (trolley acceleration)
        ṡ₃ = s₄                           (sway_x velocity)
        ṡ₄ = -(g/L)*s₃ - (s₂/L) - (c_x/(m*L²))*s₄  (sway_x acceleration)
        ṡ₅ = s₆                           (sway_y velocity)
        ṡ₆ = -(g/L)*s₅ - (c_y/(m*L²))*s₆  (sway_y acceleration)
    
    ============================================================================
    """
    
    def __init__(self):
        super().__init__('spherical_pendulum_dynamics')
        
        # =====================================================================
        # PHYSICAL PARAMETERS
        # =====================================================================
        self.declare_parameter('payload_mass', 5.0)  # kg (from URDF hook mass)
        self.declare_parameter('trolley_mass', 0.568)  # kg (from URDF trolley mass)
        self.declare_parameter('rope_length', 1.9126)  # m (from URDF hook joint origin)
        self.declare_parameter('gravity', 9.81)  # m/s²
        self.declare_parameter('damping_x', 0.1)  # Damping coefficient for θ_x
        self.declare_parameter('damping_y', 0.1)  # Damping coefficient for θ_y
        self.declare_parameter('simulation_dt', 0.01)  # Simulation timestep (s)
        self.declare_parameter('publish_rate', 50.0)  # Hz
        self.declare_parameter('joint_states_topic', '/joint_states')  # Topic for joint_states publishing
        
        # Load parameters
        self.m = self.get_parameter('payload_mass').value
        self.M = self.get_parameter('trolley_mass').value
        self.L = self.get_parameter('rope_length').value
        self.g = self.get_parameter('gravity').value
        self.c_x = self.get_parameter('damping_x').value
        self.c_y = self.get_parameter('damping_y').value
        self.dt = self.get_parameter('simulation_dt').value
        publish_rate = self.get_parameter('publish_rate').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        
        # =====================================================================
        # STATE VARIABLES
        # =====================================================================
        # State: [x, x_dot, theta_x, theta_x_dot, theta_y, theta_y_dot]
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        
        # Control input (force on trolley)
        self.F_control = 0.0
        self.target_acceleration = 0.0
        
        # Additional joints (slewing and hook)
        self.slewing_angle = 0.0
        self.hook_position = 0.0  # Extension of hook (affects rope length)
        
        # =====================================================================
        # ROS2 INTERFACE
        # =====================================================================
        
        # Subscriber: Control commands
        # Option 1: Twist message (linear.x = desired trolley velocity or acceleration)
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Option 2: Direct acceleration command
        self.create_subscription(
            Float64MultiArray,
            '/target_acceleration',
            self.acceleration_callback,
            10
        )
        
        # Option 3: Force command
        self.create_subscription(
            Float64MultiArray,
            '/control_force',
            self.force_callback,
            10
        )
        
        # Publisher: Joint states for visualization in RViz
        self.joint_state_pub = self.create_publisher(
            JointState,
            joint_states_topic,
            10
        )
        
        # Publisher: State information for monitoring
        self.state_pub = self.create_publisher(
            Float64MultiArray,
            '/crane_state',
            10
        )
        
        # Timer for simulation update
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.simulation_step)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('Spherical Pendulum Dynamics Node Started')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Physical Parameters:')
        self.get_logger().info(f'  Payload mass (m):     {self.m:.3f} kg')
        self.get_logger().info(f'  Trolley mass (M):     {self.M:.3f} kg')
        self.get_logger().info(f'  Rope length (L):      {self.L:.3f} m')
        self.get_logger().info(f'  Gravity (g):          {self.g:.3f} m/s²')
        self.get_logger().info(f'  Damping X (c_x):      {self.c_x:.3f}')
        self.get_logger().info(f'  Damping Y (c_y):      {self.c_y:.3f}')
        self.get_logger().info(f'  Simulation dt:        {self.dt:.4f} s')
        self.get_logger().info(f'  Publish rate:         {publish_rate:.1f} Hz')
        self.get_logger().info('=' * 70)
        self.get_logger().info('Subscribed topics:')
        self.get_logger().info('  /cmd_vel              (geometry_msgs/Twist)')
        self.get_logger().info('  /target_acceleration  (std_msgs/Float64MultiArray)')
        self.get_logger().info('  /control_force        (std_msgs/Float64MultiArray)')
        self.get_logger().info('Publishing topics:')
        self.get_logger().info(f'  {joint_states_topic}         (sensor_msgs/JointState)')
        self.get_logger().info('  /crane_state          (std_msgs/Float64MultiArray)')
        self.get_logger().info('=' * 70)
    
    def cmd_vel_callback(self, msg):
        """
        Callback for Twist commands.
        Interprets linear.x as desired trolley acceleration.
        """
        self.target_acceleration = msg.linear.x
        # Simple proportional control to convert acceleration to force
        # F = (M + m) * a_desired
        self.F_control = (self.M + self.m) * self.target_acceleration
        
    def acceleration_callback(self, msg):
        """
        Callback for direct acceleration commands.
        msg.data[0]: trolley acceleration (m/s²)
        """
        if len(msg.data) > 0:
            self.target_acceleration = msg.data[0]
            self.F_control = (self.M + self.m) * self.target_acceleration
    
    def force_callback(self, msg):
        """
        Callback for direct force commands.
        msg.data[0]: control force on trolley (N)
        """
        if len(msg.data) > 0:
            self.F_control = msg.data[0]
    
    def dynamics(self, state, t):
        """
        System dynamics: computes state derivatives.
        
        This implements the NONLINEAR equations of motion derived from
        Lagrangian mechanics.
        
        Args:
            state: [x, x_dot, theta_x, theta_x_dot, theta_y, theta_y_dot]
            t: time (not used, but required by odeint)
        
        Returns:
            state_dot: time derivatives of state
        """
        # Unpack state
        x, x_dot, theta_x, theta_x_dot, theta_y, theta_y_dot = state
        
        # Current rope length (can vary with hook position)
        L_current = self.L + self.hook_position
        
        # Prevent division by zero
        if L_current < 0.1:
            L_current = 0.1
        
        # =====================================================================
        # NONLINEAR EQUATIONS OF MOTION
        # =====================================================================
        
        # For numerical stability and realistic crane behavior, we use
        # the nonlinear formulation but with reasonable angle limits
        
        cos_theta_x = np.cos(theta_x)
        sin_theta_x = np.sin(theta_x)
        cos_theta_y = np.cos(theta_y)
        sin_theta_y = np.sin(theta_y)
        
        # Total system mass
        m_total = self.M + self.m
        
        # Coupled mass matrix (from Lagrangian):
        # [M+m      m*L*cos(θ_x)  ] [ẍ   ]   [RHS_x ]
        # [m*L*cos(θ_x)  m*L²    ] [θ̈_x ] = [RHS_θx]
        
        # Right-hand side terms
        RHS_x = self.F_control + self.m * L_current * theta_x_dot**2 * sin_theta_x
        RHS_theta_x = -self.m * self.g * L_current * sin_theta_x - self.c_x * theta_x_dot
        
        # Solve 2x2 system for x_ddot and theta_x_ddot
        det = m_total * self.m * L_current**2 - (self.m * L_current * cos_theta_x)**2
        
        if abs(det) < 1e-6:  # Singular matrix protection
            x_ddot = self.F_control / m_total
            theta_x_ddot = -(self.g / L_current) * sin_theta_x - (self.c_x / (self.m * L_current**2)) * theta_x_dot
        else:
            # Cramer's rule
            x_ddot = (RHS_x * self.m * L_current**2 - RHS_theta_x * self.m * L_current * cos_theta_x) / det
            theta_x_ddot = (m_total * RHS_theta_x - self.m * L_current * cos_theta_x * RHS_x) / det
        
        # Y-direction sway (decoupled for spherical pendulum in this formulation)
        # θ̈_y = -(g/L)*sin(θ_y)/cos(θ_x) - (c_y/(m*L²))*θ̇_y
        # Simplified with small angle for Y:
        if abs(cos_theta_x) < 0.1:  # Avoid singularity
            cos_theta_x = 0.1 * np.sign(cos_theta_x) if cos_theta_x != 0 else 0.1
        
        theta_y_ddot = -(self.g / L_current) * sin_theta_y / cos_theta_x - \
                       (self.c_y / (self.m * L_current**2)) * theta_y_dot
        
        # =====================================================================
        # ALTERNATIVE: LINEARIZED VERSION (uncomment to use)
        # =====================================================================
        # This is the small-angle approximation that makes coupling more explicit
        # 
        # # Compute trolley acceleration from control input
        # x_ddot = self.F_control / m_total
        # 
        # # Coupled sway dynamics - trolley acceleration DIRECTLY affects sway
        # # θ̈_x = -(g/L)*θ_x - (ẍ/L) - (c_x/(m*L²))*θ̇_x
        # theta_x_ddot = -(self.g / L_current) * theta_x - \
        #                (x_ddot / L_current) - \
        #                (self.c_x / (self.m * L_current**2)) * theta_x_dot
        # 
        # # Y-direction sway (independent in linear approximation)
        # # θ̈_y = -(g/L)*θ_y - (c_y/(m*L²))*θ̇_y
        # theta_y_ddot = -(self.g / L_current) * theta_y - \
        #                (self.c_y / (self.m * L_current**2)) * theta_y_dot
        # =====================================================================
        
        # Assemble state derivative
        state_dot = np.array([
            x_dot,                # dx/dt
            x_ddot,              # d²x/dt²
            theta_x_dot,         # dθ_x/dt
            theta_x_ddot,        # d²θ_x/dt²
            theta_y_dot,         # dθ_y/dt
            theta_y_ddot         # d²θ_y/dt²
        ])
        
        return state_dot
    
    def simulation_step(self):
        """
        Performs one simulation step and publishes results.
        """
        # Integrate dynamics using Runge-Kutta method (odeint)
        t_span = [0, self.dt]
        solution = odeint(self.dynamics, self.state, t_span)
        self.state = solution[-1, :]
        
        # Enforce position limits (from URDF)
        x_min, x_max = -2.0, 0.5
        if self.state[0] < x_min:
            self.state[0] = x_min
            self.state[1] = 0.0  # Stop velocity
        elif self.state[0] > x_max:
            self.state[0] = x_max
            self.state[1] = 0.0
        
        # Publish joint states for RViz visualization
        self.publish_joint_states()
        
        # Publish full state for monitoring
        self.publish_state()
    
    def publish_joint_states(self):
        """
        Publishes JointState message for URDF visualization.
        
        Maps the physical state to URDF joints:
        - trolley_joint: state[0] (trolley position x)
        - Additional sway joints would need to be added to URDF for full visualization
        - slewing_joint: configurable rotation
        - hook_joint: configurable extension
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # Joint names from URDF
        msg.name = ['slewing_joint', 'trolley_joint', 'hook_joint']
        
        # Joint positions
        # - slewing_angle: rotation of crane
        # - trolley position: state[0]
        # - hook extension: hook_position
        msg.position = [
            self.slewing_angle,
            self.state[0],
            self.hook_position
        ]
        
        # Joint velocities
        msg.velocity = [
            0.0,           # slewing velocity (can be controlled separately)
            self.state[1], # trolley velocity
            0.0            # hook velocity (can be controlled separately)
        ]
        
        # Effort (forces/torques) - optional
        msg.effort = []
        
        self.joint_state_pub.publish(msg)
    
    def publish_state(self):
        """
        Publishes complete state information for monitoring and control.
        
        State vector: [x, x_dot, theta_x, theta_x_dot, theta_y, theta_y_dot, F_control]
        """
        msg = Float64MultiArray()
        msg.data = self.state.tolist() + [self.F_control]
        self.state_pub.publish(msg)
        
        # Log state periodically (every 2 seconds)
        if not hasattr(self, '_last_log_time'):
            self._last_log_time = self.get_clock().now()
        
        current_time = self.get_clock().now()
        if (current_time - self._last_log_time).nanoseconds / 1e9 > 2.0:
            self.get_logger().info(
                f'State: x={self.state[0]:.3f}m, '
                f'v={self.state[1]:.3f}m/s, '
                f'θ_x={np.degrees(self.state[2]):.2f}°, '
                f'ω_x={np.degrees(self.state[3]):.2f}°/s, '
                f'θ_y={np.degrees(self.state[4]):.2f}°, '
                f'ω_y={np.degrees(self.state[5]):.2f}°/s, '
                f'F={self.F_control:.2f}N'
            )
            self._last_log_time = current_time


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    node = SphericalPendulumDynamics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

