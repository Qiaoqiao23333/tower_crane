#!/usr/bin/env python3
"""
Joint State Merger Node

This node merges joint_states from multiple sources:
1. ROS2 control (from /joint_states_control) - provides slewing_joint, hook_joint
2. Pendulum dynamics (from /joint_states_dynamics) - provides trolley_joint with physics

The merged joint_states are published to /joint_states for robot_state_publisher.

Usage:
    This node is automatically launched when use_pendulum_dynamics:=true in simulation.launch.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Time


class JointStateMerger(Node):
    """
    Merges joint states from ROS2 control and pendulum dynamics.
    
    Priority:
    - If pendulum dynamics is active: use trolley_joint from dynamics, others from control
    - If only control: use all from control
    - If only dynamics: use all from dynamics
    """
    
    def __init__(self):
        super().__init__('joint_state_merger')
        
        # Parameters
        self.declare_parameter('use_pendulum_dynamics', True)
        self.declare_parameter('control_topic', '/joint_states')
        self.declare_parameter('dynamics_topic', '/joint_states_dynamics')
        self.declare_parameter('output_topic', '/joint_states_merged')  # Avoid conflict with control's /joint_states
        
        self.use_pendulum = self.get_parameter('use_pendulum_dynamics').value
        control_topic = self.get_parameter('control_topic').value
        dynamics_topic = self.get_parameter('dynamics_topic').value
        output_topic = self.get_parameter('output_topic').value
        
        # State storage
        self.control_joint_state = None
        self.dynamics_joint_state = None
        self.control_received = False
        self.dynamics_received = False
        
        # Subscribers
        self.control_sub = self.create_subscription(
            JointState,
            control_topic,
            self.control_callback,
            10
        )
        
        if self.use_pendulum:
            self.dynamics_sub = self.create_subscription(
                JointState,
                dynamics_topic,
                self.dynamics_callback,
                10
            )
        
        # Publisher
        self.merged_pub = self.create_publisher(
            JointState,
            output_topic,
            10
        )
        
        # Timer for publishing merged states
        self.timer = self.create_timer(0.02, self.publish_merged_state)  # 50 Hz
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('Joint State Merger Node Started')
        self.get_logger().info(f'Use Pendulum Dynamics: {self.use_pendulum}')
        self.get_logger().info(f'Control Topic: {control_topic}')
        if self.use_pendulum:
            self.get_logger().info(f'Dynamics Topic: {dynamics_topic}')
        self.get_logger().info(f'Output Topic: {output_topic}')
        self.get_logger().info('=' * 70)
    
    def control_callback(self, msg):
        """Receive joint states from ROS2 control."""
        self.control_joint_state = msg
        self.control_received = True
    
    def dynamics_callback(self, msg):
        """Receive joint states from pendulum dynamics."""
        self.dynamics_joint_state = msg
        self.dynamics_received = True
    
    def publish_merged_state(self):
        """Merge and publish joint states."""
        merged_msg = JointState()
        merged_msg.header.stamp = self.get_clock().now().to_msg()
        
        if self.use_pendulum and self.dynamics_received and self.control_received:
            # Merge: use trolley_joint from dynamics, others from control
            merged_msg.name = []
            merged_msg.position = []
            merged_msg.velocity = []
            merged_msg.effort = []
            
            # Create dictionaries for easy lookup
            control_dict = {}
            dynamics_dict = {}
            
            if self.control_joint_state:
                for i, name in enumerate(self.control_joint_state.name):
                    control_dict[name] = {
                        'pos': self.control_joint_state.position[i] if i < len(self.control_joint_state.position) else 0.0,
                        'vel': self.control_joint_state.velocity[i] if i < len(self.control_joint_state.velocity) else 0.0,
                        'eff': self.control_joint_state.effort[i] if i < len(self.control_joint_state.effort) else 0.0,
                    }
            
            if self.dynamics_joint_state:
                for i, name in enumerate(self.dynamics_joint_state.name):
                    dynamics_dict[name] = {
                        'pos': self.dynamics_joint_state.position[i] if i < len(self.dynamics_joint_state.position) else 0.0,
                        'vel': self.dynamics_joint_state.velocity[i] if i < len(self.dynamics_joint_state.velocity) else 0.0,
                        'eff': self.dynamics_joint_state.effort[i] if i < len(self.dynamics_joint_state.effort) else 0.0,
                    }
            
            # Collect all unique joint names
            all_joints = set(control_dict.keys()) | set(dynamics_dict.keys())
            
            # Merge: trolley_joint from dynamics, others from control (fallback to dynamics)
            for joint_name in sorted(all_joints):
                merged_msg.name.append(joint_name)
                
                if joint_name == 'trolley_joint' and 'trolley_joint' in dynamics_dict:
                    # Use trolley from dynamics (with physics)
                    merged_msg.position.append(dynamics_dict['trolley_joint']['pos'])
                    merged_msg.velocity.append(dynamics_dict['trolley_joint']['vel'])
                    merged_msg.effort.append(dynamics_dict['trolley_joint']['eff'])
                elif joint_name in control_dict:
                    # Use from control
                    merged_msg.position.append(control_dict[joint_name]['pos'])
                    merged_msg.velocity.append(control_dict[joint_name]['vel'])
                    merged_msg.effort.append(control_dict[joint_name]['eff'])
                elif joint_name in dynamics_dict:
                    # Fallback to dynamics
                    merged_msg.position.append(dynamics_dict[joint_name]['pos'])
                    merged_msg.velocity.append(dynamics_dict[joint_name]['vel'])
                    merged_msg.effort.append(dynamics_dict[joint_name]['eff'])
                else:
                    # Default values
                    merged_msg.position.append(0.0)
                    merged_msg.velocity.append(0.0)
                    merged_msg.effort.append(0.0)
        
        elif self.control_received and self.control_joint_state:
            # Only control available
            merged_msg = self.control_joint_state
            merged_msg.header.stamp = self.get_clock().now().to_msg()
        
        elif self.use_pendulum and self.dynamics_received and self.dynamics_joint_state:
            # Only dynamics available
            merged_msg = self.dynamics_joint_state
            merged_msg.header.stamp = self.get_clock().now().to_msg()
        
        else:
            # No data yet
            return
        
        self.merged_pub.publish(merged_msg)


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    node = JointStateMerger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

