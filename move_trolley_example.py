#!/usr/bin/env python3
"""
Example script to move trolley joint 90 degrees using MoveIt/ROS2 Control
This uses the action server interface which is the recommended method.
Fixed to properly initialize and enable motors before movement.
"""

import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
import sys
import time


class TrolleyMover(Node):
    def __init__(self):
        super().__init__('trolley_mover')
        
        # Create service clients for motor control
        self.init_clients = {}
        self.enable_clients = {}
        self.position_mode_clients = {}
        
        joints = ['slewing_joint', 'trolley_joint', 'hook_joint']
        for joint in joints:
            self.init_clients[joint] = self.create_client(Trigger, f'/{joint}/init')
            self.enable_clients[joint] = self.create_client(Trigger, f'/{joint}/enable')
            self.position_mode_clients[joint] = self.create_client(Trigger, f'/{joint}/position_mode')
        
        self.action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/forward_position_controller/follow_joint_trajectory'
        )
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server(timeout_sec=10.0)
        if not self.action_client.server_is_ready():
            self.get_logger().error('Action server not available!')
            sys.exit(1)
        self.get_logger().info('Action server ready!')
    
    def initialize_motors(self):
        """Initialize all motors before movement"""
        self.get_logger().info('Initializing motors...')
        
        for joint_name, client in self.init_clients.items():
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'Init service for {joint_name} not available, skipping...')
                continue
            
            request = Trigger.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(f'✓ {joint_name} initialized')
                else:
                    self.get_logger().warn(f'⚠ {joint_name} init returned: {future.result().message}')
            else:
                self.get_logger().error(f'✗ {joint_name} init service call failed')
        
        time.sleep(0.5)  # Give motors time to initialize
    
    def set_position_mode(self):
        """Set all motors to position mode"""
        self.get_logger().info('Setting position mode...')
        
        for joint_name, client in self.position_mode_clients.items():
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'Position mode service for {joint_name} not available, skipping...')
                continue
            
            request = Trigger.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(f'✓ {joint_name} in position mode')
                else:
                    self.get_logger().warn(f'⚠ {joint_name} mode: {future.result().message}')
        
        time.sleep(0.5)
    
    def enable_motors(self):
        """Enable all motors for movement"""
        self.get_logger().info('Enabling motors...')
        
        for joint_name, client in self.enable_clients.items():
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(f'Enable service for {joint_name} not available, skipping...')
                continue
            
            request = Trigger.Request()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info(f'✓ {joint_name} enabled')
                else:
                    self.get_logger().warn(f'⚠ {joint_name} enable: {future.result().message}')
            else:
                self.get_logger().error(f'✗ {joint_name} enable service call failed')
        
        time.sleep(0.5)  # Give motors time to enable
    
    def move_trolley_90_degrees(self, current_positions=None):
        """
        Move trolley 90 degrees (0.09 meters).
        
        Args:
            current_positions: Dict with current joint positions {'hook_joint': x, 'trolley_joint': y, 'slewing_joint': z}
                             If None, will move trolley to absolute position 0.09m
        """
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['slewing_joint', 'trolley_joint', 'hook_joint']
        
        point = JointTrajectoryPoint()
        
        if current_positions:
            # Move relative to current position
            point.positions = [
                current_positions.get('slewing_joint', 0.0),
                current_positions.get('trolley_joint', 0.0) + 0.09,  # Add 0.09m to trolley
                current_positions.get('hook_joint', 0.0)
            ]
        else:
            # Move to absolute position (0.09m for trolley - 90 degree motor rotation)
            # Start with a smaller, achievable target
            point.positions = [0.0, 0.09, 0.0]  # [slewing, trolley, hook]
        
        point.time_from_start.sec = 15  # 15 seconds to complete (more time)
        point.velocities = [0.0, 0.0, 0.0]  # Optional: specify velocities
        point.accelerations = [0.0, 0.0, 0.0]
        
        goal_msg.trajectory.points = [point]
        
        # Set goal tolerances to be more lenient
        goal_msg.goal_tolerance = []
        goal_msg.goal_time_tolerance.sec = 5  # Allow extra time
        
        self.get_logger().info(f'Sending goal: trolley_joint = {point.positions[1]} m (slewing={point.positions[0]}, hook={point.positions[2]})')
        self.send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted!')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        error_codes = {
            0: 'SUCCESSFUL',
            -1: 'INVALID_GOAL',
            -2: 'INVALID_JOINTS',
            -3: 'OLD_HEADER_TIMESTAMP',
            -4: 'PATH_TOLERANCE_VIOLATED',
            -5: 'GOAL_TOLERANCE_VIOLATED'
        }
        
        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('=' * 60)
            self.get_logger().info('✓✓✓ Movement completed successfully! ✓✓✓')
            self.get_logger().info('=' * 60)
        else:
            error_name = error_codes.get(result.error_code, f'UNKNOWN({result.error_code})')
            self.get_logger().error('=' * 60)
            self.get_logger().error(f'✗ Movement failed: {error_name}')
            self.get_logger().error(f'Error code: {result.error_code}')
            
            if result.error_code == -5:
                self.get_logger().error('')
                self.get_logger().error('GOAL_TOLERANCE_VIOLATED means:')
                self.get_logger().error('  - Motors may not be properly enabled')
                self.get_logger().error('  - Check if motors are responding to commands')
                self.get_logger().error('  - Verify motor state with: ros2 topic echo /joint_states')
                self.get_logger().error('  - Check controller tolerance in tower_crane_ros2_control.yaml')
            
            self.get_logger().error('=' * 60)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Log current position during movement
        if hasattr(feedback, 'actual') and len(feedback.actual.positions) > 0:
            trolley_pos = feedback.actual.positions[1] if len(feedback.actual.positions) > 1 else 0.0
            self.get_logger().info(f'Moving... trolley at {trolley_pos:.4f}m', throttle_duration_sec=1.0)


def main():
    rclpy.init()
    
    mover = TrolleyMover()
    
    try:
        # Initialize motors before movement
        mover.get_logger().info('=' * 60)
        mover.get_logger().info('STEP 1: Initializing motors...')
        mover.get_logger().info('=' * 60)
        mover.initialize_motors()
        
        mover.get_logger().info('=' * 60)
        mover.get_logger().info('STEP 2: Setting position mode...')
        mover.get_logger().info('=' * 60)
        mover.set_position_mode()
        
        mover.get_logger().info('=' * 60)
        mover.get_logger().info('STEP 3: Enabling motors...')
        mover.get_logger().info('=' * 60)
        mover.enable_motors()
        
        mover.get_logger().info('=' * 60)
        mover.get_logger().info('STEP 4: Moving trolley 0.09m (90 degrees motor rotation)...')
        mover.get_logger().info('=' * 60)
        mover.move_trolley_90_degrees()
        
        # Keep spinning to receive feedback and result
        rclpy.spin(mover)
        
    except KeyboardInterrupt:
        mover.get_logger().info('Interrupted by user')
    finally:
        mover.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


