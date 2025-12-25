#!/usr/bin/env python3
"""
Simple test: Use direct CANopen services to move the trolley
Bypasses the trajectory controller entirely
"""

import rclpy
from rclpy.node import Node
from canopen_interfaces.srv import COTargetDouble
from sensor_msgs.msg import JointState
import sys
import time


class SimpleTrolleyTest(Node):
    def __init__(self):
        super().__init__('simple_trolley_test')
        
        # Service client for direct motor control
        self.target_client = self.create_client(COTargetDouble, '/trolley_joint/target')
        
        # Subscribe to joint states to monitor position
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.current_position = None
        self.initial_position = None
        
    def joint_state_callback(self, msg):
        """Track current trolley position"""
        try:
            idx = msg.name.index('trolley_joint')
            self.current_position = msg.position[idx]
        except (ValueError, IndexError):
            pass
    
    def wait_for_joint_states(self, timeout=5.0):
        """Wait until we receive joint states"""
        self.get_logger().info('Waiting for joint states...')
        start = time.time()
        while self.current_position is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.current_position is not None:
            self.get_logger().info(f'✓ Current trolley position: {self.current_position:.4f}m')
            return True
        else:
            self.get_logger().error('✗ No joint states received')
            return False
    
    def send_target(self, target_value):
        """Send target position directly to motor via CANopen"""
        self.get_logger().info(f'Sending target: {target_value} (motor units)...')
        
        if not self.target_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('✗ Target service not available!')
            return False
        
        request = COTargetDouble.Request()
        request.target = float(target_value)
        
        try:
            future = self.target_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.result() is not None:
                if future.result().success:
                    self.get_logger().info('✓ Target command sent successfully')
                    return True
                else:
                    self.get_logger().error(f'✗ Target command failed: {future.result()}')
                    return False
            else:
                self.get_logger().error('✗ Service call timeout')
                return False
        except Exception as e:
            self.get_logger().error(f'✗ Exception: {e}')
            return False
    
    def monitor_movement(self, duration=10.0, check_interval=0.5):
        """Monitor position changes over time"""
        self.get_logger().info(f'Monitoring movement for {duration} seconds...')
        self.get_logger().info('-' * 60)
        
        start_time = time.time()
        last_position = self.current_position
        position_history = []
        
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=check_interval)
            
            if self.current_position is not None:
                elapsed = time.time() - start_time
                change = self.current_position - self.initial_position if self.initial_position else 0
                
                position_history.append((elapsed, self.current_position))
                
                self.get_logger().info(
                    f'  t={elapsed:5.1f}s: position={self.current_position:7.4f}m, '
                    f'change={change:+7.4f}m'
                )
                
                last_position = self.current_position
        
        self.get_logger().info('-' * 60)
        return position_history


def main():
    rclpy.init()
    
    test = SimpleTrolleyTest()
    
    try:
        test.get_logger().info('=' * 70)
        test.get_logger().info('SIMPLE TROLLEY MOVEMENT TEST')
        test.get_logger().info('Using direct CANopen commands (bypassing trajectory controller)')
        test.get_logger().info('=' * 70)
        
        # Step 1: Wait for initial joint states
        test.get_logger().info('\n--- STEP 1: Get Initial Position ---')
        if not test.wait_for_joint_states():
            test.get_logger().error('Cannot proceed without joint states')
            sys.exit(1)
        
        test.initial_position = test.current_position
        test.get_logger().info(f'Initial position recorded: {test.initial_position:.4f}m')
        
        # Step 2: Send movement command
        test.get_logger().info('\n--- STEP 2: Send Movement Command ---')
        test.get_logger().info('Commanding: Move 90 degrees (motor rotation)')
        
        if not test.send_target(90.0):
            test.get_logger().error('Failed to send target command')
            sys.exit(1)
        
        # Step 3: Monitor movement
        test.get_logger().info('\n--- STEP 3: Monitor Movement ---')
        position_history = test.monitor_movement(duration=10.0, check_interval=0.5)
        
        # Step 4: Analyze results
        test.get_logger().info('\n--- STEP 4: Results ---')
        final_position = test.current_position
        
        if test.initial_position is not None and final_position is not None:
            total_change = final_position - test.initial_position
            
            test.get_logger().info(f'Initial position: {test.initial_position:.4f}m')
            test.get_logger().info(f'Final position:   {final_position:.4f}m')
            test.get_logger().info(f'Total change:     {total_change:+.4f}m')
            
            if abs(total_change) < 0.001:  # Less than 1mm
                test.get_logger().error('\n' + '=' * 70)
                test.get_logger().error('✗✗✗ MOTOR DID NOT MOVE AT ALL! ✗✗✗')
                test.get_logger().error('=' * 70)
                test.get_logger().error('')
                test.get_logger().error('The CANopen target command was accepted,')
                test.get_logger().error('but the motor position did not change.')
                test.get_logger().error('')
                test.get_logger().error('Possible causes:')
                test.get_logger().error('  1. Motor is not actually enabled (despite service saying so)')
                test.get_logger().error('  2. Motor is in wrong operation mode')
                test.get_logger().error('  3. Target value is in wrong units (90 might be too small)')
                test.get_logger().error('  4. Position feedback is stuck/not updating')
                test.get_logger().error('  5. Hardware issue: brake engaged, motor fault, etc.')
                test.get_logger().error('')
                test.get_logger().error('Next steps:')
                test.get_logger().error('  - Check /trolley_joint/tpdo topic for raw CANopen data')
                test.get_logger().error('  - Try a much larger target value (e.g., 1000)')
                test.get_logger().error('  - Check motor statusword and actual position via SDO')
            elif abs(total_change) < 0.01:  # Less than 1cm
                test.get_logger().warn('\n' + '=' * 70)
                test.get_logger().warn('⚠⚠⚠ MOTOR MOVED SLIGHTLY BUT NOT ENOUGH ⚠⚠⚠')
                test.get_logger().warn('=' * 70)
                test.get_logger().warn(f'Motor moved {total_change*1000:.1f}mm, expected ~90mm')
                test.get_logger().warn('Target value might be in wrong units')
            else:
                test.get_logger().info('\n' + '=' * 70)
                test.get_logger().info('✓✓✓ MOTOR MOVED SUCCESSFULLY! ✓✓✓')
                test.get_logger().info('=' * 70)
                test.get_logger().info(f'Motor moved {total_change*1000:.1f}mm')
                
                if abs(total_change - 0.09) < 0.01:
                    test.get_logger().info('Movement matches expected 90 degrees (0.09m)!')
                else:
                    test.get_logger().info(f'Expected 0.09m, got {total_change:.4f}m')
                    test.get_logger().info('Unit conversion may need adjustment')
        
        test.get_logger().info('\n' + '=' * 70)
        
    except KeyboardInterrupt:
        test.get_logger().info('\nInterrupted by user')
    finally:
        test.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


