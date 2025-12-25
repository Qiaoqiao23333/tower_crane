#!/usr/bin/env python3
"""
Diagnostic script that checks motor state and moves trolley
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from canopen_interfaces.srv import COReadID, COTargetDouble
import sys
import time


class TrolleyDiagnostic(Node):
    def __init__(self):
        super().__init__('trolley_diagnostic')
        
        # Create service clients
        self.init_client = self.create_client(Trigger, '/trolley_joint/init')
        self.enable_client = self.create_client(Trigger, '/trolley_joint/enable')
        self.position_mode_client = self.create_client(Trigger, '/trolley_joint/position_mode')
        self.sdo_read_client = self.create_client(COReadID, '/trolley_joint/sdo_read')
        self.target_client = self.create_client(COTargetDouble, '/trolley_joint/target')
        
    def read_sdo(self, index, subindex):
        """Read an SDO value"""
        if not self.sdo_read_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('SDO read service not available')
            return None
        
        request = COReadID.Request()
        request.index = index
        request.subindex = subindex
        
        future = self.sdo_read_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            return future.result()
        return None
    
    def check_statusword(self):
        """Check motor statusword (0x6041)"""
        self.get_logger().info('Reading Statusword (0x6041)...')
        result = self.read_sdo(0x6041, 0)
        if result and result.success:
            status = result.data
            self.get_logger().info(f'  Statusword: 0x{status:04X}')
            
            # Decode status bits
            if status & 0x0008:
                self.get_logger().info('  ✓ Voltage enabled')
            if status & 0x0010:
                self.get_logger().info('  ⚠ Warning')
            if status & 0x0020:
                self.get_logger().info('  ✓ Switch on disabled')
            if status & 0x0040:
                self.get_logger().info('  ✓ Quick stop')
            
            # Check state
            state_bits = status & 0x006F
            if state_bits == 0x0027:
                self.get_logger().info('  ✓✓✓ Motor is OPERATION ENABLED')
                return True
            elif state_bits == 0x0021:
                self.get_logger().warn('  ⚠ Motor is READY TO SWITCH ON')
            elif state_bits == 0x0023:
                self.get_logger().warn('  ⚠ Motor is SWITCHED ON (not enabled)')
            else:
                self.get_logger().warn(f'  ⚠ Unknown state: 0x{state_bits:04X}')
            return False
        else:
            self.get_logger().error('  ✗ Failed to read statusword')
            return False
    
    def check_operation_mode(self):
        """Check current operation mode (0x6061)"""
        self.get_logger().info('Reading Operation Mode Display (0x6061)...')
        result = self.read_sdo(0x6061, 0)
        if result and result.success:
            mode = result.data
            modes = {
                0: 'No mode',
                1: 'Profile Position Mode',
                3: 'Profile Velocity Mode',
                6: 'Homing Mode',
                7: 'Interpolated Position Mode',
                8: 'Cyclic Synchronous Position'
            }
            mode_name = modes.get(mode, f'Unknown ({mode})')
            self.get_logger().info(f'  Current mode: {mode_name}')
            
            if mode == 1:
                self.get_logger().info('  ✓ In Profile Position Mode')
                return True
            else:
                self.get_logger().warn(f'  ⚠ Not in Profile Position Mode (mode={mode})')
                return False
        else:
            self.get_logger().error('  ✗ Failed to read operation mode')
            return False
    
    def check_position(self):
        """Check current position (0x6064)"""
        self.get_logger().info('Reading Position Actual Value (0x6064)...')
        result = self.read_sdo(0x6064, 0)
        if result and result.success:
            pos = result.data
            # Handle signed 32-bit integer
            if pos > 0x7FFFFFFF:
                pos = pos - 0x100000000
            self.get_logger().info(f'  Current position: {pos} encoder units')
            return pos
        else:
            self.get_logger().error('  ✗ Failed to read position')
            return None
    
    def check_target_position(self):
        """Check target position (0x607A)"""
        self.get_logger().info('Reading Target Position (0x607A)...')
        result = self.read_sdo(0x607A, 0)
        if result and result.success:
            pos = result.data
            # Handle signed 32-bit integer
            if pos > 0x7FFFFFFF:
                pos = pos - 0x100000000
            self.get_logger().info(f'  Target position: {pos} encoder units')
            return pos
        else:
            self.get_logger().error('  ✗ Failed to read target position')
            return None
    
    def initialize_motor(self):
        """Initialize motor"""
        self.get_logger().info('Initializing motor...')
        if not self.init_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('  Init service not available')
            return False
        
        request = Trigger.Request()
        future = self.init_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info('  ✓ Initialized')
            return True
        else:
            self.get_logger().error('  ✗ Init failed')
            return False
    
    def set_position_mode(self):
        """Set position mode"""
        self.get_logger().info('Setting position mode...')
        if not self.position_mode_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('  Position mode service not available')
            return False
        
        request = Trigger.Request()
        future = self.position_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            # Even if it returns False, it might already be in the mode
            msg = future.result().message if future.result().message else '(no message)'
            self.get_logger().info(f'  Position mode response: {msg}')
            return True
        else:
            self.get_logger().error('  ✗ Failed to set position mode')
            return False
    
    def enable_motor(self):
        """Enable motor"""
        self.get_logger().info('Enabling motor...')
        if not self.enable_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('  Enable service not available')
            return False
        
        request = Trigger.Request()
        future = self.enable_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info('  ✓ Enabled')
            return True
        else:
            msg = future.result().message if future.result() else 'No response'
            self.get_logger().error(f'  ✗ Enable failed: {msg}')
            return False
    
    def move_motor(self, target):
        """Send target position directly via CANopen"""
        self.get_logger().info(f'Sending target position: {target}...')
        if not self.target_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('  Target service not available')
            return False
        
        request = COTargetDouble.Request()
        request.target = float(target)
        
        future = self.target_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None and future.result().success:
            self.get_logger().info('  ✓ Target sent successfully')
            return True
        else:
            self.get_logger().error('  ✗ Failed to send target')
            return False


def main():
    rclpy.init()
    
    diag = TrolleyDiagnostic()
    
    try:
        diag.get_logger().info('=' * 70)
        diag.get_logger().info('TROLLEY MOTOR DIAGNOSTIC AND MOVEMENT TEST')
        diag.get_logger().info('=' * 70)
        
        # Phase 1: Check initial state
        diag.get_logger().info('\n--- PHASE 1: Initial State Check ---')
        status_ok = diag.check_statusword()
        mode_ok = diag.check_operation_mode()
        initial_pos = diag.check_position()
        initial_target = diag.check_target_position()
        
        # Phase 2: Initialize and enable
        diag.get_logger().info('\n--- PHASE 2: Initialize and Enable ---')
        diag.initialize_motor()
        time.sleep(0.5)
        
        diag.set_position_mode()
        time.sleep(0.5)
        
        diag.enable_motor()
        time.sleep(1.0)
        
        # Phase 3: Check state after enabling
        diag.get_logger().info('\n--- PHASE 3: State After Enabling ---')
        status_ok = diag.check_statusword()
        mode_ok = diag.check_operation_mode()
        
        if not status_ok:
            diag.get_logger().error('\n✗✗✗ Motor is not in operation enabled state!')
            diag.get_logger().error('Cannot proceed with movement.')
            sys.exit(1)
        
        if not mode_ok:
            diag.get_logger().warn('\n⚠⚠⚠ Motor may not be in correct mode')
            diag.get_logger().warn('Will attempt movement anyway...')
        
        # Phase 4: Move motor
        diag.get_logger().info('\n--- PHASE 4: Direct CANopen Movement (90 degrees) ---')
        diag.move_motor(90.0)
        
        # Phase 5: Monitor movement
        diag.get_logger().info('\n--- PHASE 5: Monitoring Movement ---')
        for i in range(10):
            time.sleep(1.0)
            pos = diag.check_position()
            target = diag.check_target_position()
            
            if pos is not None and target is not None:
                error = abs(target - pos)
                diag.get_logger().info(f'  Step {i+1}: Position={pos}, Target={target}, Error={error}')
                
                if error < 10:  # Within 10 encoder units
                    diag.get_logger().info('\n✓✓✓ Movement completed!')
                    break
        else:
            diag.get_logger().warn('\n⚠ Movement monitoring timeout')
        
        # Final state
        diag.get_logger().info('\n--- FINAL STATE ---')
        final_pos = diag.check_position()
        final_target = diag.check_target_position()
        
        if initial_pos is not None and final_pos is not None:
            delta = final_pos - initial_pos
            diag.get_logger().info(f'Position change: {delta} encoder units')
            
            if abs(delta) < 5:
                diag.get_logger().error('\n✗✗✗ MOTOR DID NOT MOVE!')
                diag.get_logger().error('Possible causes:')
                diag.get_logger().error('  1. Motor not receiving commands')
                diag.get_logger().error('  2. Motor in wrong mode')
                diag.get_logger().error('  3. Hardware issue or brake engaged')
                diag.get_logger().error('  4. CANopen communication problem')
            else:
                diag.get_logger().info(f'\n✓✓✓ MOTOR MOVED {delta} encoder units!')
        
        diag.get_logger().info('=' * 70)
        
    except KeyboardInterrupt:
        diag.get_logger().info('Interrupted by user')
    finally:
        diag.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


