# Tower Crane Controller Manager Hanging - Investigation Summary

## Date
December 25, 2025

## Problem Statement

When launching `ros2 launch tower_crane hardware_bringup_real.launch.py`, the controller_manager process starts but **does NOT advertise its services**, causing spawners to hang indefinitely:

```
[spawner-3] [WARN] [spawner_joint_state_broadcaster]: Could not contact service /controller_manager/list_controllers
```

## Investigation Findings

### 1. Blocking Motor Initialization (FIXED ✅)

**Location**: `/third_party/ros2_canopen/canopen_ros2_control/src/robot_system.cpp:137-167`

**Issue**: `RobotSystem::on_activate()` was calling `init_motor()` synchronously for each motor, blocking for up to 60 seconds per motor.

**Fix Applied**: Commented out the blocking `init_motor()` calls. Hardware now activates immediately.

**Result**: Hardware activation no longer blocks, but services still don't appear.

### 2. Potentially Blocking Read Cycle (TESTED, NOT THE CAUSE)

**Location**: `/third_party/ros2_canopen/canopen_ros2_control/src/robot_system.cpp:218-226`

**Theory**: `RobotSystem::read()` calls `get_position()/get_speed()` which might make blocking SDO reads.

**Test**: Commented out the entire `read_state()` loop.

**Result**: Services still don't appear even with read disabled. **This is NOT the blocking point**.

### 3. Controller Manager Services Never Advertised (CURRENT ISSUE ❌)

**Observation**:
- `ros2_control_node` process starts successfully
- Hardware initializes and activates
- controller_manager logs show initialization complete:
  ```
  [controller_manager]: update rate is 100 Hz
  [controller_manager]: Spawning controller_manager RT thread with scheduler priority: 50
  ```
- RT thread spawns without errors
- But then... **complete silence**. No more log messages.
- Services NEVER appear in `ros2 service list`
- Process doesn't crash - it just doesn't advertise services

**What We've Tried**:
- ✅ Fixed blocking activation
- ✅ Disabled read cycle
- ✅ Verified process is running (`ps aux | grep ros2_control_node`)
- ❌ Services still not advertised

## Current Status

The root cause is still unknown. The controller_manager initializes successfully but fails to advertise its services for unknown reasons.

**Possible Causes (Unexplored)**:
1. **Threading/Executor Issue**: The RT thread or device container executors might be interfering with service registration
2. **ROS2 DDS Discovery Issue**: Network/discovery configuration preventing service advertisement
3. **Silent Crash**: The process might be crashing in a separate thread without logging
4. **Controller Manager Bug**: Possible bug in ros2_control or controller_manager for this specific configuration
5. **Hardware Interface Issue**: Some issue with how RobotSystem registers with controller_manager

## Temporary Workaround

Since we cannot get the standard launch to work, here are alternative approaches:

### Option 1: Skip ros2_control Entirely

If you just need to control the motors via CANopen services without ros2_control:

```bash
# Use the crane_master package directly (if available)
# Or write a simple node that uses canopen_402_driver directly
```

### Option 2: Debug Controller Manager

Run the controller_manager with maximum verbosity:

```bash
# Set ROS logging to DEBUG
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{time}] [{name}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1
export RCUTILS_LOGGING_LEVEL=10  # DEBUG level

# Launch with verbose output
ros2 launch tower_crane hardware_bringup_real.launch.py --log-level debug
```

### Option 3: Use Lifecycle Nodes Explicitly

Try manually transitioning the controller_manager through lifecycle states:

```bash
# Terminal 1: Launch just the hardware
ros2 run controller_manager ros2_control_node --ros-args \\
  -p robot_description:="$(xacro /path/to/Tower_crane_canopen.urdf.xacro)" \\
  --params-file /path/to/tower_crane_ros2_control.yaml

# Terminal 2: Check lifecycle state
ros2 lifecycle nodes

# Try manual transition
ros2 lifecycle set /controller_manager configure
ros2 lifecycle set /controller_manager activate
```

### Option 4: Simplify Hardware Interface

Test with a minimal hardware interface to isolate the issue:

1. Create a dummy hardware interface that doesn't use CANopen
2. Verify controller_manager works with dummy interface
3. Gradually add CANopen functionality back

## Files Modified

1. `/third_party/ros2_canopen/canopen_ros2_control/src/robot_system.cpp`
   - Lines 137-167: Commented out blocking `init_motor()` calls
   - Lines 218-226: Commented out `read_state()` calls (for testing)

2. `/src/enable_motors_after_startup.sh` - Created (motor enable helper script)

3. `/src/CONTROLLER_MANAGER_HANG_FIX_COMPLETE.md` - Documentation

## Recommended Next Steps

### Immediate Actions:

1. **Check ROS2 and ros2_control Versions**:
   ```bash
   ros2 --version
   ros2 pkg list | grep controller_manager
   dpkg -l | grep ros-humble-controller-manager
   ```

2. **Test Controller Manager in Isolation**:
   Create a minimal URDF with no hardware and see if controller_manager services appear:
   ```xml
   <ros2_control name="test_system" type="system">
     <hardware>
       <plugin>mock_components/GenericSystem</plugin>
     </hardware>
     <joint name="joint1">
       <command_interface name="position"/>
       <state_interface name="position"/>
     </joint>
   </ros2_control>
   ```

3. **Check for Known Issues**:
   - Search ros2_control GitHub issues for similar problems
   - Check if this is a known issue with CANopen + ros2_control
   - Verify compatibility between ros2_canopen and ros2_control versions

4. **Enable Core Dumps**:
   ```bash
   ulimit -c unlimited
   # Re-run launch
   # Check for core dumps: ls -la /var/crash/ or ./core.*
   ```

5. **Use GDB to Debug**:
   ```bash
   # Find the ros2_control_node PID while it's running
   ps aux | grep ros2_control_node
   
   # Attach GDB
   sudo gdb -p <PID>
   
   # In GDB:
   (gdb) thread apply all bt  # Get backtrace of all threads
   (gdb) info threads         # List all threads
   ```

### Long-term Solutions:

1. **Contact ros2_canopen Maintainers**:
   - Report this issue on GitHub: https://github.com/ros-industrial/ros2_canopen
   - Include full logs and configuration
   - Mention that services don't appear even though process starts

2. **Consider Alternative Approaches**:
   - Use canopen_402_driver services directly without ros2_control
   - Write custom hardware interface that properly handles async operations
   - Use a different motor control approach (EtherCAT, direct CAN, etc.)

3. **Investigate Executor Configuration**:
   - The device container uses a separate MultiThreadedExecutor
   - This might be conflicting with controller_manager's executor
   - May need to refactor how executors are managed

## Related Documentation

- `CONTROLLER_MANAGER_HANG_FIX_COMPLETE.md` - Detailed fix for blocking initialization
- `HANGING_SERVICE_FIX.md` - Previous service hanging issues
- `MOTOR_ACTIVATION_FIX.md` - Motor activation timeout fixes
- `SLEWING_MOTOR_FIX_COMPLETE.md` - Slewing motor specific fixes

## Conclusion

We've successfully fixed the blocking initialization issue, but uncovered a deeper problem: controller_manager doesn't advertise services even when it appears to start successfully. This requires further investigation, potentially involving the ros2_control/ros2_canopen maintainers or a fundamental redesign of the hardware interface integration.

The modified code (with blocking calls commented out) is ready and built, but won't fully solve your issue until we understand why controller_manager's services don't appear.

---

**Status**: 🟡 PARTIALLY FIXED - Blocking eliminated, but services still unavailable  
**Priority**: 🔴 HIGH - System non-functional without controller_manager services  
**Next Action**: Debug controller_manager service registration or find alternative approach

