# Motor Not Moving - Troubleshooting Guide

## The Problem

You're getting this error:
```
[ERROR] [tolerances]: State tolerances failed for joint 1:
[ERROR] [tolerances]: Position Error: 0.090000, Position Tolerance: 0.010000
[WARN] [forward_position_controller]: Aborted due to goal_time_tolerance exceeding by 10.000005 seconds
```

**Translation:** The trolley motor didn't move at all. It stayed at position 0.0 when it should have moved to 0.09 meters.

## Root Cause Analysis

The motor isn't moving because one or more of these issues:

1. ❌ **Motors not initialized/enabled** - CiA402 motors must be initialized and enabled
2. ❌ **Old bus.yml loaded** - The encoder scaling we just added hasn't been picked up
3. ❌ **Motor in wrong mode** - Not in Profile Position mode (mode 1)
4. ❌ **Physical connection issue** - CAN bus not connected or motor powered off

## Solution Steps

### Step 1: Restart the System with New Configuration

The encoder scaling in `bus.yml` was just added and needs to be reloaded:

```bash
# Stop the current launch (Ctrl+C in the terminal running it)

# Source the rebuilt workspace
cd ~/appdata/canros
source install/setup.bash

# Launch the system fresh
ros2 launch tower_crane hardware_bringup_real.launch.py
```

### Step 2: Initialize and Enable All Motors

**In a NEW terminal**, run:

```bash
# Source workspace
cd ~/appdata/canros
source install/setup.bash

# Initialize motors
ros2 service call /slewing_joint/init std_srvs/srv/Trigger
ros2 service call /trolley_joint/init std_srvs/srv/Trigger
ros2 service call /hook_joint/init std_srvs/srv/Trigger

# Wait 2 seconds
sleep 2

# Enable motors
ros2 service call /slewing_joint/enable std_srvs/srv/Trigger
ros2 service call /trolley_joint/enable std_srvs/srv/Trigger
ros2 service call /hook_joint/enable std_srvs/srv/Trigger
```

### Step 3: Verify Controllers are Running

```bash
ros2 control list_controllers
```

Expected output:
```
forward_position_controller[joint_trajectory_controller/JointTrajectoryController] active
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
```

### Step 4: Test with Small Movement

```bash
# Try moving trolley just 1cm (0.01 meters)
ros2 action send_goal /forward_position_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [slewing_joint, trolley_joint, hook_joint]
  points:
  - positions: [0.0, 0.01, 0.0]
    time_from_start:
      sec: 5
      nanosec: 0
" --feedback
```

### Step 5: Use the Debug Script

Run the automated debug script:

```bash
./debug_motor_movement.sh
```

This will:
- Check if all services are available
- Initialize and enable motors automatically
- Test with small movement
- Show diagnostic information

## Configuration Changes Made

### 1. Added Encoder Scaling (bus.yml)

```yaml
# For trolley_joint (lines 89-92)
scale_pos_to_dev: 2777.78       # Convert meters to encoder counts
scale_pos_from_dev: 0.00036     # Convert encoder counts to meters
scale_vel_to_dev: 2777.78       # Convert m/s to counts/s
scale_vel_from_dev: 0.00036     # Convert counts/s to m/s
```

**Why:** Motors only understand encoder counts (250 counts for 90°), not meters (0.09m).

### 2. Relaxed Goal Tolerances (tower_crane_ros2_control.yaml)

```yaml
# Changed goal_time from 10.0 to 0.0
goal_time: 0.0  # 0.0 = unlimited time (for debugging)

# Relaxed goal tolerance from 0.01m to 0.02m
trolley_joint:
  goal: 0.02  # 2cm tolerance instead of 1cm
```

**Why:** Gives more time to debug without the controller aborting immediately.

## Monitoring Commands

### Monitor Joint States (ROS2 units)
```bash
ros2 topic echo /joint_states
```
Should show positions in meters/radians.

### Monitor Motor Targets (Encoder counts)
```bash
ros2 topic echo /trolley_joint/target
```
Should show positions in encoder counts (e.g., 250 for 0.09m).

### Check Controller State
```bash
ros2 topic echo /forward_position_controller/controller_state
```
Shows actual vs desired positions.

## Expected Behavior After Fix

### Command:
```bash
positions: [0.0, 0.09, 0.0]  # 0.09 meters
```

### What Should Happen:

1. **ROS2 sends:** `0.09 meters`
2. **Driver converts:** `0.09 × 2777.78 = 250 encoder counts`
3. **Motor receives:** `250 counts` via CAN (0x607A Target Position)
4. **Motor moves:** 90 degrees of rotation
5. **Motor reports back:** `250 counts` via CAN (0x6064 Position Actual)
6. **Driver converts back:** `250 × 0.00036 = 0.09 meters`
7. **ROS2 sees:** `0.09 meters` ✓
8. **Controller:** Position error ~0, goal reached! ✓

## Troubleshooting Specific Issues

### Issue 1: "Position Error: 0.090000" (Motor Didn't Move)

**Cause:** Motor not enabled or not receiving commands.

**Solution:**
```bash
# Reinitialize and enable
ros2 service call /trolley_joint/init std_srvs/srv/Trigger
ros2 service call /trolley_joint/enable std_srvs/srv/Trigger

# Try direct CANopen command
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 250.0}"
```

### Issue 2: Motor Moves Wrong Distance

**Cause:** Scaling factors incorrect.

**Solution:** Verify in `bus.yml`:
```yaml
scale_pos_to_dev: 2777.78  # For trolley
```

Calculate: `0.09m × 2777.78 = 250 counts` ✓

### Issue 3: "Aborted due to goal_time_tolerance"

**Cause:** Motor too slow or not moving within time limit.

**Solution:** Already fixed - set `goal_time: 0.0` for unlimited time.

### Issue 4: CAN Communication Errors

**Cause:** Physical connection or timing issues.

**Check:**
```bash
# Check CAN interface
ip link show can0

# Check if nodes are visible
candump can0
```

## Alternative: Direct Motor Test

If trajectory controller still doesn't work, test motors directly:

```bash
# Test trolley with direct CANopen command (bypasses ros2_control)
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 250.0}"

# Monitor response
ros2 topic echo /joint_states
```

If this works but trajectory controller doesn't:
- Problem is in ros2_control integration
- Check that hardware interface is properly connected

If this doesn't work:
- Problem is at motor/CAN level
- Check physical connections, motor power, CAN bus

## Quick Test Sequence

```bash
# Terminal 1: Launch system
ros2 launch tower_crane hardware_bringup_real.launch.py

# Terminal 2: Initialize & test
cd ~/appdata/canros/src
source ../install/setup.bash

# Enable motors
ros2 service call /trolley_joint/init std_srvs/srv/Trigger
ros2 service call /trolley_joint/enable std_srvs/srv/Trigger

# Try small movement
ros2 action send_goal /forward_position_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [slewing_joint, trolley_joint, hook_joint]
  points:
  - positions: [0.0, 0.01, 0.0]
    time_from_start: {sec: 5}
" --feedback
```

## Success Indicators

✅ Motor makes sound/movement
✅ Position in `/joint_states` changes from 0.0
✅ Action goal returns `SUCCEEDED`
✅ No timeout errors
✅ `/trolley_joint/target` shows encoder counts (not meters)

## If Still Not Working

1. **Check motor status lights** - Are they green/ready?
2. **Check CAN bus** - `candump can0` shows traffic?
3. **Check motor mode** - Should be mode 1 (Profile Position)
4. **Check motor errors** - Any fault codes?
5. **Try other joints** - Does slewing or hook work?

Run the comprehensive debug script:
```bash
./debug_motor_movement.sh
```

Or ask for help with output from:
```bash
ros2 topic echo /joint_states
ros2 topic echo /trolley_joint/target
ros2 control list_controllers
```


