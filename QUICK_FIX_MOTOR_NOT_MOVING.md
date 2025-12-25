# QUICK FIX: Motor Not Moving

## What Happened

The motor didn't move because:
1. New encoder scaling in `bus.yml` needs to be reloaded
2. Motors need to be initialized and enabled

## Quick Fix (Do This Now)

### Step 1: Stop and Restart System

**Stop the current launch** (Ctrl+C in terminal running `hardware_bringup_real.launch.py`)

**Restart with fresh config:**
```bash
cd ~/appdata/canros
source install/setup.bash
ros2 launch tower_crane hardware_bringup_real.launch.py
```

### Step 2: Initialize Motors (NEW TERMINAL)

```bash
cd ~/appdata/canros
source install/setup.bash

# Initialize all motors
ros2 service call /slewing_joint/init std_srvs/srv/Trigger
ros2 service call /trolley_joint/init std_srvs/srv/Trigger
ros2 service call /hook_joint/init std_srvs/srv/Trigger

sleep 2

# Enable all motors
ros2 service call /slewing_joint/enable std_srvs/srv/Trigger
ros2 service call /trolley_joint/enable std_srvs/srv/Trigger
ros2 service call /hook_joint/enable std_srvs/srv/Trigger
```

### Step 3: Test Again

```bash
# Small test movement (1cm)
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

**Expected Result:** Motor should move! ✓

## What We Fixed

### 1. Encoder Scaling (bus.yml) ✅
```yaml
scale_pos_to_dev: 2777.78   # Converts meters → encoder counts
```
- ROS2 sends: 0.09 meters
- Motor gets: 250 encoder counts ✓

### 2. Relaxed Timeouts (tower_crane_ros2_control.yaml) ✅
```yaml
goal_time: 0.0  # No timeout during debugging
goal: 0.02      # 2cm tolerance instead of 1cm
```

### 3. Rebuilt Package ✅
```bash
colcon build --packages-select tower_crane
```

## Verify It's Working

### Monitor joint state:
```bash
ros2 topic echo /joint_states
```
Should show trolley position changing from 0.0 → 0.01

### Monitor encoder counts being sent:
```bash
ros2 topic echo /trolley_joint/target
```
Should show ~27.78 counts (0.01m × 2777.78)

## If Still Not Moving

Run the debug script:
```bash
cd ~/appdata/canros/src
./debug_motor_movement.sh
```

Or try direct motor control:
```bash
# Bypass trajectory controller, send raw encoder counts
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 250.0}"
```

If this works → Problem is in trajectory controller integration
If this doesn't work → Problem is at motor/CAN level

## Files Modified

- ✅ `crane/tower_crane/config/robot_control/bus.yml` - Added encoder scaling
- ✅ `crane/tower_crane/config/tower_crane_ros2_control.yaml` - Relaxed tolerances
- ✅ Package rebuilt

## Next Steps After Success

Once motors are moving:
1. Test with larger movements (0.09m = 90°)
2. Test slewing joint rotation
3. Test combined movements
4. Integrate with MoveIt for planning

## Summary

**The fix is in place, you just need to:**
1. 🔄 **Restart the launch file** to load new config
2. ▶️ **Initialize and enable motors**
3. ✅ **Test movement**

The encoder conversion (meters → counts) is now automatic! 🎉


