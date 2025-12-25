# Encoder Counts Unit Conversion Fix

## Problem

The motor driver only understands **encoder counts**, not ROS2 standard units (radians/meters).

Example calculation from the hardware:
- 90 degrees → (90/360) × (10000/10) = 0.25 × 1000 = **250 encoder counts**
- This means: **1000 encoder counts per motor revolution**

The `/forward_position_controller/follow_joint_trajectory` was sending commands in ROS2 units (radians for revolute joints, meters for prismatic joints), but these weren't being converted to encoder counts.

## Solution

Added proper unit conversion scaling factors to `bus.yml` for all three joints:

### 1. Slewing Joint (Revolute)

```yaml
# Unit conversion: ROS2 (radians) <-> Device (encoder counts)
# Encoder resolution: 1000 counts per revolution (actual hardware)
# Formula: ENCODER_RESOLUTION / (2 * pi)
# For slewing joint (revolute): 1000 / (2 * pi) = 159.15
scale_pos_to_dev: 159.1549      # Convert radians to encoder counts
scale_pos_from_dev: 0.00628318  # Convert encoder counts to radians
scale_vel_to_dev: 159.1549      # Convert rad/s to counts/s
scale_vel_from_dev: 0.00628318  # Convert counts/s to rad/s
```

**Calculation:**
- 1 revolution = 2π radians = 1000 encoder counts
- To convert radians → counts: multiply by 1000/(2π) = 159.1549
- To convert counts → radians: multiply by 2π/1000 = 0.00628318

### 2. Trolley Joint (Prismatic)

```yaml
# Unit conversion: ROS2 (meters) <-> Device (encoder counts)
# Trolley is prismatic: 1 degree motor rotation = 0.001 meters
# So: 1 meter = 1000 degrees = (1000/360) * 1000 encoder counts = 2777.78 counts
# Or: 1 motor revolution (1000 counts) = 360 degrees = 0.36 meters
# Therefore: 1 meter = 1000/0.36 = 2777.78 encoder counts
scale_pos_to_dev: 2777.78       # Convert meters to encoder counts
scale_pos_from_dev: 0.00036     # Convert encoder counts to meters
scale_vel_to_dev: 2777.78       # Convert m/s to counts/s
scale_vel_from_dev: 0.00036     # Convert counts/s to m/s
```

**Calculation:**
- 1 motor revolution = 1000 encoder counts = 360 degrees of rotation
- Based on trolley mechanism: 1 degree = 0.001 meters
- Therefore: 1 revolution = 360° × 0.001 m/° = 0.36 meters
- To convert meters → counts: multiply by 1000/0.36 = 2777.78
- To convert counts → meters: multiply by 0.36/1000 = 0.00036

### 3. Hook Joint (Prismatic)

Same as trolley joint:
```yaml
scale_pos_to_dev: 2777.78       # Convert meters to encoder counts
scale_pos_from_dev: 0.00036     # Convert encoder counts to meters
scale_vel_to_dev: 2777.78       # Convert m/s to counts/s
scale_vel_from_dev: 0.00036     # Convert counts/s to m/s
```

## How It Works

When you send a trajectory command via `/forward_position_controller/follow_joint_trajectory`:

1. **ROS2 sends in standard units:**
   - Slewing: radians (e.g., 1.57 rad = 90°)
   - Trolley: meters (e.g., 0.09 m)
   - Hook: meters (e.g., 0.5 m)

2. **CANopen driver applies scaling:**
   - Multiplies by `scale_pos_to_dev` to convert to encoder counts
   - Example for trolley: 0.09 m × 2777.78 = 250 encoder counts ✓

3. **Motor driver receives encoder counts:**
   - Exactly what the hardware expects!

4. **Feedback conversion:**
   - Motor reports position in encoder counts
   - Driver multiplies by `scale_pos_from_dev` to convert back to ROS2 units
   - ROS2 receives radians/meters as expected

## Verification

### Example: Move trolley 90 degrees

**Using Joint Trajectory Action:**
```bash
ros2 action send_goal /forward_position_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [slewing_joint, trolley_joint, hook_joint]
  points:
  - positions: [0.0, 0.09, 0.0]  # 0.09 meters = 90° motor rotation
    time_from_start:
      sec: 5
" --feedback
```

**What happens internally:**
- ROS2 sends: `0.09 meters`
- Driver converts: `0.09 × 2777.78 = 250 encoder counts`
- Motor receives: `250 counts` (exactly what it expects!)

### Example: Rotate slewing 90 degrees

```bash
ros2 action send_goal /forward_position_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [slewing_joint, trolley_joint, hook_joint]
  points:
  - positions: [1.5708, 0.0, 0.0]  # 1.5708 radians = 90°
    time_from_start:
      sec: 5
" --feedback
```

**What happens internally:**
- ROS2 sends: `1.5708 radians`
- Driver converts: `1.5708 × 159.1549 = 250 encoder counts`
- Motor receives: `250 counts` ✓

## Testing

After rebuilding the workspace, test with:

```bash
# Rebuild
cd ~/appdata/canros
colcon build --packages-select tower_crane

# Source
source install/setup.bash

# Launch the system
ros2 launch tower_crane hardware_bringup_real.launch.py

# Test trolley movement (90 degrees = 0.09 meters)
ros2 action send_goal /forward_position_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [slewing_joint, trolley_joint, hook_joint]
  points:
  - positions: [0.0, 0.09, 0.0]
    time_from_start: {sec: 5}
" --feedback
```

## Files Modified

- `/home/qiaoqiaochen/appdata/canros/src/crane/tower_crane/config/robot_control/bus.yml`
  - Added `scale_pos_to_dev` and `scale_pos_from_dev` for all three joints
  - Added `scale_vel_to_dev` and `scale_vel_from_dev` for velocity scaling

## Key Points

1. **No code changes needed** - only configuration in `bus.yml`
2. **Automatic bidirectional conversion:**
   - Commands: ROS2 units → Encoder counts
   - Feedback: Encoder counts → ROS2 units
3. **Works with all interfaces:**
   - Joint Trajectory Controller
   - MoveIt
   - Any ROS2 control interface

## Previous vs. New Behavior

### Before (WRONG):
- ROS2 sends: 0.09 meters
- Driver sends to motor: 0.09 (no conversion)
- Motor confused: expects 250 counts, got 0.09 ❌

### After (CORRECT):
- ROS2 sends: 0.09 meters
- Driver converts: 0.09 × 2777.78 = 250 counts
- Driver sends to motor: 250
- Motor happy: receives 250 counts as expected ✓


