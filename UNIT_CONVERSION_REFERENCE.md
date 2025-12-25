# Unit Conversion Reference Card

## Quick Reference

| Joint | ROS2 Unit | Motor Unit | Conversion Factor (to encoder) |
|-------|-----------|------------|-------------------------------|
| **Slewing** | Radians | Encoder counts | × 159.1549 |
| **Trolley** | Meters | Encoder counts | × 2777.78 |
| **Hook** | Meters | Encoder counts | × 2777.78 |

## Common Conversions

### Slewing Joint (Revolute)

| Degrees | Radians | Encoder Counts |
|---------|---------|----------------|
| 0° | 0.0000 | 0 |
| 45° | 0.7854 | 125 |
| 90° | 1.5708 | 250 |
| 180° | 3.1416 | 500 |
| 360° | 6.2832 | 1000 |

**Formula:**
- Radians → Counts: `counts = radians × 159.1549`
- Counts → Radians: `radians = counts × 0.00628318`
- Degrees → Radians: `radians = degrees × π / 180`

### Trolley Joint (Prismatic)

| Motor Degrees | Meters | Encoder Counts |
|---------------|--------|----------------|
| 0° | 0.000 | 0 |
| 90° | 0.090 | 250 |
| 180° | 0.180 | 500 |
| 360° | 0.360 | 1000 |
| 1000° | 1.000 | 2778 |

**Formula:**
- Meters → Counts: `counts = meters × 2777.78`
- Counts → Meters: `meters = counts × 0.00036`
- Motor degrees → Meters: `meters = degrees × 0.001`

### Hook Joint (Prismatic)

Same as Trolley joint:
- Meters → Counts: `counts = meters × 2777.78`
- Counts → Meters: `meters = counts × 0.00036`

## Hardware Specs

- **Encoder Resolution:** 1000 counts per motor revolution
- **Motor Revolution:** 360 degrees = 1000 encoder counts
- **Trolley/Hook Ratio:** 1 degree motor rotation = 0.001 meters linear movement

## ROS2 Command Examples

### Example 1: Move trolley 90 degrees

```bash
# 90 degrees = 0.09 meters in ROS2
ros2 action send_goal /forward_position_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [slewing_joint, trolley_joint, hook_joint]
  points:
  - positions: [0.0, 0.09, 0.0]
    time_from_start: {sec: 5}
" --feedback
```

**What happens:**
- ROS2 sends: `0.09 meters`
- Driver converts: `0.09 × 2777.78 = 250 encoder counts`
- Motor receives: `250 counts` ✓

### Example 2: Rotate slewing 90 degrees

```bash
# 90 degrees = 1.5708 radians in ROS2
ros2 action send_goal /forward_position_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [slewing_joint, trolley_joint, hook_joint]
  points:
  - positions: [1.5708, 0.0, 0.0]
    time_from_start: {sec: 5}
" --feedback
```

**What happens:**
- ROS2 sends: `1.5708 radians`
- Driver converts: `1.5708 × 159.1549 = 250 encoder counts`
- Motor receives: `250 counts` ✓

### Example 3: Combined movement

```bash
# Slewing: 45°, Trolley: 180°, Hook: 360°
ros2 action send_goal /forward_position_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [slewing_joint, trolley_joint, hook_joint]
  points:
  - positions: [0.7854, 0.180, 0.360]
    time_from_start: {sec: 8}
" --feedback
```

**What happens:**
- Slewing: `0.7854 rad × 159.1549 = 125 counts`
- Trolley: `0.180 m × 2777.78 = 500 counts`
- Hook: `0.360 m × 2777.78 = 1000 counts`

## Verification Commands

### Check current position (in ROS2 units)
```bash
ros2 topic echo /joint_states
```

### Monitor encoder values being sent to motors
```bash
# For trolley
ros2 topic echo /trolley_joint/target

# For slewing
ros2 topic echo /slewing_joint/target

# For hook
ros2 topic echo /hook_joint/target
```

### Check controller status
```bash
ros2 control list_controllers
```

## Troubleshooting

### Motor not moving?

1. **Check if scaling is applied:**
   ```bash
   ros2 topic echo /trolley_joint/target
   ```
   Should show encoder counts (e.g., 250), not ROS2 units (e.g., 0.09)

2. **Check motor is enabled:**
   ```bash
   ros2 service call /trolley_joint/init std_srvs/srv/Trigger
   ros2 service call /trolley_joint/enable std_srvs/srv/Trigger
   ```

3. **Check controller is running:**
   ```bash
   ros2 control list_controllers
   # forward_position_controller should be "active"
   ```

### Motor moves wrong distance?

1. **Verify scaling factors in bus.yml:**
   - Slewing: `scale_pos_to_dev: 159.1549`
   - Trolley: `scale_pos_to_dev: 2777.78`
   - Hook: `scale_pos_to_dev: 2777.78`

2. **Check your encoder resolution:**
   - Current config assumes 1000 counts/revolution
   - If different, recalculate: `scale = counts_per_rev / (2 × π)` for revolute
   - Or: `scale = counts_per_rev / linear_distance_per_rev` for prismatic

## Python API Example

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class TowerCraneController(Node):
    def __init__(self):
        super().__init__('tower_crane_controller')
        self.action_client = ActionClient(
            self, 
            FollowJointTrajectory,
            '/forward_position_controller/follow_joint_trajectory'
        )
    
    def move_trolley(self, meters):
        """Move trolley by specified meters (will be converted to encoder counts automatically)"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['slewing_joint', 'trolley_joint', 'hook_joint']
        
        point = JointTrajectoryPoint()
        point.positions = [0.0, meters, 0.0]  # In meters (ROS2 units)
        point.time_from_start.sec = 5
        
        goal.trajectory.points = [point]
        
        self.action_client.wait_for_server()
        return self.action_client.send_goal_async(goal)

# Usage:
# rclpy.init()
# controller = TowerCraneController()
# future = controller.move_trolley(0.09)  # Move 90 degrees (0.09 m = 250 encoder counts)
# rclpy.spin_until_future_complete(controller, future)
```

Note: The conversion from meters (0.09) to encoder counts (250) happens automatically in the CANopen driver thanks to the `scale_pos_to_dev` parameter!


