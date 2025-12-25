# How to Move Trolley 90 Degrees - Correct Commands

Based on your actual ROS2 system, here are the **correct** ways to move the trolley 90 degrees.

## Method 1: Using CANopen Target Service (Direct Motor Control) ⭐ RECOMMENDED

This sends the command directly to the motor controller. The value is in **motor encoder units** (likely degrees or encoder counts).

```bash
# Move trolley 90 degrees (direct motor command)
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"
```

**Note:** The exact units depend on your motor configuration. If 90 doesn't work, you may need to check:
- Current position: `ros2 topic echo /trolley_joint/joint_states --once`
- The value might be in encoder counts (e.g., 90 * encoder_resolution)

## Method 2: Using Joint Trajectory Controller (ROS2 Control)

This uses the `forward_position_controller` which expects positions in **meters** (for prismatic joints) or **radians** (for revolute joints).

Since trolley is a **prismatic joint**, 90 degrees of motor rotation = **0.09 meters** (90 × 0.001 m/degree).

```bash
# Move trolley 0.09 meters (90 degrees motor rotation)
ros2 topic pub --once /forward_position_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
{
  joint_names: ['slewing_joint', 'trolley_joint', 'hook_joint'],
  points: [
    {
      positions: [0.0, 0.09, 0.0],
      time_from_start: {sec: 5, nanosec: 0}
    }
  ]
}"
```

**Important:** This keeps slewing and hook at their current positions, only moves trolley.

## Method 3: Using Action Server (For MoveIt Integration)

If you're using MoveIt, use the action server:

```bash
# Check if action server is available
ros2 action list | grep forward_position_controller
```

Then use a Python script (see `move_trolley_example.py`) or:

```python
import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Create node and action client
rclpy.init()
node = rclpy.create_node('trolley_mover')
action_client = ActionClient(node, FollowJointTrajectory, '/forward_position_controller/follow_joint_trajectory')

# Wait for server
action_client.wait_for_server()

# Create goal
goal = FollowJointTrajectory.Goal()
goal.trajectory.joint_names = ['slewing_joint', 'trolley_joint', 'hook_joint']

point = JointTrajectoryPoint()
point.positions = [0.0, 0.09, 0.0]  # [slewing, trolley, hook] in meters/radians
point.time_from_start.sec = 5

goal.trajectory.points = [point]

# Send goal
future = action_client.send_goal_async(goal)
rclpy.spin_until_future_complete(node, future)
```

## Quick Reference: Check Current Position

Before moving, always check the current position:

```bash
# Check current trolley position (in meters for ROS2, or raw value for motor)
ros2 topic echo /trolley_joint/joint_states --once

# Or check all joints
ros2 topic echo /joint_states --once | grep trolley_joint
```

## Understanding the Units

Your system has two coordinate systems:

1. **Motor/Encoder Units** (used by `/trolley_joint/target` service):
   - Direct motor encoder counts or degrees
   - Use this for: `/trolley_joint/target` service

2. **ROS2/MoveIt Units** (used by `/forward_position_controller/joint_trajectory`):
   - **Trolley (prismatic):** Meters (m)
   - **Slewing (revolute):** Radians (rad)
   - **Hook (prismatic):** Meters (m)
   - Use this for: Trajectory controller, MoveIt

## Conversion Table

| Motor Rotation | Linear Movement (m) | Command Value (Method 1) | Command Value (Method 2) |
|---------------|-------------------|-------------------------|-------------------------|
| 90° | 0.09 m | `90.0` | `0.09` |
| 180° | 0.18 m | `180.0` | `0.18` |
| 360° | 0.36 m | `360.0` | `0.36` |
| 1000° | 1.0 m | `1000.0` | `1.0` |

## Recommended Approach

**For quick testing:** Use Method 1 (service call)
```bash
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"
```

**For coordinated motion:** Use Method 2 (trajectory controller)
```bash
ros2 topic pub --once /forward_position_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['slewing_joint', 'trolley_joint', 'hook_joint'], points: [{positions: [0.0, 0.09, 0.0], time_from_start: {sec: 5}}]}"
```

## Troubleshooting

1. **Service not found:** Make sure the CANopen driver is running
   ```bash
   ros2 service list | grep trolley_joint
   ```

2. **No movement:** Check if motor is enabled
   ```bash
   ros2 service call /trolley_joint/enable std_srvs/srv/Trigger
   ```

3. **Wrong direction:** Use negative value
   ```bash
   ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: -90.0}"
   ```

4. **Out of range:** Check joint limits (-2.0 m to 0.5 m)
   ```bash
   ros2 topic echo /trolley_joint/joint_states --once
   ```

## Example: Move Relative to Current Position

```bash
# 1. Get current position
CURRENT=$(ros2 topic echo /trolley_joint/joint_states --once | grep position | head -1 | awk '{print $2}')

# 2. Calculate target (current + 0.09m for 90 degrees)
TARGET=$(echo "$CURRENT + 0.09" | bc)

# 3. Move to target
ros2 topic pub --once /forward_position_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
{
  joint_names: ['slewing_joint', 'trolley_joint', 'hook_joint'],
  points: [
    {
      positions: [0.0, $TARGET, 0.0],
      time_from_start: {sec: 5, nanosec: 0}
    }
  ]
}"
```



