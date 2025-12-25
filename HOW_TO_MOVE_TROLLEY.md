# How to Move the Trolley Joint

## Understanding the Trolley Joint

The `trolley_joint` is a **prismatic (linear)** joint, not a rotational joint. It moves along the Y-axis.

- **Joint Type:** Prismatic (linear movement)
- **Units in ROS2/MoveIt:** **Meters** (m)
- **Units in Motor Controller:** **Degrees** (motor rotation)
- **Conversion Factor:** `METERS_PER_DEGREE_TROLLEY = 0.001` (1 degree = 0.001 meters)
- **Joint Limits:** -2.0 m to 0.5 m

## If You Want to Move 90 Degrees of Motor Rotation

If you want the motor to rotate 90 degrees, that translates to:
- **90 degrees × 0.001 m/degree = 0.09 meters**

## Methods to Command the Trolley

### Method 1: Direct CANopen Service (Simplest) ⭐ RECOMMENDED

This sends the command directly to the motor controller:

```bash
# Move trolley motor 90 degrees (direct motor command)
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"
```

**Note:** The value is in motor encoder units (degrees or encoder counts depending on your motor configuration).

### Method 2: Using Joint Trajectory Controller (ROS2 Control)

This uses the `forward_position_controller` which expects positions in **meters** for prismatic joints:

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

**Note:** This keeps slewing and hook at their current positions, only moves trolley.

### Method 3: Using MoveIt (For Planning & Execution)

If you're using MoveIt, you can send a goal via the action server:

```python
# Python script example
import rclpy
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient

class TrolleyMover(Node):
    def __init__(self):
        super().__init__('trolley_mover')
        self.action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/forward_position_controller/follow_joint_trajectory'
        )
    
    def move_trolley_90_degrees(self):
        # 90 degrees motor rotation = 0.09 meters
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ['hook_joint', 'trolley_joint', 'slewing_joint']
        
        point = JointTrajectoryPoint()
        # Keep other joints at current position, move trolley 0.09m
        # You'll need to get current positions first
        point.positions = [0.0, 0.09, 0.0]  # [hook, trolley, slewing]
        point.time_from_start.sec = 5  # 5 seconds to complete
        
        goal_msg.trajectory.points = [point]
        
        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')

if __name__ == '__main__':
    rclpy.init()
    mover = TrolleyMover()
    mover.move_trolley_90_degrees()
    rclpy.spin(mover)
    rclpy.shutdown()
```

### Method 4: Check Current Position First

Before moving, always check the current position:

```bash
# Check current trolley position
ros2 topic echo /trolley_joint/joint_states --once

# Or check all joints
ros2 topic echo /joint_states --once | grep trolley_joint
```

## Quick Reference: Conversion Table

| Motor Rotation (degrees) | Linear Movement (meters) |
|-------------------------|-------------------------|
| 90° | 0.09 m |
| 180° | 0.18 m |
| 360° | 0.36 m |
| 1000° | 1.0 m |

## Important Notes

1. **Joint Limits:** The trolley can only move between **-2.0 m and 0.5 m**
   - If you try to move beyond these limits, the command will be rejected or clamped

2. **Current Position:** Always check current position before moving:
   ```bash
   ros2 topic echo /joint_states --once
   ```

3. **Conversion Factor:** The `METERS_PER_DEGREE_TROLLEY = 0.001` is defined in `crane_master/src/moveit_bridge.cpp`
   - If your actual hardware has a different ratio, you need to update this constant

4. **Safety:** Make sure the trolley path is clear before moving!

## Example: Move Trolley 90 Degrees from Current Position

```bash
# Method 1: Direct service call (90 degrees motor rotation)
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"

# Method 2: Using trajectory (0.09 meters = 90 degrees)
# First get current position
CURRENT=$(ros2 topic echo /trolley_joint/joint_states --once | grep position | head -1 | awk '{print $2}')

# Then move relative (current + 0.09m)
ros2 topic pub --once /forward_position_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
{
  joint_names: ['slewing_joint', 'trolley_joint', 'hook_joint'],
  points: [
    {
      positions: [0.0, $(echo "$CURRENT + 0.09" | bc), 0.0],
      time_from_start: {sec: 5, nanosec: 0}
    }
  ]
}"
```

## Troubleshooting

- **No movement:** Check if controllers are running: `ros2 control list_controllers`
- **Wrong direction:** Check the sign of your command (positive/negative)
- **Out of range:** Verify your target is within -2.0 to 0.5 meters
- **Motor not responding:** Check CANopen communication (see CANOPEN_TROUBLESHOOTING.md)

