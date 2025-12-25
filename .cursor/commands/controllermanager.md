# Why Motor Doesn't Move: Diagnosis & Solution

## Problem Summary
- `ros2 topic pub` to `/forward_position_controller/joint_trajectory` doesn't work (wrong interface - need action)
- `ros2 service call /trolley_joint/target` returns success but motor doesn't move

## Root Cause Analysis

### Issue 1: Wrong Controller Interface
The `JointTrajectoryController` **does NOT accept commands via topic**. It only accepts commands through the **action interface**.

**❌ Wrong (what you tried):**
```bash
ros2 topic pub --once /forward_position_controller/joint_trajectory ...
```

**✅ Correct:**
```bash
ros2 action send_goal /forward_position_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "..."
```

### Issue 2: Motor Not Enabled or Wrong Mode
Even though `bus.yml` (lines 75-80) configures the trolley to:
- Set position mode (line 77: `- {index: 0x6060, sub_index: 0, value: 1}`)
- Enable the motor (lines 78-80: control words 6→7→15)

The motor still needs to be **explicitly enabled** through the service interface.

## Complete Solution

### Option A: Use Action Interface (Proper Way)

```bash
# Send goal to the trajectory controller
ros2 action send_goal /forward_position_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [slewing_joint, trolley_joint, hook_joint]
  points:
  - positions: [0.0, 0.09, 0.0]
    velocities: []
    accelerations: []
    time_from_start:
      sec: 5
      nanosec: 0
" --feedback
```

### Option B: Direct CANopen Control (Testing/Debugging)

**Step 1: Initialize the motor**
```bash
ros2 service call /trolley_joint/init std_srvs/srv/Trigger
```

**Step 2: Enable the motor**
```bash
ros2 service call /trolley_joint/enable std_srvs/srv/Trigger
```

**Step 3: Send target position**
```bash
# Move 90 degrees (motor rotation, not trolley linear position)
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"
```

**Step 4: Monitor movement**
```bash
ros2 topic echo /trolley_joint/joint_states
```

### Option C: Check Motor Status First

Before attempting to move, diagnose the current state:

```bash
# 1. Check statusword (should be 0x0027 or 0x001F for enabled)
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}"

# 2. Check operation mode (should be 1 for position mode)
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6061, subindex: 0}"

# 3. Check current position
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6064, subindex: 0}"

# 4. Check for errors
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 1001, subindex: 0}"
```

## Understanding the Motor Control Architecture

```
┌─────────────────────────────────────────────┐
│  High Level: MoveIt / Trajectory Commands  │
│  (uses action interface)                    │
└───────────────┬─────────────────────────────┘
                │
                ↓
┌─────────────────────────────────────────────┐
│  forward_position_controller                │
│  (JointTrajectoryController)                │
│  - Listens: /follow_joint_trajectory (action)│
│  - NOT topic: /joint_trajectory             │
└───────────────┬─────────────────────────────┘
                │
                ↓
┌─────────────────────────────────────────────┐
│  ros2_canopen Hardware Interface            │
│  (Cia402Driver)                             │
└───────────────┬─────────────────────────────┘
                │
                ↓
┌─────────────────────────────────────────────┐
│  CANopen Services (Direct Motor Control)    │
│  - /trolley_joint/target                    │
│  - /trolley_joint/enable                    │
│  - /trolley_joint/init                      │
│  - /trolley_joint/position_mode             │
└─────────────────────────────────────────────┘
```

## Key Findings from Your System

**From bus.yml configuration:**
- Node ID: 2
- Driver: Cia402Driver (CiA 402 motor drive profile)
- Position Mode: 1 (Profile Position)
- Boot sequence configured to enable motor automatically
- TPDO1 maps: Statusword, Mode display, Position, Velocity
- RPDO1 maps: Controlword, Mode, Target position

**However:** Boot configuration may fail silently if:
1. CAN communication isn't stable
2. Motor is in fault state
3. Hardware not ready during boot

## Recommended Workflow

For testing a single motor:
```bash
# 1. Enable
ros2 service call /trolley_joint/enable std_srvs/srv/Trigger

# 2. Move
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"

# 3. Monitor
ros2 topic echo /trolley_joint/joint_states
```

For coordinated multi-joint motion with MoveIt:
```bash
# Use the action interface
ros2 action send_goal /forward_position_controller/follow_joint_trajectory ...
```

## Debugging Tools

Check controller status:
```bash
ros2 control list_controllers
```

List available actions:
```bash
ros2 action list
ros2 action info /forward_position_controller/follow_joint_trajectory
```

Monitor joint states:
```bash
ros2 topic echo /joint_states
```

## Common Errors

| Error | Statusword | Cause | Solution |
|-------|-----------|-------|----------|
| Operation Inhibit | 0x0040 | Motor locked | Call `/enable` service |
| Fault | 0x0008 | Hardware fault | Call `/recover` service |
| Wrong Mode | Mode ≠ 1 | Not in position mode | Call `/position_mode` service |
| No Response | N/A | Not initialized | Call `/init` service |

## References
- CANopen CiA 402 specification (motor drive profile)
- bus.yml: Lines 63-99 (trolley configuration)
- ROS2 Control: JointTrajectoryController documentation


