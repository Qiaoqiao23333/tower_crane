# Simple MoveIt Setup - Hardware + MoveIt Separate Launch

This guide explains how to launch the crane system with hardware and MoveIt as separate components.

## Architecture Overview

```
Terminal 1: Hardware Layer (CANopen + Bridge)
  ├── canopen_ros2.launch.py → Controls real motors via CAN
  └── moveit_bridge → Translates between MoveIt and CANopen
      ├── Subscribes to: MoveIt trajectory commands
      ├── Publishes to: CANopen motor commands
      └── Publishes: /robot_description, /joint_states, /tf

Terminal 2: MoveIt Layer
  └── moveit_only.launch.py → Planning + Visualization
      ├── Subscribes to: /robot_description, /joint_states
      └── Provides: Motion planning, RViz interface
```

## Step-by-Step Launch Procedure

### Terminal 1: Launch Hardware

```bash
# Launch CANopen communication with real hardware
ros2 launch crane_master canopen_ros2.launch.py

# In another terminal, launch the MoveIt bridge
ros2 run crane_master moveit_bridge
```

**What this does:**
- Establishes CAN communication with motors
- Publishes `/robot_description` (URDF model)
- Publishes `/joint_states` (current joint angles from encoders)
- Publishes `/tf` and `/tf_static` (coordinate transforms)
- Listens for trajectory commands from MoveIt
- Sends position commands to motors via CANopen

### Terminal 2: Launch MoveIt + RViz

```bash
# Launch MoveIt motion planning and RViz visualization
ros2 launch tower_crane_moveit_config moveit_only.launch.py
```

**What this does:**
- Subscribes to `/robot_description` and `/joint_states` from crane_master
- Starts MoveIt move_group node (motion planning)
- Opens RViz with MoveIt Motion Planning panel

### Without RViz (headless)

```bash
ros2 launch tower_crane_moveit_config moveit_only.launch.py use_rviz:=false
```

## Expected Behavior

✅ **Two-Way Communication:**

1. **MoveIt → Robot:**
   - Plan a motion in RViz Motion Planning panel
   - Click "Execute"
   - MoveIt publishes trajectory → moveit_bridge receives it → motors move

2. **Robot → RViz:**
   - Move motors manually or via other commands
   - Encoders report position → moveit_bridge publishes /joint_states
   - RViz model updates to match real robot position

## Verification Commands

```bash
# Check that robot_description is being published
ros2 topic echo /robot_description --once

# Check that joint states are being published
ros2 topic echo /joint_states

# Check that MoveIt planning service is available
ros2 service list | grep plan

# Check all active nodes
ros2 node list
```

## Troubleshooting

### RViz doesn't show the robot model
- Check if `/robot_description` topic exists: `ros2 topic list | grep robot_description`
- Verify moveit_bridge is running: `ros2 node list | grep moveit_bridge`

### Robot doesn't move when executing plans
- Check if moveit_bridge is subscribed to trajectory topics
- Verify CANopen motors are enabled and ready
- Check motor status: `ros2 topic echo /diagnostics`

### RViz model doesn't update with real robot motion
- Verify `/joint_states` is being published: `ros2 topic echo /joint_states`
- Check that joint names in /joint_states match URDF joint names

## Comparison with Other Launch Files

| Launch File | Robot State Publisher | Hardware | MoveIt | RViz | Use Case |
|-------------|----------------------|----------|---------|------|----------|
| `demo.launch.py` | ✅ | ✅ Mock | ✅ | ✅ | Full simulation demo |
| `bringup.launch.py` | ✅ | ✅ Real | ✅ | ✅ | All-in-one real hardware |
| `moveit_only.launch.py` | ❌ | ❌ | ✅ | ✅ | **MoveIt only (you launch hardware separately)** |

**Use `moveit_only.launch.py` when:**
- You want to manage hardware (crane_master) separately
- You need to restart MoveIt without restarting hardware
- You're developing/debugging and want modular control


