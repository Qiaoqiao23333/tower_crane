# tower_crane_moveit_config

MoveIt configuration package for the tower crane robot. Provides motion planning, trajectory execution, and RViz visualization for the three crane axes: **slewing** (rotation), **trolley** (horizontal travel), and **hoist/hook** (vertical travel).

## Quick Start

```bash
# Build
cd ~/appdata/ws_tower_crane
colcon build --packages-select tower_crane_moveit_config --symlink-install
source install/setup.bash

# Launch (simulation + MoveIt + RViz)
ros2 launch tower_crane_moveit_config bringup.launch.py
```

## Launch Files

### `bringup.launch.py` — Main entry point

Single launch file that covers all scenarios.

| Argument | Default | Description |
|----------|---------|-------------|
| `can_interface_name` | `vcan0` | CAN interface. `vcan*` → simulation, `can*` → real hardware (auto-detected). |
| `launch_hardware` | `true` | Launch RSP + ros2_control. Set `false` when crane_master + moveit_bridge are already running. |
| `use_rviz` | `true` | Start RViz with MoveIt Motion Planning panel. |

**Examples:**

```bash
# 1. Simulation (mock hardware) — default
ros2 launch tower_crane_moveit_config bringup.launch.py

# 2. Real hardware
ros2 launch tower_crane_moveit_config bringup.launch.py can_interface_name:=can0

# 3. MoveIt-only (crane_master + moveit_bridge already running)
ros2 launch tower_crane_moveit_config bringup.launch.py launch_hardware:=false

# 4. Headless (no RViz)
ros2 launch tower_crane_moveit_config bringup.launch.py use_rviz:=false
```

### `setup_assistant.launch.py`

Launches the MoveIt Setup Assistant for editing this configuration package.

```bash
ros2 launch tower_crane_moveit_config setup_assistant.launch.py
```

## Joints

| Joint | Type | Motor (crane_master) | MoveIt Unit | Motor Unit |
|-------|------|----------------------|-------------|------------|
| `slewing_joint` | Revolute | `/slewing` (node_id 3) | radians | degrees |
| `trolley_joint` | Prismatic | `/trolley` (node_id 2) | meters | degrees |
| `hook_joint` | Prismatic | `/hoist` (node_id 1) | meters | degrees |

### Unit Conversion (moveit_bridge)

The `moveit_bridge` node (in the `crane_master` package) translates between MoveIt units and motor units:

- **slewing_joint**: radians ↔ degrees (`rad2deg` / `deg2rad`)
- **trolley_joint**: meters ↔ degrees (factor: `METERS_PER_DEGREE_TROLLEY = 0.001`)
- **hook_joint**: meters ↔ degrees (factor: `METERS_PER_DEGREE_HOIST = 0.001`)

> **Note:** The meter-per-degree conversion factors are placeholders. Adjust them in `crane_master/src/moveit_bridge.cpp` to match your actual winch/lead-screw parameters.

## Architecture

```
bringup.launch.py
│
├── [1] Robot State Publisher    (publishes /robot_description + TF)
├── [2] Hardware Bringup         (simulation.launch.py OR hardware_bringup_real.launch.py)
├── [3] Static Virtual Joint TFs (world → base_link)
├── [4] MoveIt move_group        (motion planning + trajectory execution)
└── [5] RViz                     (visualization + interactive planning)
```

When using real hardware with `crane_master` separately:

```
Terminal 1:  ros2 launch crane_master canopen_ros2.launch.py can_interface:=can0 node_id:=all
Terminal 2:  ros2 run crane_master moveit_bridge
Terminal 3:  ros2 launch tower_crane_moveit_config bringup.launch.py launch_hardware:=false
```

## Configuration Files

| File | Description |
|------|-------------|
| `config/tower_crane.urdf.xacro` | Robot description (URDF) |
| `config/tower_crane.srdf` | Semantic robot description (planning groups, virtual joints) |
| `config/ros2_controllers.yaml` | ros2_control controller configuration (`forward_position_controller`) |
| `config/moveit_controllers.yaml` | MoveIt controller manager configuration |
| `config/kinematics.yaml` | Kinematics solver settings (KDL) |
| `config/joint_limits.yaml` | Velocity/acceleration limits and scaling factors |
| `config/initial_positions.yaml` | Default joint positions for mock hardware |
| `config/pilz_cartesian_limits.yaml` | Cartesian limits for Pilz planner |
| `config/moveit.rviz` | RViz layout with MoveIt panels |

## Controller

The package uses a single `JointTrajectoryController` named `forward_position_controller` that commands all three joints in position mode.

- Action: `/forward_position_controller/follow_joint_trajectory`
- Type: `control_msgs/action/FollowJointTrajectory`

## Verification

While the system is running, verify in another terminal:

```bash
# Check nodes are running
ros2 node list | grep -E "robot_state|move_group|rviz"

# Check TF tree
ros2 run tf2_ros tf2_echo world base_link

# Check joint states are being published
ros2 topic echo /joint_states --once
```

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Robot model not visible in RViz | Set Fixed Frame to `world`; check `/robot_description` is published |
| MoveIt planning fails | Verify `move_group` node is running; check `/joint_states` topic |
| RViz doesn't open | Check `$DISPLAY` is set (`echo $DISPLAY`) |
| Motor not moving with MoveIt plan | Ensure `moveit_bridge` is running and subscribed to motor feedback |

## License

BSD
