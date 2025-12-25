# Tower Crane MoveIt Launch Architecture

## Overview

This document explains how the launch files are organized and what each component does.

## Launch File Hierarchy

### demo.launch.py (Recommended Entry Point)

```
demo.launch.py
│
├── Arguments:
│   ├── can_interface_name (default: vcan0)
│   ├── use_real_hardware (default: false)
│   └── use_rviz (default: true)
│
├── [1] rsp.launch.py
│   └── Starts: robot_state_publisher
│       ├── Loads: tower_crane.urdf.xacro
│       ├── Publishes: /robot_description
│       └── Publishes: TF tree (world → base → joints)
│
└── [2] moveit_planning_execution.launch.py
    │
    ├── (Same RSP - redundant but harmless)
    │
    ├── [IF use_real_hardware=false] robot_control.launch.py (tower_crane)
    │   ├── Starts: ros2_control with mock_components
    │   ├── Spawns: joint_state_broadcaster
    │   ├── Spawns: tower_crane_controller
    │   └── Optional: crane_master nodes (CANopen mock)
    │
    ├── [IF use_real_hardware=true] hardware_bringup_real.launch.py
    │   ├── Starts: ros2_control with CanopenSystem
    │   └── Connects to: Real CANopen hardware
    │
    ├── [3] static_virtual_joint_tfs.launch.py
    │   └── Publishes: Static TF for virtual joints
    │
    ├── [4] move_group.launch.py
    │   └── Starts: MoveIt move_group node
    │       ├── Loads: SRDF (robot semantics)
    │       ├── Loads: Planning pipeline
    │       ├── Loads: Kinematics solvers
    │       └── Provides: Motion planning services
    │
    └── [5] moveit_rviz.launch.py (if use_rviz=true)
        └── Starts: RViz2
            ├── Loads: moveit.rviz config
            ├── Shows: Robot model
            ├── Shows: Planning scene
            └── Provides: Interactive motion planning
```

### bringup.launch.py (Alternative Entry Point)

```
bringup.launch.py
│
├── Arguments:
│   ├── can_interface_name (default: vcan0)
│   ├── use_crane_master (default: true)
│   ├── auto_start (default: true)
│   └── use_rviz (default: true)
│
├── [1] rsp.launch.py
│   └── (Same as above)
│
├── [2] robot_control.launch.py (from tower_crane package)
│   ├── ros2_control with mock or fake slaves
│   └── Optional crane_master nodes
│
├── [3] static_virtual_joint_tfs.launch.py
│   └── (Same as above)
│
├── [4] move_group.launch.py
│   └── (Same as above)
│
└── [5] moveit_rviz.launch.py
    └── (Same as above)
```

## Key Components Explained

### 1. Robot State Publisher (RSP)

**Purpose:** The foundation - loads and publishes the robot model

**What it does:**
- Parses the URDF/xacro file (`tower_crane.urdf.xacro`)
- Publishes `/robot_description` parameter
- Subscribes to `/joint_states`
- Publishes TF transforms for all robot links

**Why it's critical:** Without RSP:
- No robot model in RViz ❌
- No TF tree ❌
- MoveIt can't plan ❌

### 2. Hardware/Control Layer

**Purpose:** Controls the robot (real or simulated)

**Mock Mode (use_real_hardware=false):**
- Uses `mock_components/GenericSystem`
- Simulates joint positions
- No real hardware needed

**Real Mode (use_real_hardware=true):**
- Uses `canopen_ros2_control/CanopenSystem`
- Connects via CAN bus (can0)
- Controls real motors via CANopen protocol

### 3. Static Virtual Joint TFs

**Purpose:** Publishes fixed transforms for virtual joints

**What it does:**
- Publishes `world → base_link` transform
- Anchors the robot in space
- Required for MoveIt planning

### 4. MoveIt Move Group

**Purpose:** The brain - handles motion planning

**What it does:**
- Loads robot kinematics
- Loads motion planning algorithms (OMPL, Pilz, etc.)
- Collision checking
- Trajectory execution
- Provides services:
  - `/plan_kinematic_path`
  - `/execute_trajectory`
  - `/compute_cartesian_path`

**Required inputs:**
- `/robot_description` (from RSP)
- `/joint_states` (from hardware/control)
- TF tree (from RSP + virtual joints)

### 5. RViz

**Purpose:** Visualization and interactive planning

**What it shows:**
- Robot 3D model
- Planning scene (obstacles, collision objects)
- Planned trajectories
- Interactive markers for goal poses

**Plugins loaded:**
- `moveit_rviz_plugin/MotionPlanning`
- `rviz_default_plugins/RobotModel`
- `rviz_default_plugins/TF`

## Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│                        User Input (RViz)                     │
└─────────────────────┬───────────────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────────────┐
│                   MoveIt Move Group                          │
│  - Planning algorithms (OMPL, Pilz)                          │
│  - Kinematics (KDL, TracIK)                                  │
│  - Collision checking                                        │
└─────────────────┬───────────────────┬───────────────────────┘
                  │                   │
                  │ Plans             │ Commands
                  ▼                   ▼
┌──────────────────────────┐  ┌──────────────────────────────┐
│  Robot State Publisher   │  │   ros2_control               │
│  - Publishes TF          │  │   - Joint controllers        │
│  - robot_description     │  │   - Hardware interface       │
└──────────────────────────┘  └───────────┬──────────────────┘
                                           │
                                           ▼
                              ┌────────────────────────────────┐
                              │  Hardware / Simulation         │
                              │  - Mock (GenericSystem)        │
                              │  - Real (CanopenSystem → CAN)  │
                              └────────────────────────────────┘
```

## Topics Published

| Topic | Publisher | Subscribers |
|-------|-----------|-------------|
| `/robot_description` | robot_state_publisher | move_group, rviz |
| `/joint_states` | ros2_control | robot_state_publisher, move_group |
| `/tf` | robot_state_publisher, static_tfs | move_group, rviz |
| `/tower_crane_controller/commands` | move_group | ros2_control |
| `/planning_scene` | move_group | rviz |
| `/display_planned_path` | move_group | rviz |

## Services Provided

| Service | Provider | Purpose |
|---------|----------|---------|
| `/plan_kinematic_path` | move_group | Plan motion to goal |
| `/execute_trajectory` | move_group | Execute planned path |
| `/compute_cartesian_path` | move_group | Cartesian path planning |
| `/get_planning_scene` | move_group | Get current scene |
| `/apply_planning_scene` | move_group | Update scene |

## Configuration Files Used

| File | Purpose |
|------|---------|
| `tower_crane.urdf.xacro` | Robot geometry and structure |
| `tower_crane.srdf` | Semantic robot description (MoveIt) |
| `joint_limits.yaml` | Joint velocity/acceleration limits |
| `kinematics.yaml` | Kinematics solver configuration |
| `moveit_controllers.yaml` | Controller mappings |
| `ros2_controllers.yaml` | ros2_control configuration |
| `moveit.rviz` | RViz display configuration |

## Startup Sequence

1. **RSP starts** → Loads URDF, publishes `/robot_description` and TF
2. **Hardware/Control starts** → Connects to motors, publishes `/joint_states`
3. **Static TFs start** → Publishes `world → base_link`
4. **Move Group starts** → Waits for `/robot_description`, `/joint_states`, TF
5. **RViz starts** → Connects to move_group, displays robot

## Common Issues & Solutions

### Issue: "Robot model not loading"
**Cause:** RSP not running or URDF file not found
**Solution:** Verify RSP is in launch file (✅ now fixed!)

### Issue: "TF timeout errors"
**Cause:** Missing static transforms or RSP not publishing
**Solution:** Check that static_virtual_joint_tfs.launch.py is included

### Issue: "move_group complains about missing robot_description"
**Cause:** RSP not started yet or failed to load URDF
**Solution:** Check RSP node logs, verify URDF file exists

### Issue: "Can't plan in RViz"
**Cause:** move_group not receiving joint states
**Solution:** Check ros2_control and controllers are loaded

## The Fix Applied

**Before:** Launch files were missing Robot State Publisher (RSP)
- ❌ No robot model loaded
- ❌ No TF tree published
- ❌ RViz couldn't display robot
- ❌ MoveIt couldn't plan

**After:** Added RSP to all main launch files
- ✅ Robot model loads correctly
- ✅ TF tree published
- ✅ RViz displays robot
- ✅ MoveIt can plan motions

## Summary

The key insight is that **Robot State Publisher is the foundation** of the entire system. It must start first and successfully load the URDF before any other component can work properly.

The launch files now follow the proper startup sequence:
1. RSP (loads model)
2. Control (moves joints)
3. MoveIt (plans motion)
4. RViz (visualizes)

All launch files have been fixed to include RSP at the beginning of their startup sequence.


