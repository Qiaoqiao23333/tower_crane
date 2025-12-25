# Launch Files Fixed - Tower Crane MoveIt Config

## Summary of Changes

The launch files in `tower_crane_moveit_config` have been fixed to properly:
1. **Load the robot model (URDF)** - Added robot_state_publisher (RSP) to all main launch files
2. **Enable RViz visualization** - RViz now properly launches with the robot model visible

## What Was Wrong

The previous launch files were missing the **Robot State Publisher (RSP)** node, which is responsible for:
- Loading the robot URDF/xacro model
- Publishing the `/robot_description` parameter
- Publishing the TF tree for all robot links

Without RSP, neither the robot model nor RViz could work properly.

## Files Modified

1. **demo.launch.py** - Added RSP before moveit_planning_execution
2. **bringup.launch.py** - Added RSP before robot_control  
3. **moveit_planning_execution.launch.py** - Added RSP before hardware bringup

## How to Use the Fixed Launch Files

### Option 1: Demo Launch (Recommended for Testing)

```bash
ros2 launch tower_crane_moveit_config demo.launch.py
```

**Arguments:**
- `use_rviz:=true/false` - Enable/disable RViz (default: true)
- `use_real_hardware:=true/false` - Use real CANopen hardware or mock (default: false)
- `can_interface_name:=vcan0/can0` - CAN interface to use (default: vcan0)

**Examples:**
```bash
# Launch with RViz and mock hardware (simulation)
ros2 launch tower_crane_moveit_config demo.launch.py

# Launch without RViz
ros2 launch tower_crane_moveit_config demo.launch.py use_rviz:=false

# Launch with real hardware on can0
ros2 launch tower_crane_moveit_config demo.launch.py use_real_hardware:=true can_interface_name:=can0
```

### Option 2: Bringup Launch (Full System with Crane Master)

```bash
ros2 launch tower_crane_moveit_config bringup.launch.py
```

**Arguments:**
- `use_rviz:=true/false` - Enable/disable RViz (default: true)
- `can_interface_name:=vcan0/can0` - CAN interface (default: vcan0)
- `use_crane_master:=true/false` - Start crane_master CANopen nodes (default: true)
- `auto_start:=true/false` - Auto-start motors (default: true)

**Examples:**
```bash
# Full bringup with RViz and crane_master
ros2 launch tower_crane_moveit_config bringup.launch.py

# Bringup without crane_master (mock only)
ros2 launch tower_crane_moveit_config bringup.launch.py use_crane_master:=false

# Real hardware with can0
ros2 launch tower_crane_moveit_config bringup.launch.py can_interface_name:=can0
```

### Option 3: MoveIt Planning & Execution Launch

```bash
ros2 launch tower_crane_moveit_config moveit_planning_execution.launch.py
```

**Arguments:**
- Same as demo.launch.py

## What You Should See

When launching with RViz enabled, you should see:

1. **RViz window opens** with the MoveIt Motion Planning plugin
2. **Robot model displays** in the 3D view (tower crane with all links)
3. **Interactive markers** for planning motion
4. **Planning groups** available in the Motion Planning panel
5. **TF tree** publishing all robot frames

## Verify Everything Works

Check that the robot model is loaded:
```bash
ros2 topic echo /robot_description --once
```

Check that TF is publishing:
```bash
ros2 run tf2_tools view_frames
```

Check that MoveIt move_group is running:
```bash
ros2 node list | grep move_group
```

## Troubleshooting

### Robot model not visible in RViz
- Make sure the `robot_description` topic is being published
- Check the RobotModel display is enabled in RViz
- Verify the Fixed Frame is set to `world` or `base_link`

### RViz doesn't launch
- Check that `use_rviz:=true` is set
- Verify the moveit.rviz config file exists in the config folder
- Check terminal output for errors

### TF errors
- Ensure robot_state_publisher is running: `ros2 node list | grep robot_state`
- Verify static transforms are being published
- Check for TF frame conflicts

## Additional Launch Files

Other available launch files in `tower_crane_moveit_config/launch/`:

- **rsp.launch.py** - Only robot state publisher (now included in main launches)
- **move_group.launch.py** - Only MoveIt move_group node
- **moveit_rviz.launch.py** - Only RViz with MoveIt plugin
- **static_virtual_joint_tfs.launch.py** - Static transforms for virtual joints
- **spawn_controllers.launch.py** - Spawn ros2_control controllers
- **warehouse_db.launch.py** - MoveIt warehouse database
- **setup_assistant.launch.py** - MoveIt Setup Assistant

## Architecture

The launch files now follow this structure:

```
demo.launch.py
├── rsp.launch.py (Robot State Publisher - loads URDF)
└── moveit_planning_execution.launch.py
    ├── robot_control.launch.py (mock) OR hardware_bringup_real.launch.py (real)
    ├── static_virtual_joint_tfs.launch.py
    ├── move_group.launch.py (MoveIt move_group node)
    └── moveit_rviz.launch.py (RViz visualization)
```

```
bringup.launch.py
├── rsp.launch.py (Robot State Publisher - loads URDF)
├── robot_control.launch.py (from tower_crane package)
├── static_virtual_joint_tfs.launch.py
├── move_group.launch.py
└── moveit_rviz.launch.py
```

## Key Difference Between demo.launch.py and bringup.launch.py

- **demo.launch.py**: Unified launch that supports both mock and real hardware via `use_real_hardware` argument
- **bringup.launch.py**: Always uses `robot_control.launch.py` with optional `crane_master` nodes

Choose `demo.launch.py` for most use cases as it's more flexible.


