# Quick Launch Guide - Tower Crane MoveIt

## ✅ What Was Fixed

The launch files now properly:
- **Load the robot model** (URDF) via Robot State Publisher
- **Launch RViz** with the robot visualization
- **Publish TF tree** for all robot links

## 🚀 Quick Start

### Most Common: Launch Demo with RViz

```bash
ros2 launch tower_crane_moveit_config demo.launch.py
```

This will:
- ✅ Load the tower crane robot model
- ✅ Start MoveIt move_group
- ✅ Open RViz with motion planning interface
- ✅ Use mock hardware (simulation mode)

### Real Hardware

```bash
ros2 launch tower_crane_moveit_config demo.launch.py use_real_hardware:=true can_interface_name:=can0
```

### Without RViz (headless)

```bash
ros2 launch tower_crane_moveit_config demo.launch.py use_rviz:=false
```

## 📋 All Launch Options

| Launch File | Purpose | Use Case |
|-------------|---------|----------|
| `demo.launch.py` | **Most flexible** - supports mock & real hardware | **Recommended** for most users |
| `bringup.launch.py` | Full system with crane_master nodes | For production with CANopen |
| `moveit_planning_execution.launch.py` | MoveIt + hardware control | Alternative to demo |

## 🔧 Common Arguments

| Argument | Default | Options | Description |
|----------|---------|---------|-------------|
| `use_rviz` | `true` | `true`/`false` | Launch RViz visualization |
| `use_real_hardware` | `false` | `true`/`false` | Use real CANopen hardware |
| `can_interface_name` | `vcan0` | `vcan0`/`can0` | CAN interface |

## ✔️ Verify It Works

### Check robot model is loaded:
```bash
ros2 topic echo /robot_description --once | head -20
```

### Check nodes are running:
```bash
ros2 node list
```

You should see:
- `/move_group`
- `/robot_state_publisher`
- `/rviz2` (if RViz enabled)

### Check TF tree:
```bash
ros2 run tf2_ros tf2_echo world base_link
```

## 🎯 In RViz

Once launched, in RViz you should see:

1. **3D robot model** of the tower crane
2. **Motion Planning** panel on the left
3. **Planning Group** dropdown (select joint group)
4. **Interactive markers** to drag and plan motion
5. **Plan & Execute** buttons

## 📝 Modified Files

The following launch files were updated to include Robot State Publisher:

- `crane/tower_crane_moveit_config/launch/demo.launch.py`
- `crane/tower_crane_moveit_config/launch/bringup.launch.py`
- `crane/tower_crane_moveit_config/launch/moveit_planning_execution.launch.py`

## 🐛 Troubleshooting

### Problem: Robot model not visible in RViz

**Solution:**
1. Check RobotModel display is enabled in RViz (left panel)
2. Verify Fixed Frame is set to `world` or `base_link`
3. Check topic: `ros2 topic list | grep robot_description`

### Problem: TF errors in terminal

**Solution:**
1. Ensure robot_state_publisher is running: `ros2 node list | grep robot_state`
2. Check frames: `ros2 run tf2_tools view_frames`

### Problem: RViz doesn't launch

**Solution:**
1. Verify argument: `use_rviz:=true`
2. Check config exists: `ls ~/appdata/canros/src/crane/tower_crane_moveit_config/config/moveit.rviz`
3. Look for errors in terminal output

## 📚 More Details

For comprehensive information, see: `LAUNCH_FILES_FIXED.md`


