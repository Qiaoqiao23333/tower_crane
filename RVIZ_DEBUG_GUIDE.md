# RViz Not Showing Up - Debug Guide

## Problem

RViz2 graphical interface is not showing up when launching the tower crane MoveIt config.

## Step-by-Step Debugging

Follow these steps in order to identify the issue:

### Step 1: Check if RViz2 is installed

```bash
which rviz2
```

**Expected output:** Path to rviz2 executable (e.g., `/opt/ros/humble/bin/rviz2`)

**If not found:** Install RViz2
```bash
sudo apt update
sudo apt install ros-humble-rviz2 ros-humble-rviz-default-plugins
```

### Step 2: Test RViz2 standalone (minimal)

```bash
ros2 launch tower_crane_moveit_config test_rviz_minimal.launch.py
```

**Expected:** Empty RViz window should open

**If this fails:**
- Check DISPLAY environment variable: `echo $DISPLAY`
- If using SSH: Make sure X forwarding is enabled (`ssh -X`)
- If using WSL: Install X server (VcXsrv, Xming, or WSLg)
- Check for errors in terminal output

### Step 3: Test RViz2 with config file

```bash
ros2 launch tower_crane_moveit_config test_rviz_only.launch.py
```

**Expected:** RViz window should open with MoveIt config

**If this fails but Step 2 worked:**
- Config file might be corrupted
- Check terminal for specific error messages

### Step 4: Check full launch with RViz

```bash
# Make sure the package is built
cd /home/qiaoqiaochen/appdata/canros
colcon build --packages-select tower_crane_moveit_config

# Source the workspace
source install/setup.bash

# Launch demo
ros2 launch tower_crane_moveit_config demo.launch.py use_rviz:=true
```

**Check terminal output for:**
- "Launching RViz" or similar message
- Any error messages mentioning rviz
- "Failed to load" messages

### Step 5: Check if RViz process is running

In another terminal while demo is running:

```bash
ps aux | grep rviz2
```

**Expected:** Should show rviz2 process

**If process exists but no window:**
- DISPLAY issue
- Window manager issue
- RViz crashed silently

### Step 6: Check ROS nodes

```bash
ros2 node list | grep rviz
```

**Expected:** Should show `/rviz2` node

**If node doesn't exist:** RViz didn't start at all

### Step 7: Launch with verbose output

```bash
ros2 launch tower_crane_moveit_config demo.launch.py use_rviz:=true --debug
```

Look for error messages in the output.

## Common Issues and Solutions

### Issue 1: DISPLAY not set

**Symptoms:**
- Error: "Could not connect to display"
- Error: "Cannot open display"

**Solution for local machine:**
```bash
export DISPLAY=:0
ros2 launch tower_crane_moveit_config demo.launch.py
```

**Solution for SSH:**
```bash
# Connect with X forwarding
ssh -X user@hostname

# Check DISPLAY is set
echo $DISPLAY  # Should show something like "localhost:10.0"

# Test X forwarding works
xeyes  # Should show a window with eyes

# If xeyes works, RViz should work too
```

**Solution for WSL2:**
1. Install VcXsrv or another X server on Windows
2. Start X server (disable access control)
3. In WSL terminal:
```bash
export DISPLAY=$(cat /etc/resolv.conf | grep nameserver | awk '{print $2}'):0
# Or for WSLg (Windows 11):
export DISPLAY=:0
```

### Issue 2: Missing RViz packages

**Symptoms:**
- "Package 'rviz2' not found"
- "Package 'rviz_default_plugins' not found"

**Solution:**
```bash
sudo apt update
sudo apt install \
    ros-humble-rviz2 \
    ros-humble-rviz-default-plugins \
    ros-humble-rviz-common \
    ros-humble-moveit-ros-visualization
```

### Issue 3: Qt/OpenGL issues

**Symptoms:**
- "Could not initialize OpenGL"
- "Qt platform plugin" errors

**Solution:**
```bash
# For software rendering (slower but more compatible)
export LIBGL_ALWAYS_SOFTWARE=1

# Or try different Qt platform
export QT_QPA_PLATFORM=xcb

# Then launch again
ros2 launch tower_crane_moveit_config demo.launch.py
```

### Issue 4: RViz starts but crashes immediately

**Symptoms:**
- RViz node appears briefly then disappears
- "Segmentation fault" in logs

**Solution:**
```bash
# Check RViz logs
ros2 run rviz2 rviz2 --log-level debug

# Try with clean config
ros2 run rviz2 rviz2
```

### Issue 5: use_rviz argument not working

**Symptoms:**
- Launched with `use_rviz:=true` but RViz doesn't start
- No error messages

**Verification:**
```bash
# Check if the argument is being parsed correctly
ros2 launch tower_crane_moveit_config demo.launch.py use_rviz:=true --show-args
```

**Solution:**
The launch file should have been fixed. If still not working, try directly:
```bash
ros2 launch tower_crane_moveit_config moveit_rviz.launch.py
```

## Quick Test Commands

### Test 1: Minimal RViz (no ROS dependencies)
```bash
rviz2
```

### Test 2: RViz with minimal launch
```bash
ros2 launch tower_crane_moveit_config test_rviz_minimal.launch.py
```

### Test 3: RViz with config
```bash
ros2 launch tower_crane_moveit_config test_rviz_only.launch.py
```

### Test 4: RViz only (from full config)
```bash
ros2 launch tower_crane_moveit_config moveit_rviz.launch.py
```

### Test 5: Full demo
```bash
ros2 launch tower_crane_moveit_config demo.launch.py
```

## Debugging Output

When you run a test, capture the output:

```bash
ros2 launch tower_crane_moveit_config demo.launch.py 2>&1 | tee rviz_debug.log
```

Then search for errors:
```bash
grep -i error rviz_debug.log
grep -i rviz rviz_debug.log
grep -i display rviz_debug.log
```

## Environment Check Script

Run this to check your environment:

```bash
cat << 'EOF' > /tmp/check_rviz_env.sh
#!/bin/bash
echo "=== RViz Environment Check ==="
echo ""
echo "1. RViz2 installed?"
which rviz2 && echo "✓ rviz2 found" || echo "✗ rviz2 NOT found"
echo ""
echo "2. DISPLAY set?"
echo "DISPLAY=$DISPLAY"
[ -z "$DISPLAY" ] && echo "✗ DISPLAY not set" || echo "✓ DISPLAY is set"
echo ""
echo "3. X server accessible?"
xdpyinfo >/dev/null 2>&1 && echo "✓ X server accessible" || echo "✗ X server NOT accessible"
echo ""
echo "4. ROS sourced?"
[ -z "$ROS_DISTRO" ] && echo "✗ ROS not sourced" || echo "✓ ROS $ROS_DISTRO sourced"
echo ""
echo "5. Workspace sourced?"
[ -z "$COLCON_PREFIX_PATH" ] && echo "✗ Workspace not sourced" || echo "✓ Workspace sourced"
echo ""
echo "6. Tower crane package found?"
ros2 pkg list | grep -q tower_crane_moveit_config && echo "✓ Package found" || echo "✗ Package NOT found"
echo ""
echo "7. OpenGL available?"
glxinfo >/dev/null 2>&1 && echo "✓ OpenGL working" || echo "⚠ glxinfo not available (install mesa-utils)"
echo ""
EOF
chmod +x /tmp/check_rviz_env.sh
/tmp/check_rviz_env.sh
```

## Modified Files

The following file has been updated to explicitly launch RViz:

- `crane/tower_crane_moveit_config/launch/moveit_rviz.launch.py`

The new version directly creates the RViz node instead of using a utility function.

## New Test Files Created

1. **test_rviz_minimal.launch.py** - Launches RViz with no configuration
2. **test_rviz_only.launch.py** - Launches RViz with MoveIt config

## What Changed in moveit_rviz.launch.py

**Before:**
```python
return generate_moveit_rviz_launch(moveit_config)
```

**After:**
```python
rviz_node = Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    output="log",
    arguments=["-d", rviz_config_file],
    parameters=[...],
)
return LaunchDescription([rviz_node])
```

Now it explicitly creates and launches the RViz node.

## Next Steps

1. Run the environment check script above
2. Try the test commands in order (1-5)
3. Report which test fails first
4. Check the specific error message
5. Apply the appropriate solution from "Common Issues"

## Getting Help

If none of these steps work, please provide:
1. Output of the environment check script
2. Output of `ros2 launch tower_crane_moveit_config test_rviz_minimal.launch.py`
3. Your operating system (Ubuntu version, WSL, etc.)
4. Whether you're using SSH or local display
5. Complete error log from the terminal


