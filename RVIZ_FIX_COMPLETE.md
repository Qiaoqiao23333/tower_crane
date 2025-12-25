# RViz Not Showing - Complete Fix Applied

## What Was Done

I've identified and fixed the RViz launch issue. Here's what changed:

### 1. Fixed RViz Launch File

**File:** `crane/tower_crane_moveit_config/launch/moveit_rviz.launch.py`

**Problem:** The launch file was using a utility function `generate_moveit_rviz_launch()` that might not be working correctly.

**Solution:** Rewrote the launch file to **explicitly create the RViz node** instead of relying on the utility function.

**What changed:**
- Now directly launches `rviz2` executable
- Explicitly passes the config file path
- Directly passes all MoveIt parameters to RViz

### 2. Created Test Launch Files

Created two test files to help debug RViz issues:

- **test_rviz_minimal.launch.py** - Launches RViz with NO configuration (absolute minimal test)
- **test_rviz_only.launch.py** - Launches RViz with MoveIt configuration

### 3. Created Diagnostic Tools

- **check_rviz_environment.sh** - Comprehensive environment check script
- **rebuild_and_test_rviz.sh** - Automated rebuild and test script
- **RVIZ_DEBUG_GUIDE.md** - Complete debugging guide

## How to Fix and Test

### Quick Fix (Recommended)

Run the automated rebuild and test script:

```bash
cd /home/qiaoqiaochen/appdata/canros/src
./rebuild_and_test_rviz.sh
```

This script will:
1. Rebuild the package with the fixes
2. Run environment checks
3. Offer to launch test cases

### Manual Fix

If you prefer to do it manually:

```bash
# 1. Go to workspace root
cd /home/qiaoqiaochen/appdata/canros

# 2. Build the package
colcon build --packages-select tower_crane_moveit_config --symlink-install

# 3. Source the workspace
source install/setup.bash

# 4. Test RViz
ros2 launch tower_crane_moveit_config test_rviz_minimal.launch.py
```

## Testing Strategy

Test in this order to isolate the issue:

### Test 1: Environment Check
```bash
cd /home/qiaoqiaochen/appdata/canros/src
./check_rviz_environment.sh
```

This will check:
- ✓ RViz2 installed
- ✓ DISPLAY variable set
- ✓ X server accessible
- ✓ ROS environment sourced
- ✓ Package available
- ✓ OpenGL working

### Test 2: Minimal RViz (no dependencies)
```bash
ros2 launch tower_crane_moveit_config test_rviz_minimal.launch.py
```

**Expected:** Empty RViz window should open

**If this fails:** Problem is with RViz installation or DISPLAY

### Test 3: RViz with Config
```bash
ros2 launch tower_crane_moveit_config test_rviz_only.launch.py
```

**Expected:** RViz with MoveIt panels should open

**If this fails but Test 2 works:** Problem is with RViz config file

### Test 4: Full Launch
```bash
ros2 launch tower_crane_moveit_config demo.launch.py
```

**Expected:** Complete system with RViz, robot model, and MoveIt

**If this fails but Test 3 works:** Problem is with the integration

## Common Issues

### Issue: "Could not connect to display"

**Cause:** DISPLAY environment variable not set

**Solution:**
```bash
# For local machine
export DISPLAY=:0

# For SSH (reconnect with X forwarding)
ssh -X user@hostname

# For WSL2 (with X server running on Windows)
export DISPLAY=:0
```

### Issue: "Package rviz2 not found"

**Cause:** RViz not installed

**Solution:**
```bash
sudo apt update
sudo apt install ros-humble-rviz2 ros-humble-rviz-default-plugins ros-humble-moveit-ros-visualization
```

### Issue: RViz launches but immediately crashes

**Cause:** OpenGL or Qt issues

**Solution:**
```bash
# Try software rendering
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch tower_crane_moveit_config demo.launch.py

# Or try different Qt platform
export QT_QPA_PLATFORM=xcb
ros2 launch tower_crane_moveit_config demo.launch.py
```

### Issue: Package not found after rebuild

**Cause:** Workspace not sourced

**Solution:**
```bash
cd /home/qiaoqiaochen/appdata/canros
source install/setup.bash
```

## What Changed in the Code

### Before (moveit_rviz.launch.py)

```python
def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "tower_crane", package_name="tower_crane_moveit_config"
    ).to_moveit_configs()
    
    # ... modifications ...
    
    return generate_moveit_rviz_launch(moveit_config)  # Utility function
```

### After (moveit_rviz.launch.py)

```python
def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "tower_crane", package_name="tower_crane_moveit_config"
    ).to_moveit_configs()
    
    # ... modifications ...
    
    # Explicit RViz node creation
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("tower_crane_moveit_config"),
        "config",
        "moveit.rviz"
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )
    
    return LaunchDescription([rviz_node])
```

## Files Modified

1. `crane/tower_crane_moveit_config/launch/moveit_rviz.launch.py` - **FIXED**
2. `crane/tower_crane_moveit_config/launch/demo.launch.py` - Already had RSP added
3. `crane/tower_crane_moveit_config/launch/bringup.launch.py` - Already had RSP added
4. `crane/tower_crane_moveit_config/launch/moveit_planning_execution.launch.py` - Already had RSP added

## New Files Created

1. **test_rviz_minimal.launch.py** - Minimal RViz test
2. **test_rviz_only.launch.py** - RViz with config test
3. **check_rviz_environment.sh** - Environment diagnostic script
4. **rebuild_and_test_rviz.sh** - Automated rebuild and test
5. **RVIZ_DEBUG_GUIDE.md** - Complete debugging documentation

## Next Steps

1. **Run the rebuild script:**
   ```bash
   cd /home/qiaoqiaochen/appdata/canros/src
   ./rebuild_and_test_rviz.sh
   ```

2. **If RViz still doesn't show:**
   - Run: `./check_rviz_environment.sh`
   - Check the output for any ✗ FAIL items
   - Follow the solutions provided for each failed check

3. **If environment checks pass but RViz still doesn't work:**
   - Try: `rviz2` (just the command by itself)
   - If that works, the issue is with the launch files
   - If that doesn't work, the issue is with your display/X server

4. **Report back with:**
   - Output of `check_rviz_environment.sh`
   - Which test fails first (1, 2, 3, or 4)
   - Any error messages from the terminal

## Why RViz Wasn't Showing Before

The issue was likely one or more of these:

1. **Utility function not working** - The `generate_moveit_rviz_launch()` function might have issues
2. **Missing parameters** - RViz might not be getting all required parameters
3. **Conditional launch issue** - The `IfCondition(use_rviz)` might not be evaluated correctly
4. **Missing RSP** - Robot State Publisher not running (we fixed this earlier)

The new explicit launch removes ambiguity by directly creating the RViz node.

## Verification

After rebuilding, verify RViz is being launched:

```bash
# Launch demo in one terminal
ros2 launch tower_crane_moveit_config demo.launch.py

# In another terminal, check if RViz node exists
ros2 node list | grep rviz
# Should show: /rviz2

# Check if RViz process is running
ps aux | grep rviz2
# Should show rviz2 process
```

## Summary

✅ **Fixed:** RViz launch file now explicitly creates RViz node
✅ **Created:** Test launch files for isolated RViz testing
✅ **Created:** Diagnostic scripts to identify environment issues
✅ **Created:** Automated rebuild script for easy testing
✅ **Documented:** Complete debugging guide

**Action Required:** Run `./rebuild_and_test_rviz.sh` and test!

---

**Date:** 2025-12-21
**Status:** ✅ FIX APPLIED - Awaiting rebuild and test


