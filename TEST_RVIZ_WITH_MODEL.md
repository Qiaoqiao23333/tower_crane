# Test RViz with Robot Model - Updated

## What Changed

Both test files now include **Robot State Publisher (RSP)** which loads the robot model!

### test_rviz_minimal.launch.py
- ✅ Launches Robot State Publisher (loads URDF)
- ✅ Launches RViz with default view
- ✅ Robot model should be visible

### test_rviz_only.launch.py  
- ✅ Launches Robot State Publisher (loads URDF)
- ✅ Launches RViz with MoveIt configuration
- ✅ Robot model should be visible with MoveIt panels

## How to Test

### Step 1: Rebuild the Package

```bash
cd /home/qiaoqiaochen/appdata/canros
colcon build --packages-select tower_crane_moveit_config --symlink-install
source install/setup.bash
```

### Step 2: Test Minimal RViz with Model

```bash
ros2 launch tower_crane_moveit_config test_rviz_minimal.launch.py
```

**What you should see:**
- RViz window opens ✅
- Robot model (tower crane) appears in 3D view ✅
- You can add displays manually if needed

**To see the robot in RViz:**
1. In RViz, click "Add" button (bottom left)
2. Add "RobotModel" display
3. Set "Description Topic" to `/robot_description`
4. Set "Fixed Frame" to `world` or `base_link`

### Step 3: Test RViz with MoveIt Config

```bash
ros2 launch tower_crane_moveit_config test_rviz_only.launch.py
```

**What you should see:**
- RViz window opens with MoveIt layout ✅
- Robot model (tower crane) visible ✅
- MoveIt Motion Planning panel on the left ✅

### Step 4: Full Demo

```bash
ros2 launch tower_crane_moveit_config demo.launch.py
```

**What you should see:**
- Everything from Step 3 ✅
- Plus move_group node running ✅
- Plus ros2_control controllers ✅
- Ready for motion planning! ✅

## Verify Robot Model is Loaded

While any of the tests are running, check in another terminal:

```bash
# Check robot_description topic
ros2 topic list | grep robot_description
# Should show: /robot_description

# Check robot_state_publisher node
ros2 node list | grep robot_state
# Should show: /robot_state_publisher

# Check TF frames
ros2 run tf2_ros tf2_echo world base_link
# Should show transforms
```

## What Each Component Does

```
┌─────────────────────────────────────────┐
│  Robot State Publisher (RSP)            │
│  - Loads tower_crane.urdf.xacro         │
│  - Publishes /robot_description         │
│  - Publishes TF tree                    │
└──────────────┬──────────────────────────┘
               │
               │ Publishes
               │ robot_description
               │ + TF transforms
               ▼
┌─────────────────────────────────────────┐
│  RViz2                                  │
│  - Reads /robot_description             │
│  - Displays 3D robot model              │
│  - Shows TF frames                      │
└─────────────────────────────────────────┘
```

## In RViz - How to See the Robot

If the robot doesn't appear automatically:

### Option 1: Add RobotModel Display
1. Click "Add" button (bottom left in Displays panel)
2. Select "RobotModel" from the list
3. Click OK
4. In RobotModel settings:
   - Check "Enabled" is checked
   - "Description Topic" should be `/robot_description`
   - "TF Prefix" should be empty

### Option 2: Set Fixed Frame
1. Look at top of Displays panel
2. Find "Fixed Frame"
3. Change it to `world` or `base_link`
4. Robot should appear

### Option 3: Reset View
1. In 3D view, middle-mouse drag to rotate
2. Scroll to zoom
3. Press "Home" or reset view to see the whole robot

## Troubleshooting

### Robot model not visible in RViz

**Check 1: Is robot_description published?**
```bash
ros2 topic echo /robot_description --once | head -20
```
Should show URDF XML content.

**Check 2: Is RobotModel display enabled?**
- Look in Displays panel
- Find "RobotModel" 
- Make sure checkbox is checked
- Check for error messages in Status

**Check 3: Is Fixed Frame correct?**
- Fixed Frame should be `world` or `base_link`
- Not `map` or other frames

**Check 4: Is TF being published?**
```bash
ros2 topic hz /tf
```
Should show updates at ~10-30 Hz

### "No transform from [frame] to [frame]"

**Solution:** Check TF tree
```bash
ros2 run tf2_tools view_frames
```
This creates a PDF showing all frames.

### RViz shows but robot is invisible

**Solution:** Zoom out or reset view
- Scroll to zoom out
- Or click "Views" panel → "Reset"

### "Could not load robot model"

**Solution:** Check URDF file exists
```bash
ros2 pkg prefix tower_crane_moveit_config
ls $(ros2 pkg prefix tower_crane_moveit_config)/share/tower_crane_moveit_config/config/
```

## Quick Command Reference

```bash
# Rebuild and test
cd /home/qiaoqiaochen/appdata/canros
colcon build --packages-select tower_crane_moveit_config --symlink-install
source install/setup.bash

# Test 1: Minimal (RSP + RViz)
ros2 launch tower_crane_moveit_config test_rviz_minimal.launch.py

# Test 2: With MoveIt config (RSP + RViz with MoveIt panels)
ros2 launch tower_crane_moveit_config test_rviz_only.launch.py

# Test 3: Full demo (RSP + Hardware + MoveIt + RViz)
ros2 launch tower_crane_moveit_config demo.launch.py

# Check what's running
ros2 node list
ros2 topic list

# Check robot description
ros2 topic echo /robot_description --once | head -20
```

## Summary

✅ **test_rviz_minimal.launch.py** - Now launches RSP + RViz (robot model loads!)
✅ **test_rviz_only.launch.py** - Now launches RSP + RViz with MoveIt config (robot model loads!)
✅ **demo.launch.py** - Already had RSP, now fully working

**Next step:** Rebuild and test!
```bash
cd /home/qiaoqiaochen/appdata/canros
colcon build --packages-select tower_crane_moveit_config --symlink-install
source install/setup.bash
ros2 launch tower_crane_moveit_config test_rviz_only.launch.py
```

You should now see the tower crane robot model in RViz! 🎉


