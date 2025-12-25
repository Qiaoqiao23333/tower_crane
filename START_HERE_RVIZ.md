# RViz Fix - START HERE

## The Problem

RViz graphical interface is not showing up when you launch the tower crane MoveIt configuration.

## The Solution

I've fixed the RViz launch file and created tools to help you test it.

## What You Need To Do NOW

### Step 1: Run the Automated Fix (Easiest)

```bash
cd /home/qiaoqiaochen/appdata/canros/src
./rebuild_and_test_rviz.sh
```

This will:
- ✅ Rebuild the package with fixes
- ✅ Check your environment
- ✅ Let you test RViz

**That's it!** The script will guide you through everything.

---

## If You Prefer Manual Steps

### Step 1: Rebuild the Package

```bash
cd /home/qiaoqiaochen/appdata/canros
colcon build --packages-select tower_crane_moveit_config --symlink-install
source install/setup.bash
```

### Step 2: Test RViz

Try these in order:

```bash
# Test 1: Minimal RViz with robot model
ros2 launch tower_crane_moveit_config test_rviz_minimal.launch.py
```

If RViz opens AND you see the tower crane robot: **✅ SUCCESS!**

```bash
# Test 2: RViz with MoveIt panels and robot model
ros2 launch tower_crane_moveit_config test_rviz_only.launch.py
```

If RViz opens with MoveIt panels and robot model: **✅ BETTER!**

```bash
# Test 3: Full demo with everything
ros2 launch tower_crane_moveit_config demo.launch.py
```

If everything works and you can plan motion: **✅ COMPLETE SUCCESS!**

---

## If RViz Still Doesn't Show

### Check Your Environment

```bash
cd /home/qiaoqiaochen/appdata/canros/src
./check_rviz_environment.sh
```

This will tell you what's wrong.

### Common Quick Fixes

**If using SSH:**
```bash
# Reconnect with X forwarding
ssh -X username@hostname
```

**If using WSL2:**
```bash
# Make sure X server is running on Windows (VcXsrv/Xming)
export DISPLAY=:0
```

**If on local machine:**
```bash
export DISPLAY=:0
```

**If RViz not installed:**
```bash
sudo apt update
sudo apt install ros-humble-rviz2 ros-humble-rviz-default-plugins
```

---

## Summary of What Was Fixed

1. **RViz launch file** - Now explicitly launches RViz (was using broken utility function)
2. **Robot State Publisher** - Added to all launch files (loads robot model)
3. **Test files created** - Easy ways to test if RViz works
4. **Diagnostic tools** - Scripts to check your environment

## Files You Can Now Use

### Launch Files
- `demo.launch.py` - **Main launch file** (robot + MoveIt + RViz)
- `test_rviz_minimal.launch.py` - Test if RViz works at all
- `test_rviz_only.launch.py` - Test RViz with MoveIt config

### Helper Scripts
- `rebuild_and_test_rviz.sh` - **Run this first!**
- `check_rviz_environment.sh` - Check what's wrong

### Documentation
- `RVIZ_FIX_COMPLETE.md` - Complete details of all fixes
- `RVIZ_DEBUG_GUIDE.md` - Debugging guide
- `LAUNCH_FILES_FIXED.md` - Original robot model fix
- `LAUNCH_ARCHITECTURE.md` - System architecture

---

## The Bottom Line

**Run this command and follow the prompts:**

```bash
cd /home/qiaoqiaochen/appdata/canros/src
./rebuild_and_test_rviz.sh
```

That's all you need to do!

---

## Still Having Issues?

If RViz still doesn't work after running the script:

1. Run the environment check:
   ```bash
   ./check_rviz_environment.sh
   ```

2. Tell me:
   - Which checks failed (marked with ✗)
   - What error messages you see
   - Are you using SSH, WSL2, or local machine?

3. I'll provide specific solutions for your situation.

---

**Quick Summary:**
- ✅ RViz launch is fixed
- ✅ Robot model loading is fixed  
- ✅ Test tools are ready
- 🚀 Run `./rebuild_and_test_rviz.sh` now!

