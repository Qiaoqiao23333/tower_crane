# Fix: "Mode is not supported" Error

## Problem

When starting the tower crane system, you see:
```
[INFO] [slewing_joint]: Switching to 'slewing_joint/position' command mode with CIA402 operation mode '1'
[INFO] [canopen_402_driver]: Mode is not supported.
[ERROR] [resource_manager]: Component 'canopen_ros2_control/CanopenSystem' could not perform switch
```

## Root Cause

The system **skips automatic motor initialization** to prevent blocking controller_manager startup:
```
[WARN] [canopen_ros2_control/CanopenSystem_interface]: Skipping automatic motor initialization to prevent blocking
```

Without initialization:
1. `motor_->handleInit()` is never called
2. Motor modes are never registered in the `modes_` map  
3. `allocMode()` returns null → "Mode is not supported" error

## Solution: Restart with Pre-Enable

The system must be restarted with motor pre-enablement. This sends CIA402 enable commands via CAN **before** controller_manager starts.

### Step 1: Stop Current System

Press **Ctrl+C** in the terminal running the hardware system.

### Step 2: Restart with Pre-Enable

Use the updated start script:

```bash
sudo /home/qiaoqiaochen/appdata/canros/src/start_tower_crane.sh pre_enable
```

Or manually:

```bash
sudo /home/qiaoqiaochen/appdata/canros/src/start_tower_crane.sh
# When prompted, launch with:
ros2 launch tower_crane hardware_bringup_real.launch.py pre_enable_drives:=true
```

### What This Does

With `pre_enable_drives:=true`, the launch file (lines 161-189):
1. Sends CAN commands to set mode 0x6060=1 (Profile Position Mode)
2. Sends CIA402 state machine sequence:
   - Shutdown (controlword = 0x06)
   - Switch On (controlword = 0x07)  
   - Enable Operation (controlword = 0x0F)
3. Verifies statusword shows 0x0027 (Operation Enabled)
4. **THEN** starts controller_manager

This ensures motors are properly enabled before ros2_control tries to use them.

## Verification

After restarting, you should see:

1. **Pre-enable output** showing motor enablement:
```
[pre_enable_drives] enabling CiA402 nodes 1/2/3 on can0
[pre_enable_drives] node 1 attempt 30 statusword: ..27
[pre_enable_drives] node 2 attempt 30 statusword: ..27
[pre_enable_drives] node 3 attempt 30 statusword: ..27
```

2. **No mode error** - controller should activate cleanly

3. **Joint states** publishing:
```bash
ros2 topic echo /joint_states --once
```

## Alternative: Enable Manually via CAN

If you cannot restart, try manual CAN enablement:

```bash
cd /home/qiaoqiaochen/appdata/canros/src
sudo ./manually_enable_motors.sh
```

But this is **not recommended** because modes still won't be registered. Restarting with `pre_enable_drives:=true` is the proper solution.

## Why This Happens

The ros2_control integration (`robot_system.cpp` lines 140-157) deliberately skips calling `init_motor()` because:
- It can block for 60 seconds per motor (180s total)
- Controller_manager can't respond to service requests while blocked
- Spawners timeout and fail

The trade-off is that motors must be pre-enabled via CAN before starting ros2_control.

