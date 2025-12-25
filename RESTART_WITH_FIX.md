# 🔧 Restart System with Fix Applied

## Quick Steps

### 1. Stop Current System
In the terminal where the system is running, press:
```
Ctrl+C
```

### 2. Restart with Fixed Code
```bash
sudo /home/qiaoqiaochen/appdata/canros/src/start_tower_crane.sh
```

**NOTE**: Do NOT use `pre_enable` - it's not needed anymore!

### 3. BE PATIENT!

The system will now take **up to 3-4 minutes** to start because it's properly initializing all motors.

You'll see messages like:
```
[INFO] [...]: Initializing motors (this may take up to 180 seconds)...
[INFO] [...]: Initializing motor: hook_joint
[INFO] [canopen_402_driver]: Init: Read State
[INFO] [canopen_402_driver]: Init: Enable
```

**This is normal and expected!** Just wait.

### 4. Success Indicators

After motors initialize, you should see:
```
[INFO] [...]: Successfully initialized: hook_joint
[INFO] [...]: Successfully initialized: trolley_joint  
[INFO] [...]: Successfully initialized: slewing_joint
[INFO] [...]: All motors initialized successfully!
```

Then controllers will spawn:
```
[INFO] [slewing_joint]: Switching to 'slewing_joint/position' command mode with CIA402 operation mode '1'
✅ NO "Mode is not supported" ERROR!
[INFO] [spawner]: Configured and activated forward_position_controller
```

### 5. Verify It Works

In a new terminal:
```bash
source /opt/ros/humble/setup.bash
source /home/qiaoqiaochen/appdata/canros/install/setup.bash

# Check joint states
ros2 topic echo /joint_states --once
```

You should see positions for all 3 joints!

## What Changed?

- ✅ Motors are now **automatically initialized** at startup
- ✅ Operation modes are properly registered
- ✅ No more "Mode is not supported" error
- ⏱️ Startup time increased from 10s to ~180s (acceptable trade-off)

## Troubleshooting

### If motors fail to initialize:

Check CAN traffic in another terminal:
```bash
candump can0 | grep "58[123].*41 60"
```

You should see statuswords changing from `0x0250` → `0x0027`

### If it takes longer than 5 minutes:

Something is wrong. Check:
1. Are motors powered on?
2. Is CAN interface working? `ip link show can0`
3. Any CAN errors? `ip -s link show can0`

Press Ctrl+C and check the troubleshooting section in `FINAL_FIX_MODE_NOT_SUPPORTED.md`.

## Need More Info?

- **Full technical details**: `FINAL_FIX_MODE_NOT_SUPPORTED.md`
- **What was changed**: See "Files Modified" section in FINAL_FIX
- **Why this fix works**: See "Why This Fix Works" section

## Timeline

- **0-10s**: System starts, ros2_control_node launches
- **10-190s**: Motors being initialized (be patient!)
- **190-200s**: Controller spawners start
- **200s+**: System ready to use!

Total: ~3.5 minutes from start to ready.

