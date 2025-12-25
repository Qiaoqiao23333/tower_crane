# 🚀 Quick Fix Guide: Start Here!

## Your Error

```
[INFO] [canopen_402_driver]: Mode is not supported.
[ERROR] [resource_manager]: Component could not perform switch
```

## The Fix (3 Steps)

### Step 1: Stop Current System

In the terminal where the system is running, press:

```
Ctrl+C
```

Wait for everything to shut down.

### Step 2: Restart with Motor Pre-Enablement

Run this command:

```bash
sudo /home/qiaoqiaochen/appdata/canros/src/start_tower_crane.sh pre_enable
```

### Step 3: Verify It Works

Watch for these success indicators:

1. **Pre-enable messages** (motors being enabled via CAN):
   ```
   [pre_enable_drives] enabling CiA402 nodes 1/2/3 on can0
   [pre_enable_drives] node 1 attempt X statusword: ...27
   [pre_enable_drives] node 2 attempt X statusword: ...27
   [pre_enable_drives] node 3 attempt X statusword: ...27
   ```

2. **Controller activation** (no error):
   ```
   [INFO] [spawner]: Configured and activated forward_position_controller
   ✅ NO "Mode is not supported" error!
   ```

3. **Joint states publishing**:
   ```bash
   # In a new terminal:
   source /opt/ros/humble/setup.bash
   source /home/qiaoqiaochen/appdata/canros/install/setup.bash
   ros2 topic echo /joint_states --once
   ```

## What This Does

The `pre_enable` flag tells the system to:
1. Send CAN commands to enable motors **BEFORE** ros2_control starts
2. Verify motors are in "Operation Enabled" state (statusword 0x0027)
3. Then start the controller_manager

This avoids the "mode not supported" error because motors are properly initialized.

## Troubleshooting

### If pre-enable seems to hang

Check CAN traffic:
```bash
# In another terminal:
sudo candump can0
```

You should see CAN messages being exchanged. If you see only NMT heartbeats (0x701, 0x702, 0x703) but no other traffic, the motors may be in fault state.

### If motors show fault status

Try resetting them:
```bash
# Stop the system (Ctrl+C)
# Reset CAN interface:
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000

# Try starting again:
sudo /home/qiaoqiaochen/appdata/canros/src/start_tower_crane.sh pre_enable
```

### If you still get errors

Check the detailed documentation:
- `MODE_NOT_SUPPORTED_COMPLETE_FIX.md` - Full technical explanation
- `QUICK_FIX_MODE_NOT_SUPPORTED.md` - Alternative solutions

## Why This Happened

The system **intentionally skips** automatic motor initialization to prevent controller_manager from blocking for 180+ seconds. The trade-off is that motors must be pre-enabled via CAN before starting ros2_control.

The error message "Mode is not supported" is misleading - it actually means "Mode was never registered because motors were never initialized."

## Next Steps After Success

Once the system starts successfully:

1. **Check motor positions**:
   ```bash
   ros2 topic echo /joint_states
   ```

2. **Send a test trajectory**:
   ```bash
   # Example: Move trolley joint slightly
   ros2 topic pub /forward_position_controller/joint_trajectory \
     trajectory_msgs/msg/JointTrajectory \
     "{joint_names: ['trolley_joint'], points: [{positions: [0.1], time_from_start: {sec: 2}}]}" \
     --once
   ```

3. **Monitor CAN traffic**:
   ```bash
   sudo candump can0
   ```

## Command Reference

| Command | Purpose |
|---------|---------|
| `sudo start_tower_crane.sh pre_enable` | Start with motor pre-enablement (RECOMMENDED) |
| `sudo start_tower_crane.sh` | Start without pre-enablement (requires manual enable) |
| `ros2 topic echo /joint_states` | Check current joint positions |
| `sudo candump can0` | Monitor CAN bus traffic |
| `ros2 service list \| grep joint` | List available motor services |

## Need Help?

1. Read `MODE_NOT_SUPPORTED_COMPLETE_FIX.md` for full technical details
2. Check CAN traffic with `candump` to verify communication
3. Ensure CAN interface is UP: `ip link show can0`
4. Verify bitrate is 500kbps: `ip -details link show can0`

