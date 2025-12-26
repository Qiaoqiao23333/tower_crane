# Quick Start - CANopen Timeout Fix Applied

## ⚡ Quick Launch Commands

### Standard Launch (Try This First)
```bash
cd ~/appdata/ws_tower_crane
source install/setup.bash
ros2 launch tower_crane hardware_bringup_real.launch.py
```

### With Diagnostics (Recommended for First Test)
```bash
ros2 launch tower_crane hardware_bringup_real.launch.py diagnose_canopen:=true
```

### With Pre-Enable (If Standard Launch Has Issues)
```bash
ros2 launch tower_crane hardware_bringup_real.launch.py pre_enable_drives:=true
```

## ✅ What Was Fixed

- **Increased SDO timeout**: 5s → 10s for all devices
- **Increased boot timeout**: 5s → 10s for all devices  
- **Added state transition delays**: Prevents overwhelming devices
- **Fixed build system**: Removed dependency on missing dcfgen tool

## 🔍 Quick Diagnostics

### Check Device Status
```bash
cd ~/appdata/ws_tower_crane/src/tower_crane
./diagnose_canopen.sh
```

### Check CAN Bus
```bash
ip link show can0          # Should show: UP,LOWER_UP,ECHO
ip -s link show can0       # Check for errors
candump can0 | head -20    # Monitor traffic
```

### Manual Device Query (Node 2 - Trolley)
```bash
# Read statusword
cansend can0 602#4041600000000000 && candump -n 1 can0,582:7FF

# Expected: 0x0237 (Operation Enabled) or 0x0231 (Ready to Switch On)
```

## 📊 Expected Output

### Successful Launch Should Show:
```
[INFO] [canopen_402_driver]: Init: Read State
[INFO] [canopen_402_driver]: Init: Enable
[INFO] [slewing_joint]: Switching to 'slewing_joint/position' command mode
[INFO] [trolley_joint]: Switching to 'trolley_joint/position' command mode
[INFO] [hook_joint]: Switching to 'hook_joint/position' command mode
[INFO] [spawner_forward_position_controller]: Configured and activated
```

### ❌ No More Timeout Errors:
~~`sync_sdo_write_typed: id=2 index=0x6040 subindex=0 timed out`~~

## 🛠️ If Problems Persist

1. **Run diagnostics:**
   ```bash
   ./diagnose_canopen.sh
   ```

2. **Check for fault state:**
   ```bash
   # Reset node 2 if in fault
   cansend can0 602#2B40600080000000
   sleep 0.1
   cansend can0 602#2B40600000000000
   ```

3. **Verify CAN interface:**
   ```bash
   sudo ip link set can0 down
   sudo ip link set can0 up type can bitrate 500000
   ```

4. **Check physical connections:**
   - CAN-H and CAN-L properly connected
   - 120Ω termination at both ends
   - No loose connections

## 📚 Documentation

- **Summary**: `TIMEOUT_FIX_SUMMARY.md`
- **Detailed Guide**: `CANOPEN_TIMEOUT_FIX.md`
- **This File**: `QUICK_START.md`

## 🎯 Test Commands After Launch

Once the system is running, test with:

```bash
# Check joint states
ros2 topic echo /joint_states

# Send a position command
ros2 topic pub /forward_position_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['slewing_joint', 'trolley_joint', 'hook_joint'],
  points: [{
    positions: [0.0, 0.0, 0.0],
    time_from_start: {sec: 2, nanosec: 0}
  }]
}" --once
```

## 💡 Tips

- First launch may take longer as devices initialize
- Watch for the "Switching to position command mode" messages
- All three joints should switch modes successfully
- Controllers should activate without errors

## 🚨 Emergency Stop

If something goes wrong:
```bash
# Press Ctrl+C in the launch terminal

# Or send NMT stop to all nodes
cansend can0 000#0200  # Stop node 2
cansend can0 000#0100  # Stop node 1  
cansend can0 000#0300  # Stop node 3
```

## ✨ Success Indicators

- ✅ No timeout errors in logs
- ✅ All three joints report "Switching to position command mode"
- ✅ Controllers activate successfully
- ✅ `/joint_states` topic publishes data
- ✅ System responds to position commands

---

**Last Updated**: After applying CANopen timeout fixes
**Status**: Ready for testing

