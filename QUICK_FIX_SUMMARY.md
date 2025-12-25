# SDO Timeout Fix - Quick Summary

## What I Did

### 1. ✅ Increased SDO Timeout Values
Modified `crane/tower_crane/config/robot_control/bus.yml`:
- Changed `sdo_timeout_ms` from **100ms → 2000ms** for slewing_joint
- Changed `sdo_timeout_ms` from **100ms → 2000ms** for trolley_joint  
- Changed `sdo_timeout_ms` from **1000ms → 2000ms** for hook_joint

This gives your CANopen nodes more time to respond, which helps with:
- Slow USB-CAN adapters
- Network latency
- Busy CAN buses

### 2. 📋 Created Diagnostic Tools

**`diagnose_canopen.sh`** - Automated diagnostic script that checks:
- CAN interface status
- Node responsiveness
- Error registers
- SDO communication

**`CANOPEN_TROUBLESHOOTING.md`** - Comprehensive troubleshooting guide

## Immediate Action Required

### Step 1: Check Your Hardware

**Most likely cause:** The motor drives (nodes) are not responding because they're:
- ❌ Powered off
- ❌ Not connected to CAN bus
- ❌ Wrong node ID configuration
- ❌ CAN interface (`can0`) is down

### Step 2: Verify CAN Interface

```bash
# Check if can0 is up
ip link show can0

# If it's down, bring it up (500 kbps baud rate)
sudo ip link set can0 up type can bitrate 500000
```

### Step 3: Run Diagnostics

```bash
cd /home/qiaoqiaochen/appdata/canros/src
./diagnose_canopen.sh
```

This will tell you **exactly which nodes are responding**.

### Step 4: Test With Your Application

After verifying nodes respond, rebuild and run:

```bash
# Rebuild (only if needed - timeout is in YAML, not compiled code)
cd /home/qiaoqiaochen/appdata/canros
colcon build --packages-select tower_crane

# Source the workspace
source install/setup.bash

# Launch with diagnostics enabled
ros2 launch tower_crane hardware_bringup_real.launch.py \
  diagnose_canopen:=true \
  can_interface_name:=can0
```

## Expected Results

### ✅ Success Case
You should see:
```
[canopen_diag] node 1 statusword:581#4B41600027000000 error_code:581#...
[canopen_diag] node 2 statusword:582#4B41600027000000 error_code:582#...
[canopen_diag] node 3 statusword:583#4B41600027000000 error_code:583#...
```

### ❌ Still Timing Out
If you still see `<no_reply>`:
1. **Check power** to motor drives
2. **Check CAN cables** (CANH, CANL, GND)
3. **Verify node IDs** on physical hardware (should be 1, 2, 3)
4. **Check baud rate** on drives (should be 500 kbps)
5. **Verify termination** resistors (120Ω at each end of bus)

## Why This Happened

The error means the ROS2 CANopen driver tried to read the statusword (object 6041h) from your motor drives but didn't get a response within the timeout period.

**Original timeouts were too short:**
- 100ms is fine for direct CAN interfaces (like PCAN card)
- 100ms is often too short for USB-CAN adapters (latency)
- 100ms can be too short if CAN bus is busy

**New 2000ms timeout:**
- Gives plenty of time for USB-CAN adapters
- Accounts for network latency
- Still fast enough to detect real problems

## If You Still Have Issues

1. Read the full guide: `cat CANOPEN_TROUBLESHOOTING.md`
2. Check that your hardware matches the configuration:
   - Node 1 = hook_joint (hoist motor)
   - Node 2 = trolley_joint
   - Node 3 = slewing_joint
3. Verify EDS file matches your motor drives
4. Try increasing timeout even more (5000ms)
5. Enable pre-enable mode:
   ```bash
   ros2 launch tower_crane hardware_bringup_real.launch.py \
     pre_enable_drives:=true
   ```

## Quick Reference Commands

```bash
# Check CAN interface
ip link show can0

# Bring up CAN interface
sudo ip link set can0 up type can bitrate 500000

# Monitor CAN traffic
candump can0

# Test node 1 manually
cansend can0 601#4041600000000000
candump can0,581:7FF

# Run full diagnostics
./diagnose_canopen.sh

# Launch with diagnostics
ros2 launch tower_crane hardware_bringup_real.launch.py diagnose_canopen:=true
```

## Files Modified

- ✏️ `crane/tower_crane/config/robot_control/bus.yml` - Increased SDO timeouts
- 📄 `diagnose_canopen.sh` - New diagnostic script
- 📖 `CANOPEN_TROUBLESHOOTING.md` - Comprehensive guide
- 📋 `QUICK_FIX_SUMMARY.md` - This file

---

**Next step:** Run `./diagnose_canopen.sh` to see which nodes are responding!



