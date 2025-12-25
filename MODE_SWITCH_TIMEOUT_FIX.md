# Fix: Mode Switch Timeout Error

## The Error You Saw

```
[ros2_control_node-2] [INFO] [slewing_joint]: Switching to 'slewing_joint/position' command mode with CIA402 operation mode '7'
[ros2_control_node-2] [INFO] [canopen_402_driver]: Mode switch timed out.
[ros2_control_node-2] [ERROR] [resource_manager]: Component 'canopen_ros2_control/CanopenSystem' could not perform switch
```

## Root Cause

The `bus.yml` configuration was **inconsistent**:
- ✅ `position_mode: 7` was set (Interpolated Position Mode)
- ❌ BUT the SDO section still had `value: 1` (Profile Position Mode)  
- ❌ AND interpolation time settings (0x60C2) were **completely missing**

**Interpolated Position Mode (mode 7) REQUIRES interpolation time to be configured**, otherwise the motor rejects the mode switch!

## What Was Fixed

### 1. SDO Configuration - All 3 Joints

**Before (WRONG):**
```yaml
sdo:
  - {index: 0x6060, sub_index: 0, value: 1}  # Mode 1 - NOT SUPPORTED!
  - {index: 0x6040, sub_index: 0, value: 6}
  ...
```

**After (CORRECT):**
```yaml
sdo:
  # Set interpolation time (REQUIRED for mode 7)
  - {index: 0x60C2, sub_index: 1, value: 50}  # Interpolation time period
  - {index: 0x60C2, sub_index: 2, value: -3}  # Base 10^-3s
  - {index: 0x6081, sub_index: 0, value: 1000}  # Profile velocity
  - {index: 0x6083, sub_index: 0, value: 2000}  # Profile acceleration
  # Set to Interpolated Position mode (7)
  - {index: 0x6060, sub_index: 0, value: 7}  # Mode 7 - SUPPORTED!
  - {index: 0x6040, sub_index: 0, value: 6}
  ...
```

### 2. TPDO Configuration - Split Into 2 TPDOs (per image requirements)

**Before:**
```yaml
tpdo:
  1:
    mapping:
      - {index: 0x6041, sub_index: 0}  # statusword
      - {index: 0x6061, sub_index: 0}  # mode display
      - {index: 0x6064, sub_index: 0}  # position
      - {index: 0x606C, sub_index: 0}  # velocity
```

**After:**
```yaml
tpdo:
  1:
    mapping:
      - {index: 0x6041, sub_index: 0}  # statusword
      - {index: 0x6061, sub_index: 0}  # mode display
  2:
    mapping:
      - {index: 0x6064, sub_index: 0}  # position
      - {index: 0x606C, sub_index: 0}  # velocity
  3:
    enabled: false
  4:
    enabled: false
```

## Critical Points About Interpolated Position Mode

1. **Interpolation Time is MANDATORY**
   - 0x60C2 sub 1: Period (50 = 50 units)
   - 0x60C2 sub 2: Time base (-3 = 10^-3 seconds = milliseconds)
   - Result: 50ms interpolation period

2. **Mode Must Be Set BEFORE Enabling**
   - Order matters! Set interpolation time → Set mode → Enable motor

3. **DSY-C Motor Supports:**
   - ✅ Mode 7: Interpolated Position
   - ✅ Mode 3: Profile Velocity
   - ✅ Mode 6: Homing
   - ❌ Mode 1: Profile Position (NOT supported!)

## What To Do Now

**1. Restart your ROS2 system:**
```bash
# Press Ctrl+C to stop current process
# Then restart:
ros2 launch tower_crane hardware_bringup_real.launch.py
```

**2. Watch for successful initialization:**
```
[INFO] [slewing_joint]: Switching to 'slewing_joint/position' command mode with CIA402 operation mode '7'
[INFO] [controller_manager]: Configured and activated forward_position_controller
```

If you see "Mode switch timed out" again, there may be a hardware issue.

**3. Test motor movement:**
```bash
cd /home/qiaoqiaochen/appdata/canros/src
./move_trolley_interpolated_mode.sh
```

## Expected Behavior After Fix

✅ Mode switch should complete successfully  
✅ Controllers should activate without timeout  
✅ Motor should respond to position commands  
✅ No "Mode switch timed out" errors

## If It Still Doesn't Work

1. **Check motor status via SDO:**
   ```bash
   # Check if mode was accepted (should return 7)
   ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x6061, subindex: 0}"
   
   # Check interpolation time was set (should return 50)
   ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x60C2, subindex: 1}"
   ```

2. **Check for hardware faults:**
   ```bash
   # Read statusword (check for fault bits)
   ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x6041, subindex: 0}"
   ```

3. **Try resetting the motor:**
   ```bash
   ros2 service call /trolley_joint/recover std_srvs/srv/Trigger
   ros2 service call /trolley_joint/init std_srvs/srv/Trigger
   ```

## Files Modified

- ✅ `crane/tower_crane/config/robot_control/bus.yml` - All 3 joints fixed
  - Added interpolation time configuration (0x60C2)
  - Changed mode from 1 to 7 in SDO section
  - Split TPDO into 2 separate PDOs
  - Added profile velocity/acceleration settings

## Summary

The timeout was caused by **missing interpolation time configuration**. Interpolated Position Mode cannot work without it - the motor will reject the mode switch. Now that it's properly configured, the mode switch should succeed and the motor should respond to commands.


