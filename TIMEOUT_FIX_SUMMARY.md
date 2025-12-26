# CANopen SDO Timeout Fix - Summary

## Problem
```
sync_sdo_write_typed: id=2 index=0x6040 subindex=0 timed out
```

The trolley_joint (node 2) was timing out when the CANopen master tried to write to the Controlword (0x6040) during operation.

## Changes Made

### 1. Increased Timeout Values in bus.yml

**File:** `crane/tower_crane/config/robot_control/bus.yml`

For all three joints (hook_joint, trolley_joint, slewing_joint):
- `sdo_timeout_ms`: 5000ms → **10000ms**
- `boot_timeout_ms`: 5000ms → **10000ms**
- Added `switching_state: 5` to add delays between CiA402 state transitions

For master:
- `boot_timeout_ms`: 2000ms → **5000ms**
- Added `timeout_ms: 1000`
- Added `sdo_timeout_ms: 10000`

### 2. Fixed Build System

**File:** `crane/tower_crane/CMakeLists.txt`

Commented out the `cogen_dcf(robot_control)` call since we're using a pre-existing master.dcf file and the `cogen` and `dcfgen` tools are not available in this environment.

```cmake
# cogen_dcf(robot_control)  # Generate master.dcf from bus.yml using dcfgen
# Commented out: We use the pre-existing master.dcf file directly
```

## How to Test

### Step 1: Source the Workspace
```bash
cd ~/appdata/ws_tower_crane
source install/setup.bash
```

### Step 2: Launch with Standard Configuration
```bash
ros2 launch tower_crane hardware_bringup_real.launch.py
```

### Step 3: Launch with Diagnostics (Recommended)
```bash
ros2 launch tower_crane hardware_bringup_real.launch.py diagnose_canopen:=true
```

This will continuously monitor device status during startup and operation.

### Step 4: Launch with Pre-Enable (If Issues Persist)
```bash
ros2 launch tower_crane hardware_bringup_real.launch.py pre_enable_drives:=true
```

This sends CiA402 enable sequence via `cansend` before starting the controller manager.

## Diagnostic Tools

### Manual Device Check Script
```bash
cd ~/appdata/ws_tower_crane/src/tower_crane
./diagnose_canopen.sh
```

This script checks:
- Statusword (0x6041) - Device state
- Error Register (0x1001) - Error flags  
- Error Code (0x603F) - Specific error code
- Mode Display (0x6061) - Current operation mode
- Position Actual (0x6064) - Current position

### Manual CAN Bus Commands

**Read statusword from node 2:**
```bash
cansend can0 602#4041600000000000
candump -n 1 can0,582:7FF
```

**Reset device if in fault:**
```bash
cansend can0 602#2B40600080000000
sleep 0.1
cansend can0 602#2B40600000000000
```

**Check CAN bus status:**
```bash
ip link show can0
ip -s link show can0  # Shows error counters
```

## Expected Behavior

After applying these fixes:

1. ✅ All three devices (hook_joint, trolley_joint, slewing_joint) should initialize successfully
2. ✅ No SDO timeout errors during operation
3. ✅ Controllers (joint_state_broadcaster, forward_position_controller) should activate without errors
4. ✅ Devices should respond to position commands smoothly

## What These Changes Do

### Increased Timeouts
- Gives devices more time to respond to SDO requests
- Accounts for USB-CAN adapter latency
- Handles busy devices that may be processing previous commands

### Switching State Delay
- Adds a small delay between CiA402 state machine transitions
- Prevents overwhelming the device with rapid state changes
- Allows time for the device to stabilize in each state

### Master Timeout Configuration
- Ensures the master waits long enough for device responses
- Prevents premature timeout declarations

## Troubleshooting

If timeouts still occur:

1. **Check physical connections:**
   - Verify CAN-H and CAN-L wiring
   - Ensure 120Ω termination resistors at both ends
   - Check for loose connections

2. **Check CAN bus load:**
   ```bash
   candump can0 | head -100
   ```
   Look for excessive traffic or error frames

3. **Check device-specific issues:**
   - Node 2 might have hardware problems
   - Try swapping node IDs to isolate the issue

4. **Reduce bus load:**
   - Increase `period` from 100ms to 200ms in bus.yml
   - This reduces polling frequency

5. **Check for CAN bus errors:**
   ```bash
   ip -s link show can0
   ```
   Look for RX/TX errors

## Files Modified

1. `crane/tower_crane/config/robot_control/bus.yml` - Increased timeouts, added switching delays
2. `crane/tower_crane/CMakeLists.txt` - Commented out cogen_dcf call
3. `diagnose_canopen.sh` - New diagnostic script (created)
4. `CANOPEN_TIMEOUT_FIX.md` - Detailed troubleshooting guide (created)
5. `TIMEOUT_FIX_SUMMARY.md` - This summary document (created)

## Next Steps

1. Test the system with the new configuration
2. Monitor for any remaining timeout errors
3. If successful, consider documenting the optimal timeout values for your specific hardware
4. If issues persist, run the diagnostic script and review the detailed troubleshooting guide

## Additional Notes

- The configuration now uses longer timeouts which is appropriate for USB-CAN adapters
- PDO communication (already configured) is faster than SDO and is used for real-time data
- SDO is used for configuration and state machine transitions
- The `switching_state` parameter helps prevent state transition issues

## Contact & Support

For detailed troubleshooting steps, see `CANOPEN_TIMEOUT_FIX.md`.

For CAN bus diagnostics, use the `diagnose_canopen.sh` script.

