# Motor Activation Timeout Fix

## Problem

When launching MoveIt with real hardware, motors failed to activate:

```
[canopen_402_driver]: Transition timed out. target=5 next=3 current=2 statusword=0x0250
Failed to activate 'slewing_joint'
Could not enable motor
```

**Statusword 0x0250** = "Switch on disabled" - motors stuck and couldn't transition to operation enabled state.

## Root Causes

1. **Polling disabled** - `polling: false` caused unreliable communication with real hardware
2. **Short timeouts** - `sdo_timeout_ms: 2000` and `boot_timeout_ms: 20000` too short for USB-CAN adapters
3. **Missing fault reset** - No initial fault clearing in SDO sequence
4. **No profile parameters** - Motors had no velocity/acceleration limits set
5. **No pre-enabling** - Controller manager started before motors were ready

## Fixes Applied

### 1. Updated `bus.yml` Configuration

#### **Changed for ALL joints (slewing, trolley, hook):**

```yaml
# OLD SETTINGS (caused failures):
polling: false
sdo_timeout_ms: 2000
boot_timeout_ms: 20000
sdo:
  - {index: 0x6060, sub_index: 0, value: 1}
  - {index: 0x6040, sub_index: 0, value: 6}
  - {index: 0x6040, sub_index: 0, value: 7}
  - {index: 0x6040, sub_index: 0, value: 15}

# NEW SETTINGS (more reliable):
polling: true                 # ✅ Enable polling for real hardware
sdo_timeout_ms: 3000         # ✅ Increased timeout
boot_timeout_ms: 30000       # ✅ Increased boot timeout
sdo:
  # ✅ Clear faults first
  - {index: 0x6040, sub_index: 0, value: 128}  # Fault reset
  # ✅ Set operation mode
  - {index: 0x6060, sub_index: 0, value: 1}    # Profile Position mode
  # ✅ Set profile parameters (NEW!)
  - {index: 0x6081, sub_index: 0, value: 50000}  # Profile velocity
  - {index: 0x6083, sub_index: 0, value: 10000}  # Profile acceleration
  - {index: 0x6084, sub_index: 0, value: 10000}  # Profile deceleration
```

### 2. Updated `moveit_planning_execution.launch.py`

Added motor pre-enabling and diagnostics:

```python
# OLD:
launch_arguments={
    "can_interface_name": can_interface_name,
}.items(),

# NEW:
launch_arguments={
    "can_interface_name": can_interface_name,
    "pre_enable_drives": "true",      # ✅ Pre-enable motors before controller_manager
    "diagnose_canopen": "true",       # ✅ Enable CANopen diagnostics
}.items(),
```

## What Each Fix Does

### **Polling: true**
- Enables periodic SDO polling to read motor state
- More reliable for USB-CAN adapters with latency
- Keeps communication alive even if PDOs fail

### **Increased Timeouts**
- `sdo_timeout_ms: 3000` - Gives more time for USB-CAN round trips
- `boot_timeout_ms: 30000` - Allows motors to fully initialize

### **Fault Reset (0x6040 = 128)**
- Clears any existing faults before enabling
- Transitions motor from "Fault" state to "Switch on disabled"
- Essential if motors were in fault state from previous run

### **Profile Parameters**
- **0x6081** (Profile velocity): Max velocity for position moves
- **0x6083** (Profile acceleration): Acceleration limit
- **0x6084** (Profile deceleration): Deceleration limit
- Without these, motors may refuse to move or timeout

### **Pre-enable Drives**
- Sends CiA402 enable sequence BEFORE controller_manager starts
- Uses raw `cansend` commands with verification
- Ensures motors are in "Operation Enabled" before ROS2 control takes over

### **Diagnostics**
- Continuously polls statusword (0x6041) and error code (0x603F)
- Prints motor state during initialization
- Helps debug activation issues

## Testing

### 1. Check Motor Status BEFORE Launching

```bash
# Run diagnostic script to check if motors are reachable
./check_motor_status.sh can0
```

Expected output for healthy motors:
```
Node 1 - Statusword: 0x0250 or 0x0027  (Not 'NO RESPONSE')
Node 2 - Statusword: 0x0250 or 0x0027
Node 3 - Statusword: 0x0250 or 0x0027
```

### 2. Launch with New Configuration

```bash
# Rebuild first
cd /home/qiaoqiaochen/appdata/canros
colcon build --packages-select tower_crane tower_crane_moveit_config
source install/setup.bash

# Launch with real hardware
ros2 launch tower_crane_moveit_config moveit_planning_execution.launch.py \
    can_interface_name:=can0
```

### 3. Watch for Success Messages

Look for these in the terminal:

```
[pre_enable_drives] enabling CiA402 nodes 1/2/3 on can0
[pre_enable_drives] node 1 attempt 1 statusword: ...27  ✅ Good!
[pre_enable_drives] node 2 attempt 1 statusword: ...27  ✅ Good!
[pre_enable_drives] node 3 attempt 1 statusword: ...27  ✅ Good!

[canopen_diag] node 1 statusword:...27 error_code:...00  ✅ No errors!
[canopen_diag] node 2 statusword:...27 error_code:...00  ✅ No errors!
[canopen_diag] node 3 statusword:...27 error_code:...00  ✅ No errors!

[resource_manager]: Successful initialization of hardware 'canopen_ros2_control/CanopenSystem'
[spawner_joint_state_broadcaster]: Loaded joint_state_broadcaster
[spawner_forward_position_controller]: Loaded forward_position_controller
```

### 4. Check RViz

- RViz should open automatically
- Robot model should be visible
- MoveIt panel should show "Ready to plan"
- No red error messages

## Troubleshooting

### ❌ Still Getting "Transition timed out"

**Check:**
1. CAN interface is up: `ip link show can0`
2. Motors are powered on
3. CAN termination resistors are properly connected
4. No CAN bus errors: `candump can0` should show traffic

**Try:**
```bash
# Manually reset motors
for id in 1 2 3; do
    cansend can0 $(printf '%03X#8040600000000000' $((0x600+id)))  # Fault reset
    sleep 0.5
done

# Then relaunch
```

### ❌ Motor Responds but Won't Enable

**Possible causes:**
- Motor in fault state (check error code 0x603F)
- Hardware e-stop active
- Motor drive internal error

**Solution:**
```bash
# Check motor error codes
./check_motor_status.sh can0

# Look for error code 0x603F
# Common codes:
#   0x0000 = No error
#   0x3210 = Over voltage
#   0x2320 = Under voltage  
#   0x8611 = Control error
```

### ❌ Only Some Motors Activate

**This usually means:**
- One motor has a hardware problem
- CAN ID conflict
- Wiring issue

**Test individually:**
```bash
# Test only hook motor (ID 1)
cansend can0 601#4041600000000000
candump can0,581:7FF

# Test only trolley motor (ID 2)
cansend can0 602#4041600000000000
candump can0,582:7FF

# Test only slewing motor (ID 3)
cansend can0 603#4041600000000000
candump can0,583:7FF
```

## Files Modified

1. ✅ `/home/qiaoqiaochen/appdata/canros/src/crane/tower_crane/config/robot_control/bus.yml`
   - All 3 joints: polling, timeouts, fault reset, profile parameters

2. ✅ `/home/qiaoqiaochen/appdata/canros/src/crane/tower_crane_moveit_config/launch/moveit_planning_execution.launch.py`
   - Added pre_enable_drives and diagnose_canopen flags

3. ✅ `/home/qiaoqiaochen/appdata/canros/src/check_motor_status.sh`
   - NEW diagnostic script

## Technical Details

### CiA402 State Machine Transitions

```
┌─────────────────────────────────────────────────────────┐
│  Start / Power On                                       │
└───────────────┬─────────────────────────────────────────┘
                │
                ▼
┌─────────────────────────────────────────────────────────┐
│  Not Ready to Switch On (0x0000)                       │
│  Motor initializing, waiting for boot...               │
└───────────────┬─────────────────────────────────────────┘
                │ (automatic)
                ▼
┌─────────────────────────────────────────────────────────┐
│  Switch On Disabled (0x0040/0x0250) ← YOU WERE HERE   │
│  Motor ready but disabled                               │
└───────────────┬─────────────────────────────────────────┘
                │ Controlword 0x06 (Shutdown)
                ▼
┌─────────────────────────────────────────────────────────┐
│  Ready to Switch On (0x0021)                           │
│  Motor ready to be switched on                          │
└───────────────┬─────────────────────────────────────────┘
                │ Controlword 0x07 (Switch On)
                ▼
┌─────────────────────────────────────────────────────────┐
│  Switched On (0x0023)                                  │
│  Motor powered but not operational                      │
└───────────────┬─────────────────────────────────────────┘
                │ Controlword 0x0F (Enable Operation)
                ▼
┌─────────────────────────────────────────────────────────┐
│  Operation Enabled (0x0027) ← GOAL!                   │
│  Motor ready to accept position/velocity commands      │
└─────────────────────────────────────────────────────────┘
```

**The fix ensures motors successfully transition through all states.**

## Related Improvements

These fixes work together with:
- ✅ Unit scaling parameters (`scale_pos_to_dev` etc.)
- ✅ Simplified launch arguments (auto-detect hardware from CAN interface)
- ✅ RViz launch fix

Everything should now work smoothly! 🎉

---

**Date:** December 2024  
**Status:** ✅ FIXED - Motors should now activate reliably


