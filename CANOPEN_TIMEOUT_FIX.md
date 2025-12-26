# CANopen SDO Timeout Fix Guide

## Problem Description
Error: `sync_sdo_write_typed: id=2 index=0x6040 subindex=0 timed out`

This error occurs when the CANopen master tries to write to the Controlword (0x6040) of node 2 (trolley_joint) via SDO, but the device doesn't respond within the timeout period.

## Root Causes

1. **Device is busy or in wrong state**: The motor controller might be busy processing previous commands or in a fault state
2. **CAN bus congestion**: Too many messages on the bus causing delays
3. **Insufficient timeout**: The timeout value is too short for the device response time
4. **Hardware issues**: Physical connection problems, termination resistors, or EMI

## Solutions Applied

### 1. Increased SDO Timeouts
- Changed `sdo_timeout_ms` from 5000ms to 10000ms for all devices
- Changed `boot_timeout_ms` from 5000ms to 10000ms for all devices
- Added `timeout_ms` and `sdo_timeout_ms` to master configuration

### 2. Added State Transition Delays
- Added `switching_state: 5` parameter to all devices
- This adds a small delay between CiA402 state transitions

### 3. Master Configuration Enhancement
- Increased master `boot_timeout_ms` from 2000ms to 5000ms
- Added explicit timeout parameters

## Testing Steps

### Step 1: Rebuild the Package
```bash
cd ~/appdata/ws_tower_crane
colcon build --packages-select tower_crane
source install/setup.bash
```

### Step 2: Check CAN Bus Status
```bash
ip link show can0
# Should show: UP,LOWER_UP,ECHO
```

### Step 3: Run Diagnostic Script
```bash
cd ~/appdata/ws_tower_crane/src/tower_crane
./diagnose_canopen.sh
```

This will check:
- Statusword (0x6041) - Device state
- Error Register (0x1001) - Error flags
- Error Code (0x603F) - Specific error code
- Mode Display (0x6061) - Current operation mode
- Position Actual (0x6064) - Current position

### Step 4: Launch with Diagnostics
```bash
ros2 launch tower_crane hardware_bringup_real.launch.py diagnose_canopen:=true
```

This will continuously monitor the device status during startup.

### Step 5: Pre-enable Drives (Alternative)
If the issue persists, try pre-enabling the drives:
```bash
ros2 launch tower_crane hardware_bringup_real.launch.py pre_enable_drives:=true
```

This sends CiA402 enable sequence before starting the controller manager.

## Additional Troubleshooting

### Check for CAN Bus Errors
```bash
ip -s link show can0
# Look for errors in RX/TX counters
```

### Monitor CAN Traffic
```bash
candump can0
# Watch for error frames or missing responses
```

### Check Device Statusword Manually
```bash
# Read statusword from node 2
cansend can0 602#4041600000000000
candump -n 1 can0,582:7FF
```

Expected statusword values:
- `0x0250` (592) - Switch On Disabled
- `0x0231` (561) - Ready to Switch On
- `0x0233` (563) - Switched On
- `0x0237` (567) - Operation Enabled
- `0x0008` (8) - Fault

### Reset Device if in Fault
```bash
# Send fault reset command to node 2
cansend can0 602#2B40600080000000
sleep 0.1
cansend can0 602#2B40600000000000
```

## Hardware Checks

1. **Verify CAN termination**: Both ends of the CAN bus should have 120Ω resistors
2. **Check cable quality**: Use twisted-pair cables, keep away from power lines
3. **Verify power supply**: Ensure all devices have stable power
4. **Check grounding**: All devices should share common ground

## Configuration Parameters Reference

### Key Timing Parameters
- `sdo_timeout_ms`: Timeout for SDO read/write operations (now 10000ms)
- `boot_timeout_ms`: Timeout for device boot sequence (now 10000ms)
- `heartbeat_producer`: Device sends heartbeat every N ms (1000ms)
- `heartbeat_consumer`: Master expects heartbeat within N ms (2000ms)
- `sync_period`: SYNC message period in microseconds (10000µs = 10ms)
- `period`: Polling period for device updates (100ms)
- `switching_state`: Delay between state transitions (5 cycles)

### CiA402 State Machine
```
       ┌──────────────────────────────────────┐
       │                                      │
       ▼                                      │
  [0] Start ──────────────────────────────────┤
       │                                      │
       │ Automatic                            │
       ▼                                      │
  [1] Not Ready to Switch On                 │
       │                                      │
       │ Automatic                            │
       ▼                                      │
  [2] Switch On Disabled ◄────────────────────┤
       │                     │                │
       │ Shutdown (0x06)     │ Fault (0x80)   │
       ▼                     │                │
  [3] Ready to Switch On     │                │
       │                     │                │
       │ Switch On (0x07)    │                │
       ▼                     │                │
  [4] Switched On            │                │
       │                     │                │
       │ Enable Op (0x0F)    │                │
       ▼                     │                │
  [5] Operation Enabled      │                │
       │                     │                │
       └─────────────────────┘                │
                                              │
  [6] Fault ───────────────────────────────────┘
       │
       │ Fault Reset (0x80)
       └──────────────────────────────────────►[2]
```

## Expected Behavior After Fix

1. All three devices should initialize successfully
2. No timeout errors during operation
3. Controllers should activate without errors
4. Devices should respond to position commands

## If Problem Persists

1. **Check device-specific issues**: Node 2 might have hardware problems
2. **Try different node ID**: Temporarily swap node IDs to isolate the problem
3. **Reduce bus load**: Increase polling period to 200ms
4. **Use PDO instead of SDO**: Configure RPDO for controlword (already done)
5. **Check device firmware**: Ensure motor controller firmware is up to date

## Contact Information

If the issue continues after trying all solutions:
1. Run the diagnostic script and save output
2. Capture CAN bus traffic: `candump -l can0`
3. Check system logs: `journalctl -xe`
4. Review motor controller documentation for device-specific requirements

