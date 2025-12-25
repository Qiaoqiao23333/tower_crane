# CANopen SDO Timeout Troubleshooting Guide

## Problem Description

You're seeing errors like:
```
[ERROR] [hook_joint]: AsyncUpload:01:6041:00: SDO protocol timed out (05040000): SDO protocol timed out
[ERROR] [slewing_joint]: AsyncUpload:03:6041:00: SDO protocol timed out (05040000): SDO protocol timed out
```

This means the CANopen master cannot communicate with nodes 1 (hook_joint) and 3 (slewing_joint).

## Root Causes (Most Common)

1. **Hardware Not Connected/Powered**
   - Motor drives are powered off
   - CAN cables disconnected
   - USB-CAN adapter not plugged in

2. **CAN Interface Not Configured**
   - Interface `can0` is not up
   - Wrong baud rate (should be 500 kbps)

3. **Wrong Node IDs**
   - Physical devices have different node IDs than configured
   - Check DIP switches or device configuration

4. **CAN Bus Issues**
   - Missing termination resistors (need 120Ω at each end)
   - Broken cables
   - Electromagnetic interference

5. **Timeout Too Short**
   - USB-CAN adapters can be slow
   - Network latency

## Quick Diagnostic Steps

### Step 1: Run the diagnostic script

```bash
cd /home/qiaoqiaochen/appdata/canros/src
./diagnose_canopen.sh
```

This will check:
- CAN interface status
- Node responsiveness
- Error registers

### Step 2: Check CAN interface manually

```bash
# Check if can0 exists and is UP
ip link show can0

# If DOWN, bring it up with correct baud rate
sudo ip link set can0 up type can bitrate 500000

# Check for any CAN traffic (Ctrl+C to stop)
candump can0
```

### Step 3: Test individual nodes

```bash
# Test node 1 (hook_joint) - read statusword
cansend can0 601#4041600000000000

# Watch for response (should see 581#...)
candump can0,581:7FF

# Repeat for node 2 (trolley_joint)
cansend can0 602#4041600000000000
candump can0,582:7FF

# Repeat for node 3 (slewing_joint)
cansend can0 603#4041600000000000
candump can0,583:7FF
```

**Expected response format:** `581#4B41600027000000` (or similar)
**If no response:** Node is not responding - check power/connection/node ID

### Step 4: Use built-in ROS2 diagnostics

Your launch file has a diagnostic mode:

```bash
ros2 launch tower_crane hardware_bringup_real.launch.py \
  diagnose_canopen:=true \
  can_interface_name:=can0
```

This will continuously poll statusword (0x6041) and error code (0x603F) for all nodes.

### Step 5: Try pre-enabling drives

If nodes respond but don't initialize:

```bash
ros2 launch tower_crane hardware_bringup_real.launch.py \
  pre_enable_drives:=true \
  can_interface_name:=can0
```

This sends CiA402 enable sequence before starting the controller manager.

## Solutions Applied

### ✅ 1. Increased SDO Timeout Values

Updated `/home/qiaoqiaochen/appdata/canros/src/crane/tower_crane/config/robot_control/bus.yml`:

- **Before:** `sdo_timeout_ms: 100` (slewing_joint, trolley_joint) and `1000` (hook_joint)
- **After:** `sdo_timeout_ms: 2000` (all joints)

This gives more time for slow USB-CAN adapters or nodes to respond.

### 📝 2. Configuration Summary

Current settings:
- **CAN Interface:** `can0`
- **Baud Rate:** 500 kbps
- **Node IDs:**
  - Node 1: hook_joint (hoist motor)
  - Node 2: trolley_joint
  - Node 3: slewing_joint
- **SDO Timeout:** 2000ms (2 seconds)
- **Boot Timeout:** 20000ms (20 seconds)

## Hardware Checklist

Before running the system, verify:

- [ ] All motor drives are powered on (check power indicator LEDs)
- [ ] CAN cables are securely connected (CANH, CANL, GND)
- [ ] CAN bus has 120Ω termination at both ends
- [ ] USB-CAN adapter is plugged in and recognized (`lsusb`)
- [ ] Node IDs on physical devices match configuration (1, 2, 3)
- [ ] Baud rate on drives is set to 500 kbps
- [ ] CAN interface is up: `ip link show can0 | grep UP`

## Common USB-CAN Adapter Setup

If using PEAK USB-CAN or similar:

```bash
# Load driver (if needed)
sudo modprobe peak_usb

# Check if device is detected
lsusb | grep -i peak

# Set up can0 interface
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up

# Verify
ip -details link show can0
```

## Advanced Debugging

### Enable verbose CANopen logging

Edit your ROS2 launch file or set environment variable:

```bash
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}] [{time}]: {message}"
export RCUTILS_LOGGING_MIN_SEVERITY=DEBUG
```

### Monitor all CAN traffic

```bash
# Show all CAN frames with timestamps
candump -L can0

# Filter by node responses (581, 582, 583 are SDO responses)
candump can0,581:7FF,582:7FF,583:7FF
```

### Check for CAN errors

```bash
# Show interface statistics
ip -details -statistics link show can0

# Look for:
# - RX/TX errors
# - Bus-off state
# - Error warning/passive states
```

## Next Steps

1. **Run diagnostic script:** `./diagnose_canopen.sh`
2. **Check output for which nodes respond**
3. **If nodes still don't respond:**
   - Verify physical connections
   - Check power supplies
   - Verify node ID configuration on hardware
   - Try different CAN interface or adapter
4. **If timeout persists but nodes respond sometimes:**
   - Increase `sdo_timeout_ms` further (try 5000)
   - Check for network congestion
   - Reduce sync_period if too frequent

## Reference Documentation

- **CiA 402 Profile:** Motor control standard
- **Object 0x6041:** Statusword (motor state)
- **Object 0x6040:** Controlword (motor commands)
- **Object 0x603F:** Error code register
- **Error 05040000:** SDO protocol timeout

## Contact Information

For further assistance:
1. Check motor drive manufacturer documentation for CANopen setup
2. Verify EDS file (`DSY-C.EDS`) matches your hardware
3. Consult ros2_canopen documentation: https://github.com/ros-industrial/ros2_canopen



