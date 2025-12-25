# Fixing hook_joint SDO Timeout Errors

## Problem
```
[ERROR] [hook_joint]: AsyncUpload:01:6041:00: SDO protocol timed out
[ERROR] [hook_joint]: AsyncDownload:01:6040:00: SDO protocol timed out
```

**hook_joint** (Node ID 1) is not responding to CANopen SDO requests.

## Immediate Diagnostic Steps

### Step 1: Check if Node 1 is on the CAN Bus

```bash
# Test direct CAN communication with node 1
cansend can0 601#4041600000000000

# Watch for response (should see 581#...)
candump can0,581:7FF
```

**If no response:** Node 1 is not connected/powered/configured correctly.

### Step 2: Check All Nodes

Run the diagnostic script:
```bash
cd /home/qiaoqiaochen/appdata/canros/src
./check_all_nodes.sh
```

Or manually test each node:
```bash
# Test node 1 (hook_joint)
cansend can0 601#4041600000000000
timeout 1s candump can0,581:7FF

# Test node 2 (trolley_joint)  
cansend can0 602#4041600000000000
timeout 1s candump can0,582:7FF

# Test node 3 (slewing_joint)
cansend can0 603#4041600000000000
timeout 1s candump can0,583:7FF
```

### Step 3: Check CAN Interface

```bash
# Verify CAN interface is up
ip link show can0

# If down, bring it up
sudo ip link set can0 up type can bitrate 500000

# Check for CAN errors
ip -details -statistics link show can0
```

## Root Causes

### 1. Hardware Not Connected/Powered ⭐ MOST COMMON

**Symptoms:**
- No response to SDO requests
- No heartbeat messages
- Node doesn't appear on CAN bus

**Solutions:**
- ✅ Check power supply to hook motor drive
- ✅ Verify CAN cables are connected (CANH, CANL, GND)
- ✅ Check if power LED is on the drive
- ✅ Verify motor drive is powered on

### 2. Wrong Node ID

**Symptoms:**
- Node responds but with different ID
- Multiple nodes with same ID (bus conflicts)

**Solutions:**
- ✅ Check DIP switches on motor drive
- ✅ Verify node ID is set to 1 for hook_joint
- ✅ Check bus.yml configuration matches hardware

### 3. CAN Bus Issues

**Symptoms:**
- Intermittent communication
- Some nodes work, others don't
- Bus errors in `ip link show can0`

**Solutions:**
- ✅ Check CAN termination (120Ω at both ends)
- ✅ Verify CANH and CANL are not swapped
- ✅ Check for broken cables
- ✅ Verify baud rate is 500 kbps on all nodes

### 4. Node in Error/Fault State

**Symptoms:**
- Node responds but rejects commands
- Error register (0x1001) shows errors

**Solutions:**
```bash
# Check error register
ros2 service call /hook_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 1001, subindex: 0}"

# Try to recover
ros2 service call /hook_joint/recover std_srvs/srv/Trigger
```

## Quick Fix Sequence

### Option 1: If Node is Not Responding at All

1. **Check hardware:**
   ```bash
   # Verify CAN interface
   ip link show can0
   
   # Test direct CAN communication
   cansend can0 601#4041600000000000
   candump can0,581:7FF
   ```

2. **If no response:**
   - Check power to hook motor drive
   - Verify CAN wiring
   - Check node ID configuration
   - Verify baud rate (500 kbps)

### Option 2: If Node Responds but ROS2 Can't Communicate

1. **Restart ROS2 nodes:**
   ```bash
   # Stop current launch
   # Then restart with diagnostics
   ros2 launch tower_crane hardware_bringup_real.launch.py diagnose_canopen:=true
   ```

2. **Check if node is initialized:**
   ```bash
   ros2 service call /hook_joint/init std_srvs/srv/Trigger
   ```

3. **Increase timeout (if needed):**
   - Already set to 2000ms in bus.yml
   - If still timing out, node is likely not responding

## Configuration Check

Your `bus.yml` now has:
- ✅ `sdo_timeout_ms: 2000` (increased from 100ms)
- ✅ `boot_timeout_ms: 20000` (20 seconds)
- ✅ Proper SDO boot sequence configured

**The timeout is not the problem** - the node is simply not responding.

## Workaround: Disable hook_joint Temporarily

If you only need trolley_joint and slewing_joint:

1. **Comment out hook_joint in bus.yml:**
   ```yaml
   # hook_joint:
   #   node_id: 1
   #   ...
   ```

2. **Or modify URDF to exclude hook_joint** (if possible)

3. **Restart the system**

## Next Steps

1. **Run diagnostic:**
   ```bash
   ./check_all_nodes.sh
   ```

2. **Test direct CAN communication:**
   ```bash
   cansend can0 601#4041600000000000
   candump can0,581:7FF
   ```

3. **Check hardware:**
   - Power supply
   - CAN wiring
   - Node ID configuration

4. **If node responds on CAN but not in ROS2:**
   - Check ROS2 node status
   - Restart CANopen driver
   - Check for conflicting nodes

## Expected Behavior

When working correctly:
- Node 1 responds to SDO requests within 2000ms
- Statusword (0x6041) can be read
- Controlword (0x6040) can be written
- Boot sequence completes successfully
- `/hook_joint/joint_states` topic publishes data

## Summary

**The error means:** Node 1 (hook_joint) is not responding on the CAN bus.

**Most likely cause:** Hardware issue (power, wiring, or node ID).

**Action:** Check physical connections and power supply first, then verify CAN bus communication.


