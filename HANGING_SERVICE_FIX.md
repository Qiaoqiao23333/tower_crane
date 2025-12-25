# Fix: Service Calls Hanging ("waiting for service to become available...")

## Problem

You run a service call like:
```bash
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"
```

And it **hangs forever** showing:
```
waiting for service to become available...
```

**BUT** the service is listed when you check:
```bash
ros2 service list | grep trolley_joint/target
# Shows: /trolley_joint/target ✓
```

## Root Cause

The service **exists** but the CANopen node is **stuck waiting for the motor** to respond via SDO (Service Data Object) protocol. When you call the service:

1. ROS2 service call → CANopen node
2. CANopen node → Sends SDO request to motor via CAN bus
3. Motor **should** respond with SDO acknowledgment
4. **Motor doesn't respond** → Node waits forever
5. Your service call **hangs** because the node is blocked

## Why Does This Happen?

### Common Causes:

1. **Motor not powered/connected** 
   - Motor is off or CAN cable disconnected
   - Motor is on but not responding to SDO

2. **Motor in wrong state**
   - Not initialized
   - Not in correct operation mode
   - In fault/error state

3. **CANopen communication issue**
   - SDO timeout too short (but yours is already 2000ms)
   - CAN bus errors
   - Motor using PDO instead of SDO

4. **Node in deadlocked state**
   - Previous command is still processing
   - Internal mutex/lock issue
   - Need to restart node

## Quick Diagnosis

### Step 1: Run the diagnostic script

```bash
cd /home/qiaoqiaochen/appdata/canros/src
./test_trolley_with_timeout.sh
```

This will:
- ✅ Check CAN interface
- ✅ Test direct motor communication
- ✅ Test services with timeout protection
- ✅ Show exactly where it fails

### Step 2: Test motor directly on CAN bus

```bash
# Send SDO read request to trolley motor (Node ID 2)
cansend can0 602#4041600000000000

# Watch for response (should appear within 1 second)
timeout 1 candump can0,582:7FF
```

**Expected:** You should see something like `582#4B41600027000000`

**If no response:** Motor is not communicating → Check power/cables

### Step 3: Check service with timeout

```bash
# Try service call with 3 second timeout
timeout 3 ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"
```

**If it times out:** CANopen node is stuck waiting for motor

## Solutions

### Solution 1: Try Recovery Sequence (If Motor Responds on CAN)

Run the recovery script:
```bash
./fix_hanging_service.sh
```

Or manually:
```bash
# Set position mode (with timeout)
timeout 3 ros2 service call /trolley_joint/position_mode std_srvs/srv/Trigger

# Enable motor (with timeout)  
timeout 3 ros2 service call /trolley_joint/enable std_srvs/srv/Trigger

# Try target again (with timeout)
timeout 3 ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"
```

### Solution 2: Restart CANopen Node (Most Reliable)

If services still hang, **restart the launch file:**

1. **Stop current launch:** Press `Ctrl+C` in terminal running the launch
2. **Restart:**
   ```bash
   ros2 launch tower_crane hardware_bringup_real.launch.py
   ```
3. **Try again** after node starts up

### Solution 3: Check Motor Power/Connection

If motor doesn't respond on CAN bus:

```bash
# Check CAN interface
ip link show can0

# If DOWN, bring it up
sudo ip link set can0 up type can bitrate 500000

# Check for CAN errors
ip -details -statistics link show can0
```

Then verify:
- Motor power supply is ON
- CAN cables connected (CANH, CANL, GND)
- Node ID is correct (should be 2 for trolley_joint)
- Baud rate is 500 kbps

### Solution 4: Check for CAN Bus Errors

```bash
# Check error statistics
ip -details -statistics link show can0

# Look for high error counts in:
# - RX errors
# - TX errors  
# - Bus-off errors
```

If errors are high:
```bash
# Reset CAN interface
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 500000
```

## Prevention

To avoid this issue in the future:

1. **Always use timeouts** when calling services:
   ```bash
   timeout 5 ros2 service call /trolley_joint/target ...
   ```

2. **Check motor state before sending commands:**
   ```bash
   # Verify motor is operational
   ros2 topic echo /trolley_joint/nmt_state --once
   ```

3. **Test direct CAN communication first:**
   ```bash
   cansend can0 602#4041600000000000
   timeout 1 candump can0,582:7FF
   ```

4. **Use the complete initialization sequence:**
   ```bash
   # 1. Init
   ros2 service call /trolley_joint/init std_srvs/srv/Trigger
   
   # 2. Set mode
   ros2 service call /trolley_joint/position_mode std_srvs/srv/Trigger
   
   # 3. Enable
   ros2 service call /trolley_joint/enable std_srvs/srv/Trigger
   
   # 4. Send target
   ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"
   ```

## Scripts Available

| Script | Purpose |
|--------|---------|
| `test_trolley_with_timeout.sh` | Complete diagnostic with timeout protection |
| `fix_hanging_service.sh` | Try to recover from hanging state |
| `diagnose_trolley_motor.sh` | Check motor status and state |
| `diagnose_canopen.sh` | Check CAN bus and all nodes |

## Understanding the Issue

```
┌─────────────┐                ┌──────────────┐                ┌───────┐
│   You       │  ros2 service  │  CANopen     │  SDO via CAN   │ Motor │
│             │  ────────────> │  Node        │  ────────────> │       │
│             │                │              │                │       │
│             │                │  WAITING...  │  <───────────  │   X   │
│             │                │  (blocked)   │  NO RESPONSE!  │ (off) │
│             │                │              │                │       │
│  HANGING... │  <─────────── │  (timeout)   │                │       │
│             │  (no response) │              │                │       │
└─────────────┘                └──────────────┘                └───────┘
```

The CANopen node is stuck in a blocking call waiting for SDO response from motor.
Your ROS2 service call waits for the node, which waits for the motor.
**Everyone is waiting!**

## Technical Details

### SDO vs PDO

**SDO (Service Data Object):**
- Request/response protocol
- Slower, guaranteed delivery
- Used for configuration, commands
- **Can hang if motor doesn't respond**

**PDO (Process Data Object):**
- Broadcast protocol
- Faster, real-time
- Used for cyclic data (position, velocity)
- **Doesn't block if motor doesn't respond**

Your system is using SDO for target position commands, which is why it can hang.

### Timeout Settings

In `crane/tower_crane/config/robot_control/bus.yml`:
```yaml
trolley_joint:
  sdo_timeout_ms: 2000  # Already increased to 2 seconds
```

This is already quite high. If it still times out, the motor is genuinely not responding.

## When to Give Up and Restart

If after trying all solutions, services still hang:

**The CANopen node is in a deadlocked internal state.**

Only solution: **Restart the launch file**

This is not a bug in your code - it's a limitation of blocking SDO communication.
The node needs to be restarted to clear its internal state.

## Summary

✅ **Service exists** - Check with `ros2 service list`  
❌ **Service hangs** - Node waiting for motor SDO response  
🔧 **Quick fix** - Test direct CAN, try recovery, restart launch  
🛡️ **Prevention** - Always use timeouts, check motor first  


