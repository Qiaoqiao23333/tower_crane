# Fix: Motor Not Moving Even Though Topics Work

## Problem
- ✅ Topics are publishing
- ✅ Joint states show all at 0.0
- ❌ Motor doesn't move when sending target command

## Root Cause
The motor is likely **not enabled** or **not in position mode**.

## Quick Fix Sequence

### Step 1: Check Motor State

```bash
# Check Statusword (0x6041) - shows if motor is enabled
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}"
```

**Expected values:**
- `0x0027` or `0x001F` = Operation enabled ✓
- `0x0006` = Ready to switch on (needs enable)
- `0x0007` = Switched on (needs enable)

### Step 2: Check Operation Mode

```bash
# Check Operation Mode Display (0x6061)
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6061, subindex: 0}"
```

**Expected:** `1` = Profile Position Mode

### Step 3: Enable Motor (Complete Sequence)

```bash
# 1. Initialize (if needed)
ros2 service call /trolley_joint/init std_srvs/srv/Trigger

# 2. Set position mode
ros2 service call /trolley_joint/position_mode std_srvs/srv/Trigger

# 3. Enable motor
ros2 service call /trolley_joint/enable std_srvs/srv/Trigger

# 4. Verify status
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}"
# Should show 0x0027 or 0x001F
```

### Step 4: Try Moving Again

```bash
# Move 90 degrees
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"

# Monitor movement
ros2 topic echo /trolley_joint/joint_states
```

## Alternative: Check if Value Needs to be Different

The value `90.0` might be in wrong units. Try:

```bash
# Check current position in encoder units
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6064, subindex: 0}"

# If motor uses encoder counts (e.g., 10000 counts/revolution):
# 90 degrees = 90 × (10000/360) = 2500 counts
# Try: ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 2500.0}"
```

## Automated Script

Run the complete sequence automatically:

```bash
cd /home/qiaoqiaochen/appdata/canros/src
./enable_and_move_trolley.sh
```

## Check for Errors

If motor still doesn't move after enabling:

```bash
# Check error register
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 1001, subindex: 0}"

# If there's an error, try to recover
ros2 service call /trolley_joint/recover std_srvs/srv/Trigger
```

## Understanding Statusword Bits

- **Bit 0** (0x0001): Ready to switch on
- **Bit 1** (0x0002): Switched on
- **Bit 2** (0x0004): Operation enabled ← **This must be set!**
- **Bit 3** (0x0008): Fault
- **Bit 4** (0x0010): Voltage enabled
- **Bit 5** (0x0020): Quick stop
- **Bit 6** (0x0040): Switch on disabled

**Good values:**
- `0x0027` = 0b00100111 = Ready + Switched on + Operation enabled + Quick stop
- `0x001F` = 0b00011111 = All good states

## Most Likely Solution

Just enable the motor:

```bash
ros2 service call /trolley_joint/enable std_srvs/srv/Trigger
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"
```

Then monitor:
```bash
ros2 topic echo /trolley_joint/joint_states
```

You should see the position value change!



