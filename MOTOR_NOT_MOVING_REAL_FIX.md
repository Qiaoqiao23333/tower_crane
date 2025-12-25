# Real Fix: Motor Not Moving Despite "Success" Message

## The Problem

The trajectory controller reported **"Movement completed successfully"** but the trolley motor stayed at `0.0m` and never moved.

### Why Did It Report Success?

The controller config had goal tolerance set to `0.1m`:
- Target position: `0.09m`
- Actual position: `0.0m`  
- Error: `0.09m` (within the `0.1m` tolerance)
- Result: **FALSE SUCCESS** ✗

## Two-Part Solution

### Part 1: Fix False Success (Immediate)

**Updated `/crane/tower_crane/config/tower_crane_ros2_control.yaml`:**

```yaml
trolley_joint:
  trajectory: 0.5
  goal: 0.01  # Changed from 0.1 to 0.01 (10cm → 1cm)
```

Now if the motor doesn't move, it will correctly report failure because `0.09m` error exceeds the `0.01m` tolerance.

**You need to restart the controllers after this change:**
```bash
ros2 control load_controller forward_position_controller --set-state active
```

Or restart your entire hardware system.

### Part 2: Diagnose Why Motor Isn't Moving

**Run the new diagnostic script:**
```bash
./diagnose_and_move_trolley.py
```

This script will:
1. **Check motor statusword** (0x6041) - Is it actually enabled?
2. **Check operation mode** (0x6061) - Is it in Profile Position Mode (1)?
3. **Check current position** (0x6064) - Where is it now?
4. **Check target position** (0x607A) - What's the last commanded position?
5. **Initialize → Enable → Move** using direct CANopen commands
6. **Monitor actual movement** over 10 seconds
7. **Report if motor moved or not**

## What the Diagnostic Will Tell You

### Scenario A: Motor Is Not Properly Enabled

```
Statusword: 0x0021
⚠ Motor is READY TO SWITCH ON
✗✗✗ Motor is not in operation enabled state!
```

**Fix:** The enable sequence isn't working. Check:
- CANopen communication errors
- Motor driver faults
- Power supply issues

### Scenario B: Motor Is Enabled But Wrong Mode

```
Statusword: 0x0027
✓✓✓ Motor is OPERATION ENABLED
Current mode: No mode (0)
⚠⚠⚠ Motor may not be in correct mode
```

**Fix:** Position mode not set. The motor needs to be in Profile Position Mode (1).

### Scenario C: Motor Enabled, Right Mode, But Not Moving

```
✓✓✓ Motor is OPERATION ENABLED
✓ In Profile Position Mode
Target sent successfully
Position change: 0 encoder units
✗✗✗ MOTOR DID NOT MOVE!
```

**Possible causes:**
1. **Brake engaged** - Physical brake on motor shaft
2. **CANopen PDO not configured** - Commands not reaching motor
3. **Hardware failure** - Motor or driver issue
4. **Encoder not connected** - Position feedback missing
5. **Motor tuning** - PID gains incorrect

## How to Use Both Fixes

### Step 1: Update Tolerance (Prevent False Success)
```bash
# Edit the file (already done)
nano /crane/tower_crane/config/tower_crane_ros2_control.yaml

# Restart controllers (required!)
ros2 control load_controller forward_position_controller --set-state active
```

### Step 2: Run Diagnostic
```bash
./diagnose_and_move_trolley.py
```

Watch the output carefully. It will tell you exactly what state the motor is in and whether it actually moves when commanded.

### Step 3: Based on Diagnostic Results

**If motor moved with diagnostic:**
- The problem is with the trajectory controller, not the motor
- Check ros2_control hardware interface configuration
- Verify command interface mapping

**If motor didn't move with diagnostic:**
- Check the diagnostic output for specific error
- Look at statusword and operation mode
- May need to check hardware/wiring

## Understanding Direct CANopen vs Trajectory Controller

### Direct CANopen Command (what diagnostic uses)
```bash
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"
```
- Goes directly to motor via CANopen SDO/PDO
- Bypasses ros2_control entirely
- Good for testing if motor works at all

### Trajectory Controller (what move_trolley_example.py uses)
```bash
ros2 action send_goal /forward_position_controller/follow_joint_trajectory ...
```
- Goes through: Action → Controller → ros2_control → Hardware Interface → CANopen
- More complex chain, more points of failure
- Required for coordinated multi-joint movement

## If Both Methods Fail

If even the direct CANopen command doesn't move the motor:

1. **Check CAN bus:**
   ```bash
   candump can0
   ```
   You should see traffic when sending commands.

2. **Check node status:**
   ```bash
   ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x6041, subindex: 0}"
   ```
   Should return without error.

3. **Check error register:**
   ```bash
   ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x1001, subindex: 0}"
   ```
   Should return `0` (no error).

4. **Try recover:**
   ```bash
   ros2 service call /trolley_joint/recover std_srvs/srv/Trigger
   ```

## Expected Output from Diagnostic (Success Case)

```
=================================================================
TROLLEY MOTOR DIAGNOSTIC AND MOVEMENT TEST
=================================================================

--- PHASE 1: Initial State Check ---
Reading Statusword (0x6041)...
  Statusword: 0x0027
  ✓✓✓ Motor is OPERATION ENABLED
Reading Operation Mode Display (0x6061)...
  Current mode: Profile Position Mode
  ✓ In Profile Position Mode
Reading Position Actual Value (0x6064)...
  Current position: 0 encoder units

--- PHASE 2: Initialize and Enable ---
Initializing motor...
  ✓ Initialized
Setting position mode...
  Position mode response: (no message)
Enabling motor...
  ✓ Enabled

--- PHASE 3: State After Enabling ---
  ✓✓✓ Motor is OPERATION ENABLED
  ✓ In Profile Position Mode

--- PHASE 4: Direct CANopen Movement (90 degrees) ---
Sending target position: 90.0...
  ✓ Target sent successfully

--- PHASE 5: Monitoring Movement ---
  Step 1: Position=12, Target=90, Error=78
  Step 2: Position=34, Target=90, Error=56
  Step 3: Position=67, Target=90, Error=23
  Step 4: Position=87, Target=90, Error=3

✓✓✓ Movement completed!

--- FINAL STATE ---
Position change: 87 encoder units
✓✓✓ MOTOR MOVED 87 encoder units!
```

## Next Steps

1. Run `./diagnose_and_move_trolley.py`
2. Share the output
3. Based on the diagnostic, we'll know:
   - Is the motor hardware working?
   - Is CANopen communication working?
   - Is the issue in ros2_control layer?
   - Is the issue in trajectory controller?

Then we can apply the appropriate fix!


