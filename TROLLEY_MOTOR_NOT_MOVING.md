# Troubleshooting: Trolley Motor Not Moving

## Problem
Service call succeeds (`success=True`) but motor doesn't move.

## Common Causes & Solutions

### 1. Motor Not Enabled ⭐ MOST COMMON

The motor might be initialized but not enabled. Check and enable:

```bash
# Check current state
ros2 topic echo /trolley_joint/nmt_state --once

# Enable the motor
ros2 service call /trolley_joint/enable std_srvs/srv/Trigger
```

### 2. Motor Not Initialized

The motor needs to be initialized before it can move:

```bash
# Initialize the motor
ros2 service call /trolley_joint/init std_srvs/srv/Trigger

# Then enable it
ros2 service call /trolley_joint/enable std_srvs/srv/Trigger
```

### 3. Wrong Operation Mode

The motor might not be in position mode:

```bash
# Set to position mode
ros2 service call /trolley_joint/position_mode std_srvs/srv/Trigger

# Or use interpolated position mode
ros2 service call /trolley_joint/interpolated_position_mode std_srvs/srv/Trigger
```

### 4. Motor in Fault/Error State

Check for errors and recover:

```bash
# Check error register
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 1001, subindex: 0}"

# If there's an error, try to recover
ros2 service call /trolley_joint/recover std_srvs/srv/Trigger
```

### 5. Wrong Units/Scale

The value `90.0` might be in encoder counts, not degrees. Check the actual position:

```bash
# Check current position
ros2 topic echo /trolley_joint/joint_states --once

# Check position actual value (0x6064) in encoder units
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6064, subindex: 0}"
```

If the motor uses encoder counts (e.g., 10000 counts per revolution), then:
- 90 degrees = 90 × (encoder_resolution / 360) counts
- For 10000 counts/rev: 90° = 2500 counts

### 6. Complete Initialization Sequence

Try this complete sequence:

```bash
# Step 1: Initialize
ros2 service call /trolley_joint/init std_srvs/srv/Trigger

# Step 2: Set position mode
ros2 service call /trolley_joint/position_mode std_srvs/srv/Trigger

# Step 3: Enable
ros2 service call /trolley_joint/enable std_srvs/srv/Trigger

# Step 4: Check status
ros2 topic echo /trolley_joint/joint_states --once

# Step 5: Send target
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"
```

## Diagnostic Commands

Run the diagnostic script:
```bash
./diagnose_trolley_motor.sh
```

Or manually check:

```bash
# 1. Check NMT state (should be OPERATIONAL)
ros2 topic echo /trolley_joint/nmt_state --once

# 2. Check Statusword (0x6041) - bit 0 should be 1 (ready to switch on)
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}"

# 3. Check operation mode display (0x6061) - should be 1 for position mode
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6061, subindex: 0}"

# 4. Check error register (0x1001) - should be 0 (no errors)
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 1001, subindex: 0}"
```

## Understanding Statusword (0x6041)

The Statusword bits indicate motor state:
- Bit 0: Ready to switch on
- Bit 1: Switched on
- Bit 2: Operation enabled
- Bit 3: Fault
- Bit 4: Voltage enabled
- Bit 5: Quick stop
- Bit 6: Switch on disabled

Common values:
- `0x0027` = Operation enabled (bits 0,1,2,5 set)
- `0x0006` = Ready to switch on
- `0x0007` = Switched on
- `0x001F` = Operation enabled (all good)

## Understanding Operation Mode (0x6061)

- `1` = Profile Position Mode
- `3` = Profile Velocity Mode
- `6` = Homing Mode
- `8` = Cyclic Position Mode

## Alternative: Use Trajectory Controller

If direct service calls don't work, try the trajectory controller:

```bash
# This goes through ROS2 control which handles initialization
ros2 topic pub --once /forward_position_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
{
  joint_names: ['slewing_joint', 'trolley_joint', 'hook_joint'],
  points: [
    {
      positions: [0.0, 0.09, 0.0],
      time_from_start: {sec: 5, nanosec: 0}
    }
  ]
}"
```

## Check Motor Configuration

Verify the motor is configured correctly in `bus.yml`:
- `position_mode: 1` (Profile Position Mode)
- Motor is enabled in the boot sequence
- SDO timeout is sufficient (2000ms)

## Next Steps

1. Run diagnostic: `./diagnose_trolley_motor.sh`
2. Check if motor is enabled
3. Verify operation mode
4. Check for errors
5. Try complete initialization sequence
6. If still not working, check encoder units/scale



