# Fix: Motor Not Moving - Interpolated Position Mode

## Root Cause

The DSY-C motor driver **does not support Profile Position Mode (mode 1)**, which was configured in `bus.yml`. This is why:
- The `/trolley_joint/position_mode` service always returned `success=False`
- Target position commands were being sent but motor never moved
- The motor stayed at position 0.0

## Supported Modes

According to the motor configuration, DSY-C supports:
- ✅ **Mode 7: Interpolated Position Mode** (now configured)
- ✅ Mode 3: Profile Velocity Mode
- ✅ Mode 4: Profile Torque Mode  
- ✅ Mode 6: Homing Mode
- ❌ Mode 1: Profile Position Mode (NOT supported)

## Changes Made

### 1. Updated `bus.yml` Configuration

Changed all three joints from mode 1 to mode 7:

**Before:**
```yaml
position_mode: 1  # Profile Position - NOT SUPPORTED!
sdo:
  - {index: 0x6060, sub_index: 0, value: 1}
```

**After:**
```yaml
position_mode: 7  # Interpolated Position - SUPPORTED!
sdo:
  # Set interpolation time (required for Interpolated Position Mode)
  - {index: 0x60C2, sub_index: 1, value: 50}  # Interpolation time period
  - {index: 0x60C2, sub_index: 2, value: -3}  # Base 10^-3s
  - {index: 0x6081, sub_index: 0, value: 1000}  # Profile velocity
  - {index: 0x6083, sub_index: 0, value: 2000}  # Profile acceleration
  - {index: 0x6060, sub_index: 0, value: 7}  # Set mode to 7
```

### 2. Interpolation Time Configuration

Interpolated Position Mode REQUIRES interpolation time to be set:
- **0x60C2 sub 1 = 50**: Interpolation period
- **0x60C2 sub 2 = -3**: Time base (10^-3 seconds = milliseconds)
- **Result**: 50ms interpolation period

### 3. PDO Configuration (from image requirements)

**TPDO (Transmit PDO):**
- TPDO 1: Statusword + Mode of operation display
- TPDO 2: Position actual + Velocity actual
- Transmission: 0x01 (synchronous)

**RPDO (Receive PDO):**
- RPDO 1: Controlword + Mode of operation + Target position
- Transmission: 0x01 (synchronous)

## How to Use

### After Restarting the System

**1. Restart ROS2 with new configuration:**
```bash
# Stop current system (Ctrl+C)
ros2 launch tower_crane hardware_bringup_real.launch.py
```

**2. Use the new movement script:**
```bash
cd /home/qiaoqiaochen/appdata/canros/src
./move_trolley_interpolated_mode.sh
```

**3. Or run commands manually:**
```bash
# Initialize
ros2 service call /trolley_joint/init std_srvs/srv/Trigger

# Set Interpolated Position Mode (mode 7)
ros2 service call /trolley_joint/interpolated_position_mode std_srvs/srv/Trigger

# Enable motor
ros2 service call /trolley_joint/enable std_srvs/srv/Trigger

# Send target (32.768 ≈ 90 degrees rotation)
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 32.768}"
```

## Understanding Target Values

The `target` value is scaled by the driver:

**Formula:**
```
Encoder counts = target × scale_pos_to_dev_ (default: 1000)
Motor degrees = (encoder_counts / 131072) × 360°
```

**Common Target Values:**
| Target Value | Encoder Counts | Motor Rotation |
|--------------|----------------|----------------|
| 1.31         | 1,310          | ~3.6°          |
| 6.55         | 6,550          | ~18°           |
| 32.768       | 32,768         | ~90°           |
| 65.536       | 65,536         | ~180°          |
| 131.072      | 131,072        | ~360° (1 rev)  |

## Key Differences: Mode 1 vs Mode 7

| Feature | Profile Position (Mode 1) | Interpolated Position (Mode 7) |
|---------|---------------------------|-------------------------------|
| Support | ❌ Not on DSY-C | ✅ Supported |
| Interpolation Time | Not required | ✅ Required (0x60C2) |
| Trajectory | Motor generates | Host provides interpolated points |
| Smoothness | Built-in profile | Based on interpolation period |

## Verification

After applying changes, verify:

```bash
# Check mode is set to 7
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x6061, subindex: 0}"
# Should return: data: 7

# Check interpolation time is set
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x60C2, subindex: 1}"
# Should return: data: 50
```

## Troubleshooting

**If motor still doesn't move:**

1. Check the mode display (should be 7):
   ```bash
   ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x6061, subindex: 0}"
   ```

2. Check statusword (should have bit 2 set = operation enabled):
   ```bash
   ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x6041, subindex: 0}"
   ```

3. Try a smaller target value first:
   ```bash
   ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 1.31}"
   ```

4. Check for errors in the ROS2 logs:
   ```bash
   ros2 topic echo /rosout
   ```

## Files Modified

- ✅ `crane/tower_crane/config/robot_control/bus.yml` - Changed mode 1→7 for all joints
- ✅ `move_trolley_interpolated_mode.sh` - New script using correct mode
- ✅ `INTERPOLATED_MODE_FIX.md` - This documentation

## Next Steps

1. Restart your ROS2 system to apply the new configuration
2. Run `./move_trolley_interpolated_mode.sh` to test movement
3. If successful, the motor should now respond to position commands!


