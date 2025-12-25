# Slewing Motor Activation Timeout Fix

## Problem

The slewing_joint motor (node 3) fails to activate during system startup:

```
[canopen_402_driver]: Transition timed out. target=5 next=3 current=2 statusword=0x0250
[ERROR] Failed to activate 'slewing_joint'
terminate called after throwing an instance of 'std::runtime_error'
  what():  Failed to set the initial state of the component : canopen_ros2_control/CanopenSystem to active
```

**Statusword 0x0250** = "Switch on disabled" - motor is stuck in state 2 and cannot transition to state 3 (Ready to switch on).

## Diagnosis

### What Works
1. ✅ Slewing motor responds to NMT commands (state: Operational)
2. ✅ Slewing motor responds to basic SDO requests (device type 0x1000, error register 0x1001)
3. ✅ Slewing motor sends TPDO messages (0x283) continuously
4. ✅ TPDO mapping is correct (statusword + position)
5. ✅ Trolley motor (node 2) initializes successfully with same configuration

### What Doesn't Work
1. ❌ Slewing motor does NOT respond to CiA402 SDO requests (0x6041 statusword, 0x6060 mode)
2. ❌ SDO reads for CiA402 objects return error 0x06020000 ("Object does not exist")
3. ❌ RPDO configuration reads return error 0x08000000 ("General error")
4. ❌ Motor times out after 20 seconds trying to transition from state 2 to state 3

### CAN Traffic Analysis

**TPDO Data from Node 3 (0x283):**
```
can0  283   [8]  00 00 00 00 01 00 00 00
can0  283   [8]  00 00 00 00 00 00 00 00
can0  283   [8]  00 00 00 00 FF FF FF FF
```
- First 2 bytes should be statusword, but values (0x0000, 0x0001, 0x00FF) don't match CiA402 format
- Bytes 3-6 are position (all zeros)

**SDO Requests to Node 3:**
```
Request:  can0  603   [8]  40 41 16 00 00 00 00 00  (Read 0x6041 statusword)
Response: can0  583   [8]  80 41 16 00 00 00 02 06  (Error: Object does not exist)

Request:  can0  603   [8]  40 60 16 00 00 00 00 00  (Read 0x6060 mode)
Response: can0  583   [8]  80 60 16 00 00 00 02 06  (Error: Object does not exist)

Request:  can0  603   [8]  40 00 10 00 00 00 00 00  (Read 0x1000 device type)
Response: can0  583   [8]  43 00 10 00 92 01 02 00  (Success: 0x00020192 = CiA402 drive)
```

## Root Causes (Suspected)

### 1. **Firmware/Configuration Issue**
The motor identifies as a CiA402 drive (device type 0x00020192) but doesn't have CiA402 objects in its object dictionary. This suggests:
- Incorrect firmware version
- Motor not fully initialized
- Object dictionary corruption
- Wrong EDS file being used

### 2. **Hardware Issue**
- Emergency stop engaged
- Brake not released
- Power supply problem
- Enable signal not connected
- Motor controller fault

### 3. **Communication Issue**
- CAN termination problem (but other motors work fine)
- Node ID conflict (unlikely, NMT works)
- Baud rate mismatch (unlikely, basic communication works)

## Fixes Applied

### 1. Increased Timeouts in `bus.yml`

Changed for all motors (hook, trolley, slewing):

```yaml
# Before:
sdo_timeout_ms: 2000
boot_timeout_ms: 30000
polling: false

# After:
sdo_timeout_ms: 5000      # Increased from 2000ms
boot_timeout_ms: 60000    # Increased from 30000ms
polling: true             # Changed from false
```

**Rationale:**
- Longer timeouts give the motor more time to respond
- Polling mode uses SDO reads instead of relying only on TPDOs
- May help if motor has slow response times

### 2. Verify Motor Configuration

Need to check:
- Is the motor physically enabled? (check for enable switch/jumper)
- Is the brake released? (check brake voltage)
- Is the emergency stop active? (check E-STOP circuit)
- Is the motor in fault state? (check LED indicators)

## Testing Steps

### 1. Check Physical Hardware

```bash
# Check if motor has power and is responding
candump can0 | grep 283  # Should see TPDO messages

# Check NMT state
cansend can0 703#00
# Should see response: can0  703   [1]  05  (Operational)

# Check error register
timeout 1 candump can0 | grep 583 &
cansend can0 603#4001100000000000
# Should see: can0  583   [8]  4F 01 10 00 00 00 00 00  (No errors)
```

### 2. Try Manual Enable Sequence

```bash
# 1. Fault reset
cansend can0 603#2B40600080000000
sleep 0.5

# 2. Shutdown (state 2 -> 3)
cansend can0 603#2B40600006000000
sleep 0.5

# 3. Switch on (state 3 -> 4)
cansend can0 603#2B40600007000000
sleep 0.5

# 4. Enable operation (state 4 -> 5)
cansend can0 603#2B4060000F000000
sleep 0.5

# 5. Read final statusword
timeout 1 candump can0 | grep 583 &
cansend can0 603#40410600
# Should see statusword 0x0027 (Operation enabled)
```

### 3. Rebuild and Test

```bash
cd /home/qiaoqiaochen/appdata/canros
colcon build --packages-select tower_crane
source install/setup.bash

# Launch system
ros2 launch tower_crane hardware_bringup_real.launch.py can_interface_name:=can0
```

### 4. Monitor Initialization

```bash
# In another terminal, monitor CAN traffic
./debug_slewing_init.sh
```

## Expected Outcomes

### If Fix Works:
```
[canopen_402_driver]: Init: Read State
[canopen_402_driver]: Init: Enable
[resource_manager]: Successful 'activate' of hardware 'canopen_ros2_control/CanopenSystem'
[spawner_joint_state_broadcaster]: Loaded joint_state_broadcaster
```

### If Still Fails:
- Check motor documentation for special initialization requirements
- Contact motor manufacturer for support
- Consider replacing motor controller
- Temporarily disable slewing_joint in configuration to test other motors

## Workaround: Disable Slewing Motor

If the motor cannot be fixed immediately, you can temporarily disable it:

### Option 1: Comment out in `bus.yml`
```yaml
# slewing_joint:
#   node_id: 3
#   ...
```

### Option 2: Remove from `tower_crane_ros2_control.yaml`
```yaml
joint_state_broadcaster:
  ros__parameters:
    joints:
      # - slewing_joint  # Commented out
      - trolley_joint
      - hook_joint

forward_position_controller:
  ros__parameters:
    joints:
      # - slewing_joint  # Commented out
      - trolley_joint
      - hook_joint
```

## Next Steps

1. ✅ Applied timeout increases and polling mode
2. ⏳ Test with new configuration
3. ⏳ If still fails, check physical hardware
4. ⏳ If hardware OK, investigate firmware/EDS file
5. ⏳ Consider contacting motor manufacturer

## Additional Resources

- CiA402 State Machine: See `MOTOR_ACTIVATION_FIX.md`
- Statusword decoding: See `MOTOR_NOT_MOVING_FIX.md`
- CANopen troubleshooting: See `CANOPEN_TROUBLESHOOTING.md`


