# Complete Fix: "Mode is not supported" Error

## Error Summary

```
[INFO] [slewing_joint]: Switching to 'slewing_joint/position' command mode with CIA402 operation mode '1'
[INFO] [canopen_402_driver]: Mode is not supported.
[ERROR] [resource_manager]: Component 'canopen_ros2_control/CanopenSystem' could not perform switch
[ERROR] [controller_manager]: Error while performing mode switch.
```

## What Happened (Technical Deep Dive)

### 1. The Initialization Problem

When ros2_control activates the hardware, it calls `RobotSystem::on_activate()`. However, this function has **commented out** all motor initialization:

```cpp
// third_party/ros2_canopen/canopen_ros2_control/src/robot_system.cpp:140-157
hardware_interface::CallbackReturn RobotSystem::on_activate(...)
{
  // NOTE: init_motor() is commented out because it blocks controller_manager startup.
  // for (auto & data : robot_motor_data_)
  // {
  //   if (!data.driver->init_motor())  // ← THIS IS NEVER CALLED!
  //   {
  //     ...
  //   }
  // }
  return CallbackReturn::SUCCESS;
}
```

### 2. Why Init Matters

The `init_motor()` call chain is critical:
```
init_motor() 
  → handleInit() 
    → mode_allocators_ lambdas executed
      → isModeSupportedByDevice(mode) checks 0x6502
        → IF supported: registerMode(mode, ModeSharedPtr) 
          → modes_[mode] = ModeSharedPtr
```

**Without this chain**:
- `modes_` map remains **empty**
- `allocMode(1)` returns **null**  
- Driver says "Mode is not supported"

### 3. Why Init Was Disabled

From the code comments:
- `handleInit()` tries to enable each motor
- Can block for **60 seconds per motor**  
- With 3 motors = **180 seconds blocking time**
- During this time, controller_manager **cannot respond to service requests**
- Spawners timeout and fail

### 4. The Mode Check Logic

```cpp
// third_party/ros2_canopen/canopen_402_driver/src/motor.cpp:63-76
ModeSharedPtr Motor402::allocMode(uint16_t mode)
{
  ModeSharedPtr res;
  if (isModeSupportedByDevice(mode))  // Reads 0x6502 from device
  {
    std::unordered_map<uint16_t, ModeSharedPtr>::iterator it = modes_.find(mode);
    if (it != modes_.end())  // ← ALWAYS fails because modes_ is empty!
    {
      res = it->second;
    }
  }
  return res;  // Returns null → "Mode is not supported"
}
```

### 5. Device Capability Check

Object **0x6502** (Supported drive modes) in DSY-C.EDS:
```
DefaultValue=109  (0b01101101)
```

Decoding:
- **Bit 0 (mode 1)**: ✅ Profile Position Mode - SUPPORTED
- Bit 1 (mode 2): ❌ Velocity Mode
- **Bit 2 (mode 3)**: ✅ Profile Velocity Mode - SUPPORTED  
- Bit 3 (mode 4): ❌ Profile Torque Mode
- **Bit 5 (mode 6)**: ✅ Homing Mode - SUPPORTED
- **Bit 6 (mode 7)**: ✅ Interpolated Position Mode - SUPPORTED

**The hardware DOES support mode 1**, but the software never registered it!

## The Solution

### Option 1: Restart with Pre-Enable (RECOMMENDED)

Stop the current system (Ctrl+C) and restart:

```bash
sudo /home/qiaoqiaochen/appdata/canros/src/start_tower_crane.sh pre_enable
```

This triggers the launch file's `pre_enable_drives` logic:

```python
# hardware_bringup_real.launch.py:161-189
pre_enable_step = ExecuteProcess(
    condition=IfCondition(LaunchConfiguration("pre_enable_drives")),
    cmd=["/bin/bash", "-lc", 
        # Pre-select Profile Position mode and enable drives 1/2/3
        # via direct CAN commands BEFORE controller_manager starts
    ]
)
```

#### What Pre-Enable Does

1. **Sets operation mode** via SDO:
   ```bash
   cansend can0 603#2F60600001000000  # Node 1: Set 0x6060=1
   cansend can0 604#2F60600001000000  # Node 2: Set 0x6060=1
   cansend can0 605#2F60600001000000  # Node 3: Set 0x6060=1
   ```

2. **CIA402 state machine sequence**:
   ```bash
   # Shutdown (controlword = 0x06)
   cansend can0 603#2B40600006000000
   
   # Switch On (controlword = 0x07)
   cansend can0 603#2B40600007000000
   
   # Enable Operation (controlword = 0x0F)
   cansend can0 603#2B4060000F000000
   ```

3. **Verifies statusword** shows 0x0027 (Operation Enabled):
   ```bash
   # Read statusword 0x6041
   cansend can0 603#4041600000000000
   # Expected response: 581#4B416000XX27XXXX
   ```

4. **THEN** starts controller_manager (which no longer needs to block)

### Option 2: Uncomment Initialization (Alternative)

Edit `robot_system.cpp`:

```cpp
hardware_interface::CallbackReturn RobotSystem::on_activate(...)
{
  // Uncomment these lines:
  for (auto & data : robot_motor_data_)
  {
    if (!data.driver->init_motor())
    {
      RCLCPP_ERROR(robot_system_logger, "Failed to activate '%s'", data.joint_name.c_str());
      return CallbackReturn::FAILURE;
    }
  }
  return CallbackReturn::SUCCESS;
}
```

**Trade-off**: Controller spawners must increase their timeout to >180 seconds.

### Option 3: Manual CAN Commands (Not Recommended)

While the system is running:

```bash
cd /home/qiaoqiaochen/appdata/canros/src
sudo ./manually_enable_motors.sh
```

**Limitation**: This enables motors at hardware level, but modes still aren't registered in ros2_control. Controllers will still fail.

## Expected Results After Fix

### 1. Pre-Enable Output

```
[pre_enable_drives] enabling CiA402 nodes 1/2/3 on can0
[pre_enable_drives] node 1 : set mode 0x6060=1
[pre_enable_drives] node 1 attempt 1 statusword: 581#4B416000XX27XXXX
[pre_enable_drives] node 2 attempt 1 statusword: 582#4B416000XX27XXXX
[pre_enable_drives] node 3 attempt 1 statusword: 583#4B416000XX27XXXX
```

### 2. Clean Controller Activation

```
[INFO] [controller_manager]: Loading controller 'forward_position_controller'
[INFO] [controller_manager]: Configuring controller 'forward_position_controller'
[INFO] [slewing_joint]: Switching to 'slewing_joint/position' command mode with CIA402 operation mode '1'
✅ NO ERROR - Mode switch succeeds!
[INFO] [spawner]: Configured and activated forward_position_controller
```

### 3. Joint States Publishing

```bash
ros2 topic echo /joint_states --once
```

Output:
```yaml
header:
  stamp:
    sec: ...
  frame_id: ''
name: [hook_joint, trolley_joint, slewing_joint]
position: [0.0, 0.0, 0.0]
velocity: []
effort: []
```

## Why Services Don't Work

You might have tried:

```bash
ros2 service call /trolley_joint/init std_srvs/srv/Trigger
# → Hangs forever
```

**Why it hangs**:

```cpp
// node_canopen_402_driver_impl.hpp:436-445
void handle_init(...)
{
  if (this->activated_.load())  // True (hardware is activated)
  {
    bool temp = motor_->handleInit();  // ← Blocks for up to 60 seconds!
    response->success = temp;
  }
}
```

The service **exists** but **blocks** trying to communicate with the motor, which may be in an unexpected state. Pre-enablement via CAN is faster and more reliable.

## Verification Checklist

After restarting with `pre_enable`:

- [ ] Pre-enable messages show all 3 motors reach statusword 0x...27
- [ ] Controller spawners complete without timeout
- [ ] No "Mode is not supported" error
- [ ] `/joint_states` topic is publishing
- [ ] Can send trajectory commands:
  ```bash
  ros2 topic pub /forward_position_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "..." --once
  ```

## Files Modified

1. ✅ `/home/qiaoqiaochen/appdata/canros/src/start_tower_crane.sh`
   - Added support for `pre_enable` argument
   - Usage: `sudo start_tower_crane.sh pre_enable`

2. ✅ `/home/qiaoqiaochen/appdata/canros/src/enable_motors_after_startup.sh`
   - Added ROS2 environment sourcing
   - (Still not the recommended approach - use pre_enable instead)

## Summary

| Issue | Root Cause | Solution |
|-------|------------|----------|
| "Mode is not supported" | `modes_` map is empty | Pre-enable motors via CAN |
| Why empty? | `init_motor()` never called | Before controller_manager starts |
| Why not called? | Commented out to prevent blocking | Use `pre_enable_drives:=true` |
| Services hang? | Try to call blocking `handleInit()` | Don't use services, use pre_enable |

**Bottom line**: Restart with `sudo start_tower_crane.sh pre_enable`

