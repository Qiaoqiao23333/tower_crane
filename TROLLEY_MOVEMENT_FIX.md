# Fix for Trolley Movement Error Code -5 (GOAL_TOLERANCE_VIOLATED)

## Problem
When running `./move_trolley_example.py`, the trajectory action was accepted but failed with error code `-5` (GOAL_TOLERANCE_VIOLATED). The joints remained at position 0.0 and did not move.

## Root Cause
The CANopen motors were **not initialized and enabled** before sending movement commands. Even though the trajectory controller accepted the goal, the underlying motors couldn't execute the movement because they were not in the proper operational state.

## Solution Applied

### Changes to `move_trolley_example.py`

#### 1. Added Motor Initialization Functions
```python
def initialize_motors(self):
    """Initialize all motors before movement"""
    # Calls /[joint]/init service for each joint
    
def set_position_mode(self):
    """Set all motors to position mode"""
    # Calls /[joint]/position_mode service for each joint
    
def enable_motors(self):
    """Enable all motors for movement"""
    # Calls /[joint]/enable service for each joint
```

#### 2. Fixed Joint Order
Changed from `['hook_joint', 'trolley_joint', 'slewing_joint']` to `['slewing_joint', 'trolley_joint', 'hook_joint']` to match the controller configuration in `tower_crane_ros2_control.yaml`.

#### 3. Adjusted Target Position
- Changed from `0.5m` (too large, may exceed limits) to `0.09m` (90 degrees motor rotation)
- This is a safer, more achievable initial target

#### 4. Increased Timeout and Tolerance
- Increased movement time from 10 seconds to 15 seconds
- Added goal time tolerance of 5 extra seconds
- This gives motors more time to reach the target position

#### 5. Enhanced Error Reporting
- Added descriptive error messages for each error code
- Special handling for error code -5 with troubleshooting suggestions
- Added feedback during movement to show progress

#### 6. Updated Main Flow
The script now follows a proper 4-step initialization sequence:
1. **Initialize motors** - Wake up the CANopen devices
2. **Set position mode** - Configure for position control (mode 1)
3. **Enable motors** - Put motors in operational state
4. **Send movement command** - Execute the trajectory

## How to Use

Simply run the fixed script:

```bash
./move_trolley_example.py
```

The script will automatically:
1. Initialize all three motors (slewing, trolley, hook)
2. Set them to position control mode
3. Enable them for operation
4. Move the trolley joint 0.09m (90 degrees of motor rotation)
5. Display progress and results

## Expected Output

```
[INFO] [trolley_mover]: ============================================================
[INFO] [trolley_mover]: STEP 1: Initializing motors...
[INFO] [trolley_mover]: ============================================================
[INFO] [trolley_mover]: ✓ slewing_joint initialized
[INFO] [trolley_mover]: ✓ trolley_joint initialized
[INFO] [trolley_mover]: ✓ hook_joint initialized
[INFO] [trolley_mover]: ============================================================
[INFO] [trolley_mover]: STEP 2: Setting position mode...
[INFO] [trolley_mover]: ============================================================
[INFO] [trolley_mover]: ✓ slewing_joint in position mode
[INFO] [trolley_mover]: ✓ trolley_joint in position mode
[INFO] [trolley_mover]: ✓ hook_joint in position mode
[INFO] [trolley_mover]: ============================================================
[INFO] [trolley_mover]: STEP 3: Enabling motors...
[INFO] [trolley_mover]: ============================================================
[INFO] [trolley_mover]: ✓ slewing_joint enabled
[INFO] [trolley_mover]: ✓ trolley_joint enabled
[INFO] [trolley_mover]: ✓ hook_joint enabled
[INFO] [trolley_mover]: ============================================================
[INFO] [trolley_mover]: STEP 4: Moving trolley 0.09m (90 degrees motor rotation)...
[INFO] [trolley_mover]: ============================================================
[INFO] [trolley_mover]: Sending goal: trolley_joint = 0.09 m (slewing=0.0, hook=0.0)
[INFO] [trolley_mover]: Goal accepted!
[INFO] [trolley_mover]: Moving... trolley at 0.0234m
[INFO] [trolley_mover]: Moving... trolley at 0.0567m
[INFO] [trolley_mover]: Moving... trolley at 0.0890m
[INFO] [trolley_mover]: ============================================================
[INFO] [trolley_mover]: ✓✓✓ Movement completed successfully! ✓✓✓
[INFO] [trolley_mover]: ============================================================
```

## Verify Movement

After running the script, check the joint states:

```bash
ros2 topic echo /joint_states --once
```

You should see the `trolley_joint` position at approximately `0.09` (or close to it).

## Troubleshooting

### If services are not available:
- Make sure the hardware system is running: check if controllers are active with `ros2 control list_controllers`
- Verify CANopen communication is working

### If motors still don't move after initialization:
1. Check motor status word:
   ```bash
   ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}"
   ```
   Should return `0x0027` or `0x001F` (operation enabled)

2. Check for faults:
   ```bash
   ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 1001, subindex: 0}"
   ```
   Should return `0` (no error)

3. Try manual recovery:
   ```bash
   ros2 service call /trolley_joint/recover std_srvs/srv/Trigger
   ```

## Key Takeaway

**CANopen motors must be initialized and enabled before they will respond to movement commands.** The trajectory controller can accept goals, but if the underlying hardware isn't ready, the movement will fail with GOAL_TOLERANCE_VIOLATED.

Always follow the sequence: Init → Mode → Enable → Move


