# MoveIt Launch File Simplification

## Changes Made

### ✅ **Removed Redundant Parameter**

**BEFORE:**
```bash
# Had to specify both parameters (redundant!)
ros2 launch tower_crane_moveit_config moveit_planning_execution.launch.py \
    use_real_hardware:=true can_interface_name:=can0
```

**AFTER:**
```bash
# Auto-detects hardware type from CAN interface name
ros2 launch tower_crane_moveit_config moveit_planning_execution.launch.py \
    can_interface_name:=can0
```

### 🔧 **How It Works Now**

The launch file automatically detects hardware type based on the CAN interface name:

| CAN Interface | Hardware Type | Description |
|---------------|---------------|-------------|
| `can0` | **Real Hardware** | Physical CANopen motors |
| `can1` | **Real Hardware** | Physical CANopen motors |
| `vcan0` | **Mock Hardware** | Simulated motors |
| `vcan1` | **Mock Hardware** | Simulated motors |

**Logic:**
```python
use_real_hardware = can_interface_name_str.startswith("can") and not can_interface_name_str.startswith("vcan")
```

### 📋 **Updated Launch Arguments**

**Removed:**
- ❌ `use_real_hardware` - No longer needed!

**Kept:**
- ✅ `can_interface_name` - Default: `vcan0`
- ✅ `use_rviz` - Default: `true`

## Usage Examples

### 1️⃣ **Real Hardware with RViz** (Most Common)
```bash
ros2 launch tower_crane_moveit_config moveit_planning_execution.launch.py \
    can_interface_name:=can0
```
- ✅ Launches real hardware (auto-detected)
- ✅ RViz opens automatically
- ✅ Full MoveIt planning and execution

### 2️⃣ **Mock Hardware with RViz** (Testing/Development)
```bash
ros2 launch tower_crane_moveit_config moveit_planning_execution.launch.py
```
or
```bash
ros2 launch tower_crane_moveit_config moveit_planning_execution.launch.py \
    can_interface_name:=vcan0
```
- ✅ Launches mock hardware (auto-detected)
- ✅ RViz opens automatically
- ✅ Safe for testing without real motors

### 3️⃣ **Real Hardware WITHOUT RViz** (Headless)
```bash
ros2 launch tower_crane_moveit_config moveit_planning_execution.launch.py \
    can_interface_name:=can0 use_rviz:=false
```
- ✅ Real hardware control
- ❌ No graphical interface
- 💡 Useful for remote operation or performance

### 4️⃣ **Mock Hardware WITHOUT RViz**
```bash
ros2 launch tower_crane_moveit_config moveit_planning_execution.launch.py \
    use_rviz:=false
```

## RViz Fix

### Previous Issue
RViz wouldn't launch because the old logic had conditions that could block it.

### Solution
- RViz now launches by default (`use_rviz:=true`)
- Clean conditional logic using `IfCondition(use_rviz)`
- Works correctly with both real and mock hardware

## Technical Details

### Implementation Changes

**Before (Old Logic):**
```python
# Two separate hardware launch descriptions with conditions
hardware_mock = IncludeLaunchDescription(..., condition=UnlessCondition(use_real_hardware))
hardware_real = IncludeLaunchDescription(..., condition=IfCondition(use_real_hardware))
```

**After (New Logic):**
```python
# Single hardware launch description selected at runtime
def launch_setup(context, *args, **kwargs):
    can_interface_name_str = LaunchConfiguration("can_interface_name").perform(context)
    use_real_hardware = can_interface_name_str.startswith("can") and not can_interface_name_str.startswith("vcan")
    
    if use_real_hardware:
        hardware = IncludeLaunchDescription(...real...)
    else:
        hardware = IncludeLaunchDescription(...mock...)
    
    return [rsp, hardware, virtual_joints, move_group, rviz]
```

### Benefits

1. **Simpler Command Line** - One parameter instead of two
2. **Auto-Detection** - Intelligent hardware selection
3. **Less Error-Prone** - Can't mismatch hardware type and interface
4. **RViz Works** - Fixed conditional logic ensures RViz launches
5. **Cleaner Code** - Using `OpaqueFunction` for runtime evaluation

## Verification

Test that it works:

```bash
# Check mock hardware (should see vcan0 in logs)
ros2 launch tower_crane_moveit_config moveit_planning_execution.launch.py

# Check real hardware auto-detection (should see can0 in logs)
ros2 launch tower_crane_moveit_config moveit_planning_execution.launch.py \
    can_interface_name:=can0
```

Both should launch RViz automatically. Check the terminal output to verify:
- Mock: Look for "robot_control.launch.py" and "vcan0"
- Real: Look for "hardware_bringup_real.launch.py" and "can0"

## Related Changes

This change works together with the unit scaling fix in `bus.yml` to provide precise control with real hardware:
- `scale_pos_to_dev` / `scale_pos_from_dev` parameters added
- Proper radians ↔ encoder counts conversion
- Matches precision of `crane_master` package

---

**File Modified:** `crane/tower_crane_moveit_config/launch/moveit_planning_execution.launch.py`  
**Date:** December 2024


