# Why "Fault Reset" Always Appears During Init

## Question
When calling `/trolley_joint/init`, why does it always show "Fault Reset" even when there's no fault?

## Answer: This is **Normal Behavior** ✅

The "Fault Reset" message is **always** printed during initialization, even when there's no actual fault. This is by design in the CANopen 402 driver.

## How It Works

### 1. Init Service Always Performs Fault Reset

Looking at the code in `canopen_402_driver/src/motor.cpp`:

```cpp
bool Motor402::handleInit()
{
  // ... initialize modes ...
  
  {
    std::scoped_lock lock(cw_mutex_);
    control_word_ = 0;
    start_fault_reset_ = true;  // ← Always set to true
  }
  
  // ... enable motor ...
}
```

The `handleInit()` function **always** sets `start_fault_reset_ = true` as part of the initialization sequence.

### 2. Fault Reset is Logged

When the control word is updated, if `start_fault_reset_` is true, it logs "Fault reset":

```cpp
if (start_fault_reset_.exchange(false))
{
  RCLCPP_INFO(rclcpp::get_logger("canopen_402_driver"), "Fault reset");
  // Clear fault reset bit in control word
}
```

## Why This Design?

This is a **safety feature** to ensure:
1. ✅ The motor always starts from a clean state
2. ✅ Any lingering fault conditions are cleared
3. ✅ Consistent initialization behavior regardless of previous state

## What This Means

- ✅ **"Fault Reset" message = Normal, expected behavior**
- ✅ **Not an error** - it's part of the initialization sequence
- ✅ **Service returns `success=True`** = Initialization succeeded

## How to Verify Motor is Actually Working

After calling init, check the motor state:

```bash
# Check Statusword (0x6041)
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}"

# Good values:
# - 0x0027 or 0x001F = Operation enabled ✓
# - Bit 2 (0x0004) must be set = Operation enabled
```

Or use the diagnostic script:

```bash
./check_trolley_init_status.sh
```

## Complete Initialization Sequence

The init service does:
1. **Fault Reset** (always, even if no fault) ← This is what you see
2. Initialize motor modes
3. Read current state
4. Enable motor to Operation_Enable state

## Summary

**"Fault Reset" during init is normal and expected.** It's not indicating an actual fault - it's the driver ensuring a clean initialization state. If the service returns `success=True`, the motor has been successfully initialized and should be ready to move.



