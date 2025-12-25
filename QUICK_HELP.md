# Quick Help - Service Hanging Issue

## 🚨 Problem
Service call hangs with "waiting for service to become available..." even though service exists.

## ✅ Quick Solutions (Try in Order)

### 1. Run Diagnostic (Recommended First)
```bash
./test_trolley_with_timeout.sh
```
This will show you exactly what's wrong with timeout protection.

### 2. Try Recovery
```bash
./fix_hanging_service.sh
```
Attempts to recover without restarting.

### 3. Restart Launch File (Most Reliable)
```bash
# Stop current launch (Ctrl+C)
# Then restart:
ros2 launch tower_crane hardware_bringup_real.launch.py
```

### 4. Check Motor Communication
```bash
# Test direct CAN communication
cansend can0 602#4041600000000000
timeout 1 candump can0,582:7FF

# Should see: 582#4B41600027000000 or similar
# If no response → motor not connected/powered
```

## 📋 Files I Created for You

| File | Purpose |
|------|---------|
| `test_trolley_with_timeout.sh` | Full diagnostic with timeout protection |
| `fix_hanging_service.sh` | Recovery script for hanging services |
| `HANGING_SERVICE_FIX.md` | Complete explanation of the issue |
| `test_trolley_after_init.sh` | **Updated** - Now includes position_mode + timeouts |

## 🔧 Fixed Your Original Script

The original `test_trolley_after_init.sh` was missing:
1. ❌ **Missing:** Set position mode before enable
2. ❌ **Missing:** Timeout protection
3. ✅ **Fixed:** Now includes both!

## 💡 Root Cause

The service **exists** but the CANopen node is **stuck waiting for motor** SDO response:
- Motor not responding on CAN bus
- Node blocks forever waiting
- Your service call hangs

## 🎯 Next Steps

1. **First, run:** `./test_trolley_with_timeout.sh`
2. **If it finds issues:** Follow the suggestions it provides
3. **If services timeout:** Run `./fix_hanging_service.sh`
4. **If still hangs:** Restart launch file
5. **For details:** Read `HANGING_SERVICE_FIX.md`

## ⚡ Always Use Timeout Protection

From now on, wrap service calls in `timeout`:
```bash
# Good ✓
timeout 5 ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"

# Bad ✗ - can hang forever
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"
```

## 🐛 When to Restart Launch

If you see these symptoms, **restart is needed:**
- Multiple services timing out
- CAN communication works but services don't
- Recovery script doesn't help
- Node in deadlocked state

**Not a bug** - just a limitation of blocking SDO communication.

## 📖 More Help

- Full explanation: `HANGING_SERVICE_FIX.md`
- CANopen issues: `CANOPEN_TROUBLESHOOTING.md`
- Motor not moving: `MOTOR_NOT_MOVING_FIX.md`
- How to move trolley: `HOW_TO_MOVE_TROLLEY.md`


