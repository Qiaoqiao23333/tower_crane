#!/bin/bash
# Script to check trolley motor status after init

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ~/appdata/canros/install/setup.bash

echo "========================================="
echo "Checking Trolley Motor Status After Init"
echo "========================================="
echo ""

echo "1. Calling init service..."
ros2 service call /trolley_joint/init std_srvs/srv/Trigger
echo ""

echo "2. Waiting 2 seconds for state to stabilize..."
sleep 2
echo ""

echo "3. Checking Statusword (0x6041) - Motor State..."
STATUS=$(ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}" 2>/dev/null | grep -A 1 "data:" | tail -1 | awk '{print $2}')
if [ -n "$STATUS" ]; then
    STATUS_DEC=$((0x${STATUS}))
    echo "   Statusword: 0x${STATUS} (decimal: ${STATUS_DEC})"
    
    # Check individual bits
    if [ $((STATUS_DEC & 0x0001)) -ne 0 ]; then echo "   ✓ Ready to switch on"; fi
    if [ $((STATUS_DEC & 0x0002)) -ne 0 ]; then echo "   ✓ Switched on"; fi
    if [ $((STATUS_DEC & 0x0004)) -ne 0 ]; then echo "   ✓ Operation enabled"; fi
    if [ $((STATUS_DEC & 0x0008)) -ne 0 ]; then echo "   ⚠ Fault bit is set"; fi
    if [ $((STATUS_DEC & 0x0010)) -ne 0 ]; then echo "   ✓ Voltage enabled"; fi
    if [ $((STATUS_DEC & 0x0020)) -ne 0 ]; then echo "   ✓ Quick stop"; fi
    
    # Check if motor is operation enabled (good state)
    if [ $((STATUS_DEC & 0x0004)) -ne 0 ]; then
        echo ""
        echo "   ✓ Motor is OPERATION ENABLED - Ready to move!"
    else
        echo ""
        echo "   ⚠ Motor is NOT operation enabled"
        echo "   → Try: ros2 service call /trolley_joint/enable std_srvs/srv/Trigger"
    fi
else
    echo "   ✗ Could not read statusword"
fi
echo ""

echo "4. Checking Operation Mode (0x6061)..."
MODE=$(ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6061, subindex: 0}" 2>/dev/null | grep -A 1 "data:" | tail -1 | awk '{print $2}')
if [ -n "$MODE" ]; then
    MODE_DEC=$((0x${MODE}))
    echo "   Operation Mode: ${MODE_DEC}"
    case $MODE_DEC in
        1) echo "   ✓ Profile Position Mode (correct)";;
        3) echo "   ✓ Profile Velocity Mode";;
        6) echo "   ✓ Homing Mode";;
        8) echo "   ✓ Interpolated Position Mode";;
        *) echo "   ⚠ Mode ${MODE_DEC} (may need to set position mode)";;
    esac
else
    echo "   ✗ Could not read operation mode"
fi
echo ""

echo "5. Checking Error Register (0x1001)..."
ERROR=$(ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 1001, subindex: 0}" 2>/dev/null | grep -A 1 "data:" | tail -1 | awk '{print $2}')
if [ -n "$ERROR" ]; then
    ERROR_DEC=$((0x${ERROR}))
    if [ "$ERROR_DEC" = "0" ]; then
        echo "   ✓ No errors (0x00)"
    else
        echo "   ⚠ Error register: 0x${ERROR} (decimal: ${ERROR_DEC})"
        echo "   → Try: ros2 service call /trolley_joint/recover std_srvs/srv/Trigger"
    fi
else
    echo "   ✗ Could not read error register"
fi
echo ""

echo "6. Checking joint states topic..."
JOINT_STATE=$(ros2 topic echo /trolley_joint/joint_states --once 2>/dev/null | grep -A 1 "position:" | tail -1)
if [ -n "$JOINT_STATE" ]; then
    echo "   ✓ Joint states publishing: $JOINT_STATE"
else
    echo "   ⚠ Joint states not publishing"
fi
echo ""

echo "========================================="
echo "Summary:"
echo "========================================="
echo "The 'Fault Reset' message during init is NORMAL."
echo "It's part of the initialization sequence to ensure"
echo "the motor starts from a clean state."
echo ""
echo "If Statusword shows 'Operation enabled' (bit 2 set),"
echo "the motor is ready to move!"

