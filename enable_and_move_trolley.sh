#!/bin/bash
# Complete sequence to enable trolley motor and move it

echo "========================================="
echo "Enabling and Moving Trolley Motor"
echo "========================================="
echo ""

echo "Step 1: Check current motor state..."
echo "   Reading Statusword (0x6041)..."
STATUS=$(ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}" 2>/dev/null | grep -A 1 "data:" | tail -1 | awk '{print $2}')
if [ -n "$STATUS" ]; then
    echo "   Statusword: $STATUS"
    # Check if bit 2 (operation enabled) is set
    STATUS_DEC=$((0x${STATUS}))
    if [ $((STATUS_DEC & 0x0004)) -ne 0 ]; then
        echo "   ✓ Motor is operation enabled"
    else
        echo "   ✗ Motor is NOT operation enabled"
    fi
else
    echo "   Could not read statusword"
fi
echo ""

echo "Step 2: Check operation mode..."
echo "   Reading Operation Mode Display (0x6061)..."
MODE=$(ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6061, subindex: 0}" 2>/dev/null | grep -A 1 "data:" | tail -1 | awk '{print $2}')
if [ -n "$MODE" ]; then
    MODE_DEC=$((0x${MODE}))
    echo "   Operation Mode: $MODE_DEC"
    if [ "$MODE_DEC" = "1" ]; then
        echo "   ✓ Motor is in Profile Position Mode (correct)"
    else
        echo "   ✗ Motor is NOT in position mode (current: $MODE_DEC)"
        echo "   → Setting to position mode..."
        ros2 service call /trolley_joint/position_mode std_srvs/srv/Trigger
        sleep 1
    fi
else
    echo "   Could not read operation mode"
fi
echo ""

echo "Step 3: Initialize motor (if needed)..."
read -p "   Initialize motor? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "   Initializing..."
    ros2 service call /trolley_joint/init std_srvs/srv/Trigger
    sleep 2
fi
echo ""

echo "Step 4: Enable motor..."
read -p "   Enable motor? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "   Enabling..."
    ros2 service call /trolley_joint/enable std_srvs/srv/Trigger
    sleep 2
    
    # Check status again
    STATUS=$(ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}" 2>/dev/null | grep -A 1 "data:" | tail -1 | awk '{print $2}')
    if [ -n "$STATUS" ]; then
        STATUS_DEC=$((0x${STATUS}))
        if [ $((STATUS_DEC & 0x0004)) -ne 0 ]; then
            echo "   ✓ Motor is now enabled"
        else
            echo "   ✗ Motor enable failed (Statusword: $STATUS)"
        fi
    fi
fi
echo ""

echo "Step 5: Check current position..."
CURRENT=$(ros2 topic echo /trolley_joint/joint_states --once 2>/dev/null | grep -A 1 "position:" | tail -1 | awk '{print $2}')
echo "   Current position: $CURRENT"
echo ""

echo "Step 6: Move trolley 90 degrees..."
read -p "   Send target command (90.0)? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    echo "   Sending target: 90.0"
    ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"
    echo ""
    echo "   Monitoring movement (watch for position change)..."
    echo "   Press Ctrl+C to stop monitoring"
    ros2 topic echo /trolley_joint/joint_states
fi

echo ""
echo "========================================="
echo "Done!"
echo "========================================="



