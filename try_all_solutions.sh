#!/bin/bash

echo "=============================================="
echo "Trying All Solutions for Motor Not Moving"
echo "=============================================="
echo ""

TIMEOUT=3

echo "Solution 1: Check if node is lifecycle-managed..."
if ros2 lifecycle list 2>/dev/null | grep -q "trolley_joint"; then
    echo "   ✓ trolley_joint is a lifecycle node"
    STATE=$(ros2 lifecycle get /trolley_joint 2>/dev/null)
    echo "   Current state: $STATE"
    
    if ! echo "$STATE" | grep -q "active"; then
        echo "   → Node is NOT active, activating..."
        ros2 lifecycle set /trolley_joint configure
        sleep 1
        ros2 lifecycle set /trolley_joint activate
        sleep 2
        echo "   ✓ Node activated"
    else
        echo "   ✓ Node is already active"
    fi
else
    echo "   → trolley_joint is NOT a lifecycle node (regular node)"
fi
echo ""

echo "Solution 2: Try cyclic position mode (PDO-based)..."
echo "   Switching to cyclic_position_mode..."
RESULT=$(timeout $TIMEOUT ros2 service call /trolley_joint/cyclic_position_mode std_srvs/srv/Trigger 2>&1)
if [ $? -eq 124 ]; then
    echo "   ❌ Timed out"
elif echo "$RESULT" | grep -q "success: true"; then
    echo "   ✓ Switched to cyclic position mode"
elif echo "$RESULT" | grep -q "success: false"; then
    echo "   ⚠️  Already in cyclic position mode (or switch failed)"
else
    echo "   ⚠️  Unexpected response"
fi
echo ""

echo "Solution 3: Initialize motor..."
RESULT=$(timeout $TIMEOUT ros2 service call /trolley_joint/init std_srvs/srv/Trigger 2>&1)
if [ $? -eq 124 ]; then
    echo "   ❌ Init timed out"
elif echo "$RESULT" | grep -q "success: true"; then
    echo "   ✓ Motor initialized"
    sleep 2
else
    echo "   ⚠️  Init failed or returned false"
fi
echo ""

echo "Solution 4: Enable motor..."
RESULT=$(timeout $TIMEOUT ros2 service call /trolley_joint/enable std_srvs/srv/Trigger 2>&1)
if [ $? -eq 124 ]; then
    echo "   ❌ Enable timed out"
elif echo "$RESULT" | grep -q "success: true"; then
    echo "   ✓ Motor enabled"
    sleep 2
else
    echo "   ⚠️  Enable failed"
fi
echo ""

echo "Solution 5: Read current position..."
POSITION=$(timeout 2 ros2 topic echo /trolley_joint/joint_states --once 2>/dev/null | grep -A 1 "position:" | tail -1 | awk '{print $2}')
if [ -n "$POSITION" ]; then
    echo "   Current position: $POSITION"
else
    echo "   ⚠️  Could not read position"
fi
echo ""

echo "Solution 6: Send small test target (10 degrees)..."
RESULT=$(timeout $TIMEOUT ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 10.0}" 2>&1)
if [ $? -eq 124 ]; then
    echo "   ❌ Target timed out"
    echo ""
    echo "   DIAGNOSIS: Services are timing out!"
    echo "   → The ros2_canopen node is stuck"
    echo "   → Solution: RESTART the launch file"
    echo ""
    echo "   Run:"
    echo "   1. Press Ctrl+C in terminal running launch"
    echo "   2. ros2 launch tower_crane hardware_bringup_real.launch.py"
    exit 1
elif echo "$RESULT" | grep -q "success: true"; then
    echo "   ✓ Target command sent"
else
    echo "   ⚠️  Target failed"
fi
echo ""

echo "Solution 7: Wait and check if position changed..."
sleep 3
NEW_POSITION=$(timeout 2 ros2 topic echo /trolley_joint/joint_states --once 2>/dev/null | grep -A 1 "position:" | tail -1 | awk '{print $2}')
if [ -n "$NEW_POSITION" ]; then
    echo "   New position: $NEW_POSITION"
    
    if [ "$NEW_POSITION" != "$POSITION" ] && [ "$NEW_POSITION" != "0.0" ]; then
        echo ""
        echo "   ✅ SUCCESS! Motor is moving!"
        echo "   Position changed from $POSITION to $NEW_POSITION"
        echo ""
        echo "   The motor is working. You can now send larger targets."
        exit 0
    else
        echo "   ⚠️  Position unchanged"
    fi
else
    echo "   ⚠️  Could not read new position"
fi
echo ""

echo "=============================================="
echo "Diagnosis Results"
echo "=============================================="
echo ""
echo "Services respond but motor doesn't move."
echo ""
echo "Possible causes:"
echo "1. Motor in wrong operational state"
echo "2. Motor needs homing first"
echo "3. Target value in wrong units"
echo "4. Motor brake/enable signal not connected"
echo "5. Motor drive has internal fault"
echo ""
echo "Next steps:"
echo ""
echo "A. Check motor statusword directly via CAN:"
echo "   cansend can0 602#4041600000000000"
echo "   candump can0,582:7FF -n 1"
echo "   # Look at bytes 4-5 for statusword"
echo ""
echo "B. Check motor operation mode:"
echo "   cansend can0 602#4061600000000000"
echo "   candump can0,582:7FF -n 1"
echo "   # Byte 4 should be 0x01 (profile position) or 0x08 (cyclic position)"
echo ""
echo "C. Try sending target directly via CAN:"
echo "   # Set target position to 1000 (0x3E8) encoder counts"
echo "   cansend can0 602#237A60E8030000"
echo "   # This writes 1000 to 0x607A (Target Position)"
echo ""
echo "D. Check if motor needs controlword to start:"
echo "   # Send \"new setpoint\" bit via controlword"
echo "   cansend can0 602#2B40601F00"
echo "   # This writes 0x001F (all enable bits + new setpoint) to 0x6040"
echo ""
echo "E. Restart launch file (most reliable):"
echo "   Ctrl+C (stop current)"
echo "   ros2 launch tower_crane hardware_bringup_real.launch.py"
echo ""
echo "F. Enable verbose logging:"
echo "   ros2 launch tower_crane hardware_bringup_real.launch.py --log-level DEBUG"
echo ""


