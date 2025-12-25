#!/bin/bash

echo "=========================================="
echo "Fix Hanging Service Calls"
echo "=========================================="
echo ""

echo "This script will help recover from hanging service calls"
echo ""

echo "Step 1: Check CAN bus status..."
if ip link show can0 &>/dev/null; then
    echo "   CAN statistics:"
    ip -details -statistics link show can0 | grep -E "(RX|TX|errors)"
    echo ""
    
    # Check for errors
    RX_ERRORS=$(ip -details -statistics link show can0 | grep "RX.*errors" | awk '{print $3}')
    TX_ERRORS=$(ip -details -statistics link show can0 | grep "TX.*errors" | awk '{print $3}')
    
    if [ "$RX_ERRORS" -gt 100 ] || [ "$TX_ERRORS" -gt 100 ]; then
        echo "   ⚠️  High error count detected!"
        echo "   → Consider restarting CAN interface:"
        echo "      sudo ip link set can0 down"
        echo "      sudo ip link set can0 up type can bitrate 500000"
    fi
else
    echo "   ❌ CAN interface not found!"
    exit 1
fi
echo ""

echo "Step 2: Test direct motor communication..."
echo "   Testing trolley motor (Node 2) response..."
cansend can0 602#4041600000000000 2>/dev/null
sleep 0.1
RESPONSE=$(timeout 1 candump can0,582:7FF 2>/dev/null | head -1)
if [ -n "$RESPONSE" ]; then
    echo "   ✓ Motor is responding on CAN bus: $RESPONSE"
else
    echo "   ❌ Motor NOT responding!"
    echo "   → Check motor power and CAN connection"
    echo "   → This is why services hang - motor doesn't respond"
    exit 1
fi
echo ""

echo "Step 3: Try quick recovery sequence..."
echo "   Setting position mode (with 3s timeout)..."
RESULT=$(timeout 3 ros2 service call /trolley_joint/position_mode std_srvs/srv/Trigger 2>&1)
if [ $? -eq 124 ]; then
    echo "   ❌ TIMEOUT - service is stuck"
    echo ""
    echo "   SOLUTION: You need to restart the CANopen node/launch file"
    echo "   The node is in a deadlocked state waiting for motor SDO response."
    echo ""
    echo "   To restart:"
    echo "   1. Press Ctrl+C in the terminal running the launch file"
    echo "   2. Restart: ros2 launch tower_crane hardware_bringup_real.launch.py"
    exit 1
elif echo "$RESULT" | grep -q "success: true"; then
    echo "   ✓ Position mode set successfully"
else
    echo "   ⚠️  Position mode call failed"
fi
echo ""

echo "Step 4: Enable motor (with 3s timeout)..."
RESULT=$(timeout 3 ros2 service call /trolley_joint/enable std_srvs/srv/Trigger 2>&1)
if [ $? -eq 124 ]; then
    echo "   ❌ TIMEOUT - cannot enable motor"
    echo "   → Restart CANopen launch file required"
    exit 1
elif echo "$RESULT" | grep -q "success: true"; then
    echo "   ✓ Motor enabled successfully"
else
    echo "   ⚠️  Enable call failed"
fi
echo ""

echo "Step 5: Test target command (with 3s timeout)..."
RESULT=$(timeout 3 ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 10.0}" 2>&1)
if [ $? -eq 124 ]; then
    echo "   ❌ TIMEOUT - target service still hanging"
    echo ""
    echo "   ROOT CAUSE IDENTIFIED:"
    echo "   ====================="
    echo "   The CANopen node's SDO communication is blocking."
    echo "   Even though motor responds on CAN bus, the ROS2 service"
    echo "   is stuck waiting for SDO acknowledgment."
    echo ""
    echo "   SOLUTIONS:"
    echo "   ---------"
    echo "   1. Restart the launch file (preferred)"
    echo "   2. Check if motor is in correct PDO mode vs SDO mode"
    echo "   3. Increase SDO timeout in bus.yml (already set to 2000ms)"
    echo "   4. Check for CAN bus overload (too many messages)"
    exit 1
elif echo "$RESULT" | grep -q "success: true"; then
    echo "   ✓ Target command works!"
    echo ""
    echo "   ✅ RECOVERED! Services are responding now."
    echo "   → You can try moving the motor again"
else
    echo "   ⚠️  Target call failed"
fi
echo ""

echo "=========================================="
echo "Recovery Complete"
echo "=========================================="
echo ""
echo "If services still hang, the only solution is to:"
echo "1. Stop the CANopen launch file (Ctrl+C)"
echo "2. Restart: ros2 launch tower_crane hardware_bringup_real.launch.py"


