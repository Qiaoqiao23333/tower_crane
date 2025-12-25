#!/bin/bash

echo "=========================================="
echo "Testing Trolley Motor (With Timeout Protection)"
echo "=========================================="
echo ""

# Set timeout for all service calls
SERVICE_TIMEOUT=5

echo "Step 1: Check if CAN interface is up..."
if ip link show can0 &>/dev/null; then
    CAN_STATE=$(ip link show can0 | grep -o 'state [A-Z]*' | awk '{print $2}')
    echo "   CAN interface: $CAN_STATE"
    if [ "$CAN_STATE" != "UP" ]; then
        echo "   ⚠️  WARNING: CAN interface is not UP!"
        echo "   → Run: sudo ip link set can0 up type can bitrate 500000"
    fi
else
    echo "   ❌ ERROR: CAN interface can0 not found!"
    exit 1
fi
echo ""

echo "Step 2: Check if trolley_joint node is alive..."
if ros2 node list | grep -q "/trolley_joint"; then
    echo "   ✓ trolley_joint node is running"
else
    echo "   ❌ ERROR: trolley_joint node not found!"
    exit 1
fi
echo ""

echo "Step 3: Test direct CAN communication with motor (Node ID 2)..."
echo "   Sending SDO read request for Statusword (0x6041)..."
cansend can0 602#4041600000000000 2>/dev/null
echo "   Waiting for response (timeout 1 second)..."
RESPONSE=$(timeout 1 candump can0,582:7FF 2>/dev/null | head -1)
if [ -n "$RESPONSE" ]; then
    echo "   ✓ Motor responded: $RESPONSE"
    echo "   → Motor is communicating on CAN bus!"
else
    echo "   ❌ No response from motor!"
    echo "   → Check:"
    echo "      - Is motor powered on?"
    echo "      - Is CAN cable connected?"
    echo "      - Is Node ID = 2 configured correctly?"
    echo ""
    echo "   Exiting: Cannot proceed without motor communication"
    exit 1
fi
echo ""

echo "Step 4: Check current trolley position..."
POSITION=$(timeout 2 ros2 topic echo /trolley_joint/joint_states --once 2>/dev/null | grep -A 1 "position:" | tail -1 | awk '{print $2}')
if [ -n "$POSITION" ]; then
    echo "   Current position: $POSITION"
else
    echo "   ⚠️  Could not read position topic"
fi
echo ""

echo "Step 5: Check motor status (SDO read via ROS2)..."
echo "   Reading Statusword (0x6041) with ${SERVICE_TIMEOUT}s timeout..."
STATUS_RESULT=$(timeout $SERVICE_TIMEOUT ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}" 2>&1)
if [ $? -eq 124 ]; then
    echo "   ❌ Service call TIMED OUT after ${SERVICE_TIMEOUT}s"
    echo "   → This means the CANopen node is stuck waiting for motor response"
    echo "   → Try restarting the CANopen node/launch file"
    exit 1
elif echo "$STATUS_RESULT" | grep -q "success: true"; then
    STATUS_VALUE=$(echo "$STATUS_RESULT" | grep "data:" | awk '{print $2}')
    echo "   ✓ Statusword: $STATUS_VALUE"
else
    echo "   ⚠️  SDO read failed or returned error"
    echo "$STATUS_RESULT"
fi
echo ""

echo "Step 6: Try to enable motor (with timeout)..."
echo "   Sending enable command (${SERVICE_TIMEOUT}s timeout)..."
ENABLE_RESULT=$(timeout $SERVICE_TIMEOUT ros2 service call /trolley_joint/enable std_srvs/srv/Trigger 2>&1)
if [ $? -eq 124 ]; then
    echo "   ❌ Enable service TIMED OUT"
    echo "   → Motor is not responding to enable command"
    echo "   → Possible causes:"
    echo "      - Motor in fault state"
    echo "      - CANopen communication blocked"
    echo "      - Motor needs initialization first"
    exit 1
elif echo "$ENABLE_RESULT" | grep -q "success: true"; then
    echo "   ✓ Enable command accepted"
else
    echo "   ⚠️  Enable command failed"
    echo "$ENABLE_RESULT"
fi
echo ""

echo "Step 7: Wait 2 seconds for enable to complete..."
sleep 2
echo ""

echo "Step 8: Send target position (90 degrees) with timeout..."
echo "   Sending target command (${SERVICE_TIMEOUT}s timeout)..."
TARGET_RESULT=$(timeout $SERVICE_TIMEOUT ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}" 2>&1)
if [ $? -eq 124 ]; then
    echo "   ❌ Target service TIMED OUT"
    echo "   → Service call is hanging - motor not responding"
    echo ""
    echo "   DIAGNOSIS:"
    echo "   -----------"
    echo "   The service exists but the CANopen node is stuck waiting"
    echo "   for the motor to acknowledge the command via SDO."
    echo ""
    echo "   SOLUTIONS:"
    echo "   1. Check if motor is in correct operation mode:"
    echo "      ros2 service call /trolley_joint/position_mode std_srvs/srv/Trigger"
    echo ""
    echo "   2. Try recovering motor from fault:"
    echo "      ros2 service call /trolley_joint/recover std_srvs/srv/Trigger"
    echo ""
    echo "   3. Restart the CANopen launch file"
    echo ""
    echo "   4. Check CAN bus errors:"
    echo "      ip -details -statistics link show can0"
    exit 1
elif echo "$TARGET_RESULT" | grep -q "success: true"; then
    echo "   ✓ Target command accepted"
else
    echo "   ⚠️  Target command failed"
    echo "$TARGET_RESULT"
fi
echo ""

echo "Step 9: Wait 3 seconds for movement..."
sleep 3
echo ""

echo "Step 10: Check new position..."
NEW_POSITION=$(timeout 2 ros2 topic echo /trolley_joint/joint_states --once 2>/dev/null | grep -A 1 "position:" | tail -1 | awk '{print $2}')
if [ -n "$NEW_POSITION" ]; then
    echo "   New position: $NEW_POSITION"
    if [ "$NEW_POSITION" != "$POSITION" ] && [ "$NEW_POSITION" != "0.0" ]; then
        echo ""
        echo "   ✅ SUCCESS! Motor moved from $POSITION to $NEW_POSITION"
    else
        echo ""
        echo "   ⚠️  Position did not change (still $NEW_POSITION)"
        echo "   → Motor accepted command but didn't move"
        echo "   → Check if motor needs to be set to position mode first"
    fi
else
    echo "   ⚠️  Could not read new position"
fi
echo ""

echo "=========================================="
echo "Test Complete"
echo "=========================================="


