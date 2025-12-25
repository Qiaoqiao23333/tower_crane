#!/bin/bash

echo "=========================================="
echo "Testing Trolley Motor After Initialization"
echo "=========================================="
echo ""

# Set timeout for service calls
TIMEOUT=5

echo "Step 1: Check current trolley position..."
ros2 topic echo /trolley_joint/joint_states --once | grep -A 1 "position:"
echo ""

echo "Step 2: Set position mode (IMPORTANT - was missing!)..."
timeout $TIMEOUT ros2 service call /trolley_joint/position_mode std_srvs/srv/Trigger
if [ $? -eq 124 ]; then
    echo "⚠️  Service timed out! See HANGING_SERVICE_FIX.md"
    exit 1
fi
echo ""

echo "Step 3: Enable trolley motor..."
timeout $TIMEOUT ros2 service call /trolley_joint/enable std_srvs/srv/Trigger
if [ $? -eq 124 ]; then
    echo "⚠️  Service timed out! See HANGING_SERVICE_FIX.md"
    exit 1
fi
echo ""

echo "Step 4: Wait 2 seconds for enable to complete..."
sleep 2
echo ""

echo "Step 5: Send target position (90 degrees)..."
timeout $TIMEOUT ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"
if [ $? -eq 124 ]; then
    echo "⚠️  Service timed out! See HANGING_SERVICE_FIX.md"
    echo "Try: ./fix_hanging_service.sh"
    exit 1
fi
echo ""

echo "Step 6: Wait 3 seconds for movement..."
sleep 3
echo ""

echo "Step 7: Check new position..."
ros2 topic echo /trolley_joint/joint_states --once | grep -A 1 "position:"
echo ""

echo "=========================================="
echo "If position changed, motor is working!"
echo "If service timed out, run: ./fix_hanging_service.sh"
echo "=========================================="

