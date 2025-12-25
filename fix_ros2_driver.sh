#!/bin/bash
# Fix ROS2 CANopen driver not publishing topics

echo "========================================="
echo "ROS2 CANopen Driver Diagnostic"
echo "========================================="
echo ""

echo "1. Checking if ros2_control_node is running..."
if ros2 node list 2>/dev/null | grep -q "ros2_control_node"; then
    echo "   ✓ ros2_control_node is running"
    ros2 node list | grep "ros2_control_node"
else
    echo "   ✗ ros2_control_node is NOT running"
    echo "   → You need to launch the hardware bringup"
    echo "   → Run: ros2 launch tower_crane hardware_bringup_real.launch.py"
    exit 1
fi
echo ""

echo "2. Checking for CANopen-related nodes..."
ros2 node list | grep -E "(canopen|device_container|master)" || echo "   No CANopen nodes found"
echo ""

echo "3. Checking available joint topics..."
JOINT_TOPICS=$(ros2 topic list 2>/dev/null | grep -E "joint|canopen" | head -10)
if [ -n "$JOINT_TOPICS" ]; then
    echo "   Available topics:"
    echo "$JOINT_TOPICS" | sed 's/^/      /'
else
    echo "   ✗ No joint topics found"
fi
echo ""

echo "4. Checking if device_container is running..."
if ros2 node list 2>/dev/null | grep -q "device_container"; then
    echo "   ✓ device_container is running"
    echo "   → This is the CANopen master driver"
else
    echo "   ✗ device_container is NOT running"
    echo "   → CANopen master driver not started"
fi
echo ""

echo "5. Checking controller manager status..."
if ros2 control list_controllers 2>/dev/null > /dev/null; then
    echo "   ✓ Controller manager is accessible"
    echo ""
    echo "   Controllers:"
    ros2 control list_controllers 2>/dev/null | sed 's/^/      /'
else
    echo "   ✗ Controller manager not accessible"
fi
echo ""

echo "========================================="
echo "Recommended Actions"
echo "========================================="
echo ""
echo "If ros2_control_node is not running:"
echo "  1. Launch the hardware bringup:"
echo "     ros2 launch tower_crane hardware_bringup_real.launch.py"
echo ""
echo "  2. Or if already launched, check for errors in the terminal"
echo ""
echo "If device_container is not running:"
echo "  - The CANopen master driver may have failed to start"
echo "  - Check for SDO timeout errors in the launch terminal"
echo "  - Verify bus.yml and master.dcf configuration"
echo ""
echo "If topics exist but no data:"
echo "  - Wait a few seconds for initialization"
echo "  - Check: ros2 topic echo /hook_joint/joint_states --once"
echo "  - Check: ros2 topic echo /trolley_joint/joint_states --once"
echo ""



