#!/bin/bash
# Check what services and topics are actually available

echo "=========================================="
echo "Available Services for trolley_joint:"
echo "=========================================="
ros2 service list | grep trolley

echo ""
echo "=========================================="
echo "Available Topics for trolley_joint:"
echo "=========================================="
ros2 topic list | grep trolley

echo ""
echo "=========================================="
echo "All CANopen-related Services:"
echo "=========================================="
ros2 service list | grep -E "(canopen|sdo|target|init|enable|mode)"

echo ""
echo "=========================================="
echo "Controller Manager Services:"
echo "=========================================="
ros2 service list | grep controller

echo ""
echo "=========================================="
echo "Active Controllers:"
echo "=========================================="
ros2 control list_controllers 2>/dev/null || echo "ros2 control CLI not available"

echo ""
echo "=========================================="
echo "Hardware Interfaces:"
echo "=========================================="
ros2 control list_hardware_interfaces 2>/dev/null || echo "ros2 control CLI not available"


