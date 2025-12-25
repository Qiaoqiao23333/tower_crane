#!/bin/bash
# Correct script to move trolley 90 degrees
# Based on actual ROS2 system with CANopen

echo "========================================="
echo "Moving Trolley 90 Degrees"
echo "========================================="
echo ""

# Method 1: Direct CANopen service (RECOMMENDED)
echo "Method 1: Using CANopen target service..."
echo "Command: ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble \"{target: 90.0}\""
echo ""
read -p "Execute this command? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"
    echo "✓ Command sent!"
fi

echo ""
echo "========================================="
echo "Alternative: Using Joint Trajectory Controller"
echo "========================================="
echo ""
echo "Method 2: Using trajectory controller (0.09 meters = 90 degrees)..."
echo ""
read -p "Execute trajectory command? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    ros2 topic pub --once /forward_position_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
{
  joint_names: ['slewing_joint', 'trolley_joint', 'hook_joint'],
  points: [
    {
      positions: [0.0, 0.09, 0.0],
      time_from_start: {sec: 5, nanosec: 0}
    }
  ]
}"
    echo "✓ Trajectory command sent!"
fi

echo ""
echo "========================================="
echo "Monitor progress:"
echo "  ros2 topic echo /trolley_joint/joint_states"
echo "  ros2 topic echo /joint_states | grep trolley_joint"
echo "========================================="



