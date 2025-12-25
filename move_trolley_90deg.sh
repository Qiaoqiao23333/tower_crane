#!/bin/bash
# Script to move trolley joint 90 degrees (motor rotation)
# This translates to 0.09 meters of linear movement

echo "Moving trolley 90 degrees (0.09 meters)..."

# Method 1: Direct motor command (90 degrees)
# This sends 90 degrees directly to the motor controller
ros2 topic pub --once /trolley/target_position std_msgs/msg/Float32 "{data: 90.0}"

echo "Command sent! Monitor progress with:"
echo "  ros2 topic echo /joint_states | grep trolley_joint"



