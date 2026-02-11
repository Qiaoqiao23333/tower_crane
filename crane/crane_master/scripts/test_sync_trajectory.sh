#!/bin/bash
# Test script for synchronized trajectory action server

echo "==================================================================="
echo "  Synchronized Trajectory Action Server - Test Script"
echo "==================================================================="
echo ""
echo "This script demonstrates the synchronized trajectory control."
echo "All motors (slewing, trolley, hook) will move simultaneously"
echo "and be synchronized with SYNC (0x080) frames."
echo ""
echo "==================================================================="
echo ""

# Check if ROS 2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS 2 is not sourced!"
    echo "Please run: source /opt/ros/humble/setup.bash"
    echo "       and: source install/setup.bash"
    exit 1
fi

echo "Step 1: Check if action server is running..."
echo ""

if ros2 action list | grep -q "/forward_position_controller/follow_joint_trajectory"; then
    echo "✓ Action server is running"
else
    echo "✗ Action server not found!"
    echo ""
    echo "Please start the crane system first:"
    echo "  ros2 launch crane_master canopen_ros2.launch.py"
    echo ""
    echo "This will start both the motor nodes and the synchronized action server."
    exit 1
fi

echo ""
echo "==================================================================="
echo "Step 2: Sending synchronized trajectory command..."
echo "==================================================================="
echo ""
echo "Command: Move all three joints simultaneously"
echo "  - slewing_joint:  10.0 degrees"
echo "  - trolley_joint:   1.0 degrees"
echo "  - hook_joint:     20.0 degrees"
echo "  - Duration:        2 seconds"
echo ""

ros2 action send_goal /forward_position_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [slewing_joint, trolley_joint, hook_joint]
  points:
  - positions: [10.0, 1.0, 20.0]
    time_from_start:
      sec: 2
" --feedback

echo ""
echo "==================================================================="
echo "  Test Complete"
echo "==================================================================="
echo ""
echo "The motors should have moved simultaneously and synchronized."
echo "Check the terminal where sync_trajectory_action_server is running"
echo "to see the SYNC frame messages."
echo ""
