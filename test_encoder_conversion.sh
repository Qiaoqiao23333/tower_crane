#!/bin/bash
# Test script to verify encoder count conversion is working correctly

echo "================================"
echo "Encoder Count Conversion Test"
echo "================================"
echo ""
echo "This script will test if the /forward_position_controller"
echo "is correctly converting ROS2 units to encoder counts."
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}Test 1: Move trolley 90 degrees (0.09 meters)${NC}"
echo "Expected: Motor should receive 250 encoder counts"
echo "Command: positions: [0.0, 0.09, 0.0]"
echo ""
read -p "Press Enter to execute..."

ros2 action send_goal /forward_position_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [slewing_joint, trolley_joint, hook_joint]
  points:
  - positions: [0.0, 0.09, 0.0]
    time_from_start:
      sec: 5
      nanosec: 0
" --feedback

echo ""
echo -e "${GREEN}✓ Test 1 completed${NC}"
echo ""
sleep 2

echo -e "${YELLOW}Test 2: Rotate slewing 90 degrees (1.5708 radians)${NC}"
echo "Expected: Motor should receive 250 encoder counts"
echo "Command: positions: [1.5708, 0.0, 0.0]"
echo ""
read -p "Press Enter to execute..."

ros2 action send_goal /forward_position_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [slewing_joint, trolley_joint, hook_joint]
  points:
  - positions: [1.5708, 0.0, 0.0]
    time_from_start:
      sec: 5
      nanosec: 0
" --feedback

echo ""
echo -e "${GREEN}✓ Test 2 completed${NC}"
echo ""
sleep 2

echo -e "${YELLOW}Test 3: Move trolley back to home (0.0 meters)${NC}"
echo "Expected: Motor should receive 0 encoder counts"
echo "Command: positions: [0.0, 0.0, 0.0]"
echo ""
read -p "Press Enter to execute..."

ros2 action send_goal /forward_position_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [slewing_joint, trolley_joint, hook_joint]
  points:
  - positions: [0.0, 0.0, 0.0]
    time_from_start:
      sec: 5
      nanosec: 0
" --feedback

echo ""
echo -e "${GREEN}✓ Test 3 completed${NC}"
echo ""

echo "================================"
echo "All tests completed!"
echo "================================"
echo ""
echo "To monitor the actual encoder counts being sent to the motors,"
echo "you can use:"
echo ""
echo "  ros2 topic echo /trolley_joint/target"
echo "  ros2 topic echo /slewing_joint/target"
echo ""
echo "These should show values in encoder counts, not ROS2 units."


