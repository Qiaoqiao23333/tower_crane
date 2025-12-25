#!/bin/bash
# Script to enable motors after system restart

echo "========================================"
echo "Motor Initialization Script"
echo "========================================"
echo ""
echo "This script will:"
echo "  1. Initialize all three motors"
echo "  2. Enable all three motors"
echo "  3. Run a test movement"
echo ""
echo "Make sure the hardware_bringup_real.launch.py is running!"
echo ""

read -p "Press Enter to continue or Ctrl+C to cancel..."

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Check if ros2 is available
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}ERROR: ros2 command not found!${NC}"
    echo "Please source the workspace first:"
    echo "  cd ~/appdata/canros"
    echo "  source install/setup.bash"
    exit 1
fi

echo ""
echo -e "${YELLOW}Step 1: Initializing motors...${NC}"
echo "--------------------------------------------"

echo "Initializing slewing joint..."
ros2 service call /slewing_joint/init std_srvs/srv/Trigger 2>&1 | grep -i "success\|message"

echo "Initializing trolley joint..."
ros2 service call /trolley_joint/init std_srvs/srv/Trigger 2>&1 | grep -i "success\|message"

echo "Initializing hook joint..."
ros2 service call /hook_joint/init std_srvs/srv/Trigger 2>&1 | grep -i "success\|message"

echo ""
echo "Waiting 2 seconds..."
sleep 2

echo ""
echo -e "${YELLOW}Step 2: Enabling motors...${NC}"
echo "--------------------------------------------"

echo "Enabling slewing joint..."
ros2 service call /slewing_joint/enable std_srvs/srv/Trigger 2>&1 | grep -i "success\|message"

echo "Enabling trolley joint..."
ros2 service call /trolley_joint/enable std_srvs/srv/Trigger 2>&1 | grep -i "success\|message"

echo "Enabling hook joint..."
ros2 service call /hook_joint/enable std_srvs/srv/Trigger 2>&1 | grep -i "success\|message"

echo ""
echo "Waiting 2 seconds..."
sleep 2

echo ""
echo -e "${GREEN}✓ Motors initialized and enabled${NC}"
echo ""

# Ask if user wants to run test
read -p "Run test movement (1cm trolley)? [Y/n]: " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]] || [[ -z $REPLY ]]; then
    echo ""
    echo -e "${YELLOW}Step 3: Running test movement...${NC}"
    echo "--------------------------------------------"
    echo "Moving trolley 1cm (0.01 meters = ~27.78 encoder counts)"
    echo ""
    
    ros2 action send_goal /forward_position_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [slewing_joint, trolley_joint, hook_joint]
  points:
  - positions: [0.0, 0.01, 0.0]
    time_from_start:
      sec: 5
      nanosec: 0
" --feedback
    
    echo ""
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ Test movement completed!${NC}"
    else
        echo -e "${RED}✗ Test movement failed${NC}"
        echo "Check the output above for errors"
    fi
fi

echo ""
echo "========================================"
echo "Initialization Complete!"
echo "========================================"
echo ""
echo "You can now send trajectory commands:"
echo ""
echo "Example - Move trolley 9cm (90 degrees):"
echo "  ros2 action send_goal /forward_position_controller/follow_joint_trajectory \\"
echo "    control_msgs/action/FollowJointTrajectory \\"
echo "    \"trajectory:"
echo "      joint_names: [slewing_joint, trolley_joint, hook_joint]"
echo "      points:"
echo "      - positions: [0.0, 0.09, 0.0]"
echo "        time_from_start: {sec: 5}\""
echo ""
echo "To monitor joint states:"
echo "  ros2 topic echo /joint_states"
echo ""
echo "To monitor encoder values:"
echo "  ros2 topic echo /trolley_joint/target"
echo ""


