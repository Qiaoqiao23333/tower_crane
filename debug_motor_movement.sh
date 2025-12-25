#!/bin/bash
# Debug script for motor movement issues

echo "========================================"
echo "Motor Movement Debugging Script"
echo "========================================"
echo ""

GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to check service/topic availability
check_available() {
    if timeout 2 ros2 $1 list | grep -q "$2"; then
        echo -e "${GREEN}✓${NC} $2"
        return 0
    else
        echo -e "${RED}✗${NC} $2"
        return 1
    fi
}

echo "Step 1: Checking if controllers are loaded..."
echo "--------------------------------------------"
ros2 control list_controllers 2>/dev/null || echo -e "${RED}Controller manager not responding${NC}"
echo ""

echo "Step 2: Checking motor services..."
echo "--------------------------------------------"
check_available "service" "/trolley_joint/init"
check_available "service" "/trolley_joint/enable"
check_available "service" "/trolley_joint/target"
echo ""

echo "Step 3: Checking current motor state..."
echo "--------------------------------------------"
echo "Trolley joint state (waiting 2 seconds for data):"
timeout 2 ros2 topic echo /joint_states --once 2>/dev/null | grep -A 10 "trolley_joint" || echo -e "${RED}No data received${NC}"
echo ""

echo "Step 4: Initialize and Enable Motors"
echo "--------------------------------------------"
echo -e "${YELLOW}Initializing trolley motor...${NC}"
ros2 service call /trolley_joint/init std_srvs/srv/Trigger 2>/dev/null
sleep 1

echo -e "${YELLOW}Enabling trolley motor...${NC}"
ros2 service call /trolley_joint/enable std_srvs/srv/Trigger 2>/dev/null
sleep 1

echo -e "${YELLOW}Initializing slewing motor...${NC}"
ros2 service call /slewing_joint/init std_srvs/srv/Trigger 2>/dev/null
sleep 1

echo -e "${YELLOW}Enabling slewing motor...${NC}"
ros2 service call /slewing_joint/enable std_srvs/srv/Trigger 2>/dev/null
sleep 1

echo -e "${YELLOW}Initializing hook motor...${NC}"
ros2 service call /hook_joint/init std_srvs/srv/Trigger 2>/dev/null
sleep 1

echo -e "${YELLOW}Enabling hook motor...${NC}"
ros2 service call /hook_joint/enable std_srvs/srv/Trigger 2>/dev/null
sleep 1
echo ""

echo "Step 5: Check motor mode and status..."
echo "--------------------------------------------"
echo "Checking if motors are in correct operation mode..."
# You can add SDO reads here if available
echo ""

echo "Step 6: Test with small movement (0.01m = 2.78 degrees)"
echo "--------------------------------------------"
echo -e "${YELLOW}Sending small test trajectory...${NC}"
ros2 action send_goal /forward_position_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [slewing_joint, trolley_joint, hook_joint]
  points:
  - positions: [0.0, 0.01, 0.0]
    time_from_start:
      sec: 3
      nanosec: 0
" --feedback 2>&1 | head -20
echo ""

echo "Step 7: Monitor what's being sent to motor..."
echo "--------------------------------------------"
echo "Checking target position topic (should show encoder counts):"
timeout 2 ros2 topic echo /trolley_joint/target --once 2>/dev/null || echo -e "${RED}No target topic available${NC}"
echo ""

echo "========================================"
echo "Diagnostic Summary"
echo "========================================"
echo ""
echo "If motors still don't move, check:"
echo "1. Are motors physically powered on?"
echo "2. Is CAN bus connected properly?"
echo "3. Check motor controller status LEDs"
echo "4. Try direct CANopen command:"
echo "   ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble \"{target: 100.0}\""
echo ""
echo "To monitor encoder scaling in real-time:"
echo "  Terminal 1: ros2 topic echo /joint_states"
echo "  Terminal 2: ros2 topic echo /trolley_joint/target"
echo ""


