#!/bin/bash
# Comprehensive system status check: nodes, services, topics, and controllers

echo "=========================================="
echo "ROS2 System Status Check"
echo "=========================================="
echo ""

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 1. Check ROS2 nodes
echo "1. ROS2 Nodes:"
echo "----------------------------------------"
NODES=$(timeout 2 ros2 node list 2>/dev/null)
if [ -n "$NODES" ]; then
    echo "$NODES" | while read node; do
        echo -e "   ${GREEN}✓${NC} $node"
    done
else
    echo -e "   ${RED}✗ No nodes found${NC}"
fi
echo ""

# 2. Check key services
echo "2. Key Services:"
echo "----------------------------------------"
echo "   Controller Manager services:"
timeout 2 ros2 service list 2>/dev/null | grep controller_manager | head -5 | sed 's/^/      /' || echo -e "      ${RED}✗ No controller_manager services${NC}"

echo ""
echo "   CANopen motor services:"
for joint in hook_joint trolley_joint slewing_joint; do
    if timeout 1 ros2 service list 2>/dev/null | grep -q "/${joint}/"; then
        echo -e "      ${GREEN}✓${NC} /${joint}/ services available"
    else
        echo -e "      ${RED}✗${NC} /${joint}/ services NOT available"
    fi
done
echo ""

# 3. Check topics
echo "3. Key Topics:"
echo "----------------------------------------"
echo "   Joint-related topics:"
JOINT_TOPICS=$(timeout 2 ros2 topic list 2>/dev/null | grep -E "joint|canopen" | sort)
if [ -n "$JOINT_TOPICS" ]; then
    echo "$JOINT_TOPICS" | while read topic; do
        echo -e "      ${GREEN}✓${NC} $topic"
    done
else
    echo -e "      ${RED}✗ No joint topics found${NC}"
fi

echo ""
echo "   Testing topic data availability:"
for joint in hook_joint trolley_joint slewing_joint; do
    if timeout 2s ros2 topic echo /${joint}/joint_states --once 2>/dev/null > /dev/null; then
        POS=$(timeout 2s ros2 topic echo /${joint}/joint_states --once 2>/dev/null | grep -A 1 "position:" | tail -1 | awk '{print $2}')
        echo -e "      ${GREEN}✓${NC} /${joint}/joint_states (position: $POS)"
    else
        echo -e "      ${RED}✗${NC} /${joint}/joint_states NOT publishing"
    fi
done

if timeout 2s ros2 topic echo /joint_states --once 2>/dev/null > /dev/null; then
    echo -e "      ${GREEN}✓${NC} /joint_states (aggregated)"
else
    echo -e "      ${RED}✗${NC} /joint_states NOT publishing"
fi
echo ""

# 4. Check controllers
echo "4. Controllers:"
echo "----------------------------------------"
if timeout 2 ros2 control list_controllers 2>/dev/null > /dev/null; then
    ros2 control list_controllers 2>/dev/null | sed 's/^/      /'
else
    echo -e "      ${RED}✗ Cannot access controller manager${NC}"
fi
echo ""

# 5. Check hardware interfaces
echo "5. Hardware Interfaces:"
echo "----------------------------------------"
if timeout 2 ros2 control list_hardware_interfaces 2>/dev/null > /dev/null; then
    ros2 control list_hardware_interfaces 2>/dev/null | sed 's/^/      /'
else
    echo -e "      ${RED}✗ Cannot access hardware interfaces${NC}"
fi
echo ""

# Summary
echo "=========================================="
echo "Summary"
echo "=========================================="
echo ""
echo "If services/topics are missing:"
echo "  - Wait a few seconds for system to initialize"
echo "  - Check launch terminal for errors"
echo "  - Verify system is running: ros2 node list"
echo ""

