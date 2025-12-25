#!/bin/bash
# RViz Environment Check Script
# Run this to diagnose RViz display issues

echo "========================================="
echo "  RViz2 Environment Diagnostic Check"
echo "========================================="
echo ""

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check 1: RViz2 installed
echo -n "1. Checking if rviz2 is installed... "
if which rviz2 >/dev/null 2>&1; then
    echo -e "${GREEN}✓ PASS${NC}"
    echo "   Path: $(which rviz2)"
else
    echo -e "${RED}✗ FAIL${NC}"
    echo "   rviz2 not found in PATH"
    echo "   Install with: sudo apt install ros-humble-rviz2"
fi
echo ""

# Check 2: DISPLAY variable
echo -n "2. Checking DISPLAY environment variable... "
if [ -z "$DISPLAY" ]; then
    echo -e "${RED}✗ FAIL${NC}"
    echo "   DISPLAY is not set"
    echo "   For local: export DISPLAY=:0"
    echo "   For SSH: connect with 'ssh -X'"
    echo "   For WSL2: export DISPLAY=:0 (with X server running)"
else
    echo -e "${GREEN}✓ PASS${NC}"
    echo "   DISPLAY=$DISPLAY"
fi
echo ""

# Check 3: X server accessibility
echo -n "3. Checking X server accessibility... "
if xdpyinfo >/dev/null 2>&1; then
    echo -e "${GREEN}✓ PASS${NC}"
    echo "   X server is accessible"
else
    echo -e "${RED}✗ FAIL${NC}"
    echo "   Cannot connect to X server"
    echo "   Make sure X server is running"
    if [ ! -z "$DISPLAY" ]; then
        echo "   DISPLAY is set but X server not reachable"
    fi
fi
echo ""

# Check 4: ROS environment
echo -n "4. Checking ROS environment... "
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}✗ FAIL${NC}"
    echo "   ROS environment not sourced"
    echo "   Run: source /opt/ros/humble/setup.bash"
else
    echo -e "${GREEN}✓ PASS${NC}"
    echo "   ROS_DISTRO=$ROS_DISTRO"
fi
echo ""

# Check 5: Workspace sourced
echo -n "5. Checking workspace sourcing... "
if [ -z "$COLCON_PREFIX_PATH" ]; then
    echo -e "${YELLOW}⚠ WARNING${NC}"
    echo "   Workspace not sourced (may be OK if using system install)"
else
    echo -e "${GREEN}✓ PASS${NC}"
    echo "   Workspace sourced"
fi
echo ""

# Check 6: Tower crane package
echo -n "6. Checking tower_crane_moveit_config package... "
if ros2 pkg list 2>/dev/null | grep -q tower_crane_moveit_config; then
    echo -e "${GREEN}✓ PASS${NC}"
    PKG_PATH=$(ros2 pkg prefix tower_crane_moveit_config 2>/dev/null)
    echo "   Package found at: $PKG_PATH"
else
    echo -e "${RED}✗ FAIL${NC}"
    echo "   Package not found"
    echo "   Build with: colcon build --packages-select tower_crane_moveit_config"
    echo "   Source with: source install/setup.bash"
fi
echo ""

# Check 7: OpenGL
echo -n "7. Checking OpenGL/GLX... "
if command -v glxinfo >/dev/null 2>&1; then
    if glxinfo >/dev/null 2>&1; then
        echo -e "${GREEN}✓ PASS${NC}"
        echo "   OpenGL is working"
    else
        echo -e "${RED}✗ FAIL${NC}"
        echo "   glxinfo exists but OpenGL not working"
    fi
else
    echo -e "${YELLOW}⚠ WARNING${NC}"
    echo "   glxinfo not installed (install mesa-utils to test)"
fi
echo ""

# Check 8: RViz plugins
echo -n "8. Checking RViz plugins... "
if dpkg -l | grep -q ros-humble-rviz-default-plugins; then
    echo -e "${GREEN}✓ PASS${NC}"
else
    echo -e "${YELLOW}⚠ WARNING${NC}"
    echo "   RViz default plugins may not be installed"
    echo "   Install with: sudo apt install ros-humble-rviz-default-plugins"
fi
echo ""

# Check 9: MoveIt RViz plugin
echo -n "9. Checking MoveIt RViz plugin... "
if dpkg -l | grep -q ros-humble-moveit-ros-visualization; then
    echo -e "${GREEN}✓ PASS${NC}"
else
    echo -e "${YELLOW}⚠ WARNING${NC}"
    echo "   MoveIt RViz plugin may not be installed"
    echo "   Install with: sudo apt install ros-humble-moveit-ros-visualization"
fi
echo ""

# Summary
echo "========================================="
echo "  Summary"
echo "========================================="
echo ""
echo "Next steps:"
echo ""
echo "If all checks passed, try launching:"
echo "  ros2 launch tower_crane_moveit_config test_rviz_minimal.launch.py"
echo ""
echo "If DISPLAY or X server failed:"
echo "  - Local machine: export DISPLAY=:0"
echo "  - SSH: reconnect with 'ssh -X user@host'"
echo "  - WSL2: Start X server (VcXsrv) and export DISPLAY=:0"
echo ""
echo "If RViz not installed:"
echo "  sudo apt install ros-humble-rviz2 ros-humble-rviz-default-plugins"
echo ""
echo "If package not found:"
echo "  cd /home/qiaoqiaochen/appdata/canros"
echo "  colcon build --packages-select tower_crane_moveit_config"
echo "  source install/setup.bash"
echo ""


