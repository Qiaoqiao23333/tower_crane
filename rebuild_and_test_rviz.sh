#!/bin/bash
# Rebuild tower_crane_moveit_config and test RViz

set -e  # Exit on error

echo "========================================="
echo "  Rebuild and Test RViz"
echo "========================================="
echo ""

# Get the workspace directory (parent of src)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
WS_DIR="$(dirname "$SCRIPT_DIR")"

echo "Workspace: $WS_DIR"
echo ""

# Step 1: Check ROS is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "ROS not sourced. Sourcing /opt/ros/humble/setup.bash..."
    source /opt/ros/humble/setup.bash
fi

echo "ROS Distribution: $ROS_DISTRO"
echo ""

# Step 2: Build the package
echo "========================================="
echo "  Building tower_crane_moveit_config..."
echo "========================================="
cd "$WS_DIR"
colcon build --packages-select tower_crane_moveit_config --symlink-install

if [ $? -ne 0 ]; then
    echo ""
    echo "Build failed! Check the error messages above."
    exit 1
fi

echo ""
echo "Build successful!"
echo ""

# Step 3: Source the workspace
echo "Sourcing workspace..."
source "$WS_DIR/install/setup.bash"
echo ""

# Step 4: Run tests
echo "========================================="
echo "  Running RViz Tests"
echo "========================================="
echo ""

echo "Test 1: Check if package is found"
if ros2 pkg list | grep -q tower_crane_moveit_config; then
    echo "✓ Package found"
else
    echo "✗ Package not found after build!"
    exit 1
fi
echo ""

echo "Test 2: Check launch files exist"
LAUNCH_DIR="$WS_DIR/install/tower_crane_moveit_config/share/tower_crane_moveit_config/launch"
if [ -f "$LAUNCH_DIR/test_rviz_minimal.launch.py" ]; then
    echo "✓ test_rviz_minimal.launch.py exists"
else
    echo "✗ test_rviz_minimal.launch.py not found"
fi

if [ -f "$LAUNCH_DIR/moveit_rviz.launch.py" ]; then
    echo "✓ moveit_rviz.launch.py exists"
else
    echo "✗ moveit_rviz.launch.py not found"
fi
echo ""

# Step 5: Environment check
echo "========================================="
echo "  Environment Check"
echo "========================================="
if [ -f "$SCRIPT_DIR/check_rviz_environment.sh" ]; then
    bash "$SCRIPT_DIR/check_rviz_environment.sh"
else
    echo "check_rviz_environment.sh not found, skipping..."
fi
echo ""

# Step 6: Offer to launch
echo "========================================="
echo "  Ready to Test"
echo "========================================="
echo ""
echo "The package has been built successfully."
echo ""
echo "You can now test RViz with these commands:"
echo ""
echo "  1. Minimal RViz test (RSP + RViz, default view):"
echo "     ros2 launch tower_crane_moveit_config test_rviz_minimal.launch.py"
echo "     Shows: Robot model in basic RViz"
echo ""
echo "  2. RViz with MoveIt config (RSP + RViz with MoveIt panels):"
echo "     ros2 launch tower_crane_moveit_config test_rviz_only.launch.py"
echo "     Shows: Robot model + MoveIt panels"
echo ""
echo "  3. Full demo (RSP + Hardware + MoveIt + RViz):"
echo "     ros2 launch tower_crane_moveit_config demo.launch.py"
echo "     Shows: Everything - ready for motion planning!"
echo ""
echo "  4. Just RViz from full config:"
echo "     ros2 launch tower_crane_moveit_config moveit_rviz.launch.py"
echo "     Note: Needs RSP running separately"
echo ""

# Ask user if they want to launch a test
read -p "Would you like to run a test now? (1/2/3/4/n): " choice

case $choice in
    1)
        echo ""
        echo "Launching minimal RViz test..."
        ros2 launch tower_crane_moveit_config test_rviz_minimal.launch.py
        ;;
    2)
        echo ""
        echo "Launching RViz with config..."
        ros2 launch tower_crane_moveit_config test_rviz_only.launch.py
        ;;
    3)
        echo ""
        echo "Launching full demo..."
        ros2 launch tower_crane_moveit_config demo.launch.py
        ;;
    4)
        echo ""
        echo "Launching RViz only..."
        ros2 launch tower_crane_moveit_config moveit_rviz.launch.py
        ;;
    *)
        echo ""
        echo "No test selected. You can run the commands above manually."
        ;;
esac

