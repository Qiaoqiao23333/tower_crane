#!/bin/bash
# Tower Crane Hardware Startup Script

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# ---------------------------------------------------------
# 1. ROOT USER LOGIC (Setup Hardware Here)
# ---------------------------------------------------------
if [ "$(id -u)" = "0" ]; then
    echo -e "${YELLOW}⚠️  Running as root.${NC}"
    
    # FIX: Configure CAN0 *BEFORE* switching user, while we still have root power!
    echo -e "${BLUE}Configuring CAN hardware as root...${NC}"
    
    # Check if can0 exists
    if ! ip link show can0 >/dev/null 2>&1; then
        echo -e "${RED}❌ ERROR: CAN interface 'can0' not found! Check USB connection.${NC}"
        exit 1
    fi

    # Bring up can0 if it's not up
    # using 'grep -q' is cleaner to avoid "Device busy" errors
    if ! ip link show can0 | grep -q "state UP"; then
        echo -e "${YELLOW}Bringing up can0...${NC}"
        ip link set can0 down 2>/dev/null # Safety reset
        if ip link set can0 up type can bitrate 500000; then
            echo -e "${GREEN}✅ CAN interface configured successfully.${NC}"
        else
            echo -e "${RED}❌ Failed to start CAN interface!${NC}"
            exit 1
        fi
    else
        echo -e "${GREEN}✅ CAN interface is already UP.${NC}"
    fi

    echo -e "${BLUE}Switching to user 'qiaoqiaochen' for ROS2 software layer...${NC}"
    echo ""
    # Pass control to the user, preserving environment
    exec su - qiaoqiaochen -c "bash $0 $*"
fi

# ---------------------------------------------------------
# 2. STANDARD USER LOGIC (Run Software Here)
# ---------------------------------------------------------
# Now running as qiaoqiaochen
echo -e "${GREEN}=========================================="
echo "🏗️  Tower Crane Hardware System"
echo -e "==========================================${NC}"
echo -e "User:          ${BLUE}$(whoami)${NC}"
echo -e "Workspace:     ${BLUE}/home/qiaoqiaochen/appdata/canros${NC}"

# Double check CAN (It should be UP by now because Root handled it)
if ! ip link show can0 | grep -q "state UP"; then
    echo -e "${RED}❌ ERROR: CAN interface is DOWN.${NC}"
    echo "Since you are not root, I cannot bring it up automatically."
    echo "Please run this script as root: sudo ./start_tower_crane.sh"
    exit 1
fi
echo -e "${GREEN}✅ CAN Interface: can0 (Ready)${NC}"

# Navigate to workspace
cd /home/qiaoqiaochen/appdata/canros || {
    echo -e "${RED}❌ ERROR: Could not navigate to workspace!${NC}"
    exit 1
}

# Source ROS2
if [ -f install/setup.bash ]; then
    source install/setup.bash
else
    echo -e "${RED}❌ ERROR: Workspace not built! Run 'colcon build' first.${NC}"
    exit 1
fi

echo -e "${GREEN}=========================================="
echo "🚀 Launching Hardware System..."
echo -e "==========================================${NC}"

# Parse arguments
LAUNCH_ARGS=""
if [ "$1" = "pre_enable" ] || [ "$1" = "enable" ]; then
    echo -e "${GREEN}✅ Pre-enabling motors via CAN before startup${NC}"
    LAUNCH_ARGS="pre_enable_drives:=true"
else
    echo -e "${YELLOW}NOTE: Motors will need manual enablement after startup!${NC}"
    echo -e "${YELLOW}      To auto-enable: sudo $0 pre_enable${NC}"
fi

echo -e "${YELLOW}Press Ctrl+C to stop.${NC}"
echo ""

# Launch
ros2 launch tower_crane hardware_bringup_real.launch.py $LAUNCH_ARGS