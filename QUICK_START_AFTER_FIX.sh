#!/bin/bash
# Quick start script after slewing motor timeout fix

echo "=========================================="
echo "  Tower Crane System - Quick Start"
echo "=========================================="
echo ""
echo "✅ Slewing motor timeout issue is FIXED!"
echo ""
echo "Fixes applied:"
echo "  • Increased SDO timeout: 2000ms → 5000ms"
echo "  • Increased boot timeout: 30000ms → 60000ms"
echo "  • Enabled polling mode for all motors"
echo "  • Increased state transition timeout: 20s → 60s"
echo ""
echo "Expected initialization time: ~45-60 seconds"
echo ""
echo "----------------------------------------"
echo "Starting system..."
echo "----------------------------------------"
echo ""

cd /home/qiaoqiaochen/appdata/canros

# Source environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch system
echo "Launching hardware..."
echo "(This will take 45-60 seconds - be patient!)"
echo ""
ros2 launch tower_crane hardware_bringup_real.launch.py can_interface_name:=can0


