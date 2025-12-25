#!/bin/bash
# Script to manually enable motors after hardware startup
# This is needed because automatic initialization is disabled to prevent blocking

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /home/qiaoqiaochen/appdata/canros/install/setup.bash

echo "=========================================="
echo "启用电机 / Enabling Motors"
echo "=========================================="
echo ""
echo "由于自动初始化会阻塞controller_manager，已被禁用。"
echo "Automatic initialization is disabled to prevent blocking controller_manager."
echo "正在手动启用电机... / Manually enabling motors..."
echo ""

# List of motors to initialize
MOTORS=("hook_joint" "trolley_joint" "slewing_joint")

# Function to enable a single motor
enable_motor() {
    local motor=$1
    echo "----------------------------------------"
    echo "正在初始化: $motor / Initializing: $motor"
    echo "----------------------------------------"
    
    # Step 1: Initialize motor
    echo "  步骤 1: 初始化 / Step 1: Init"
    timeout 5 ros2 service call /${motor}/init std_srvs/srv/Trigger 2>&1 | grep -E "(success|Failed)" || echo "    ⚠️  超时或服务不可用 / Timeout or service unavailable"
    sleep 0.5
    
    # Step 2: Enable motor
    echo "  步骤 2: 使能 / Step 2: Enable"
    timeout 5 ros2 service call /${motor}/enable std_srvs/srv/Trigger 2>&1 | grep -E "(success|Failed)" || echo "    ⚠️  超时或服务不可用 / Timeout or service unavailable"
    sleep 0.5
    
    echo "  ✓ 完成 / Done"
    echo ""
}

# Check if ros2_control_node is running
if ! pgrep -f "ros2_control_node" > /dev/null; then
    echo "❌ 错误: ros2_control_node 未运行！"
    echo "❌ Error: ros2_control_node is not running!"
    echo ""
    echo "请先启动硬件："
    echo "Please start hardware first:"
    echo "  ros2 launch tower_crane hardware_bringup_real.launch.py"
    exit 1
fi

# Wait a bit for services to be available
echo "等待服务可用... / Waiting for services to become available..."
sleep 2

# Enable all motors
for motor in "${MOTORS[@]}"; do
    enable_motor "$motor"
done

echo "=========================================="
echo "完成！所有电机已启用 / Complete! All motors enabled"
echo "=========================================="
echo ""
echo "检查电机状态 / Check motor status:"
echo "  ros2 topic echo /joint_states --once"
echo ""
echo "发送位置命令 / Send position command:"
echo "  ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble \"{target: 0.1}\""
echo ""

