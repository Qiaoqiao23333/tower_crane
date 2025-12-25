#!/bin/bash
# 重新编译并启动塔吊硬件系统
# Rebuild and launch tower crane hardware system

set -e  # 出错时退出

echo "=========================================="
echo "  塔吊硬件系统启动脚本"
echo "  Tower Crane Hardware Startup Script"
echo "=========================================="
echo ""

# 切换到工作空间根目录
cd /home/qiaoqiaochen/appdata/canros

echo "步骤 1/5: 清理旧的构建文件..."
echo "Step 1/5: Cleaning old build files..."
rm -rf build/tower_crane install/tower_crane
echo "✓ 清理完成"
echo ""

echo "步骤 2/5: 重新编译 tower_crane 包..."
echo "Step 2/5: Rebuilding tower_crane package..."
colcon build --packages-select tower_crane --cmake-clean-cache
if [ $? -eq 0 ]; then
    echo "✓ 编译成功"
else
    echo "✗ 编译失败，请检查错误信息"
    exit 1
fi
echo ""

echo "步骤 3/5: 加载环境变量..."
echo "Step 3/5: Sourcing environment..."
source install/setup.bash
echo "✓ 环境变量已加载"
echo ""

echo "步骤 4/5: 检查 CAN 接口状态..."
echo "Step 4/5: Checking CAN interface status..."
if ip link show can0 | grep -q "state UP"; then
    echo "✓ CAN0 接口正常"
else
    echo "✗ CAN0 接口未启动，尝试启动..."
    sudo ip link set can0 up type can bitrate 1000000
fi
echo ""

echo "步骤 5/5: 启动硬件系统..."
echo "Step 5/5: Launching hardware system..."
echo ""
echo "提示: 如果卡在设备启动，请按 Ctrl+C 并使用带预启动选项的命令:"
echo "Tip: If stuck on device boot, press Ctrl+C and use pre-enable option:"
echo "     ros2 launch tower_crane hardware_bringup_real.launch.py pre_enable_drives:=true"
echo ""
echo "=========================================="
echo ""

# 启动系统
ros2 launch tower_crane hardware_bringup_real.launch.py

