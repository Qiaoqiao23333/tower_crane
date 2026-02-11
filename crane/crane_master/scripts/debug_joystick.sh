#!/bin/bash

echo "=========================================="
echo "摇杆控制调试脚本"
echo "=========================================="
echo ""

echo "1. 检查ROS 2节点..."
echo "----------------------------------------"
ros2 node list | grep -E "(joystick|canopen)" || echo "未找到相关节点"
echo ""

echo "2. 检查话题..."
echo "----------------------------------------"
echo "目标位置话题:"
ros2 topic list | grep target_position
echo ""

echo "3. 检查摇杆节点日志 (最近10行)..."
echo "----------------------------------------"
ros2 node info /crane_joystick_driver 2>/dev/null || echo "摇杆节点未运行"
echo ""

echo "4. 检查话题数据流..."
echo "----------------------------------------"
echo "监听 /slewing/target_position (5秒)..."
timeout 5 ros2 topic echo /slewing/target_position --once 2>/dev/null || echo "无数据或话题不存在"
echo ""

echo "5. 检查串口设备..."
echo "----------------------------------------"
if [ -e /dev/ttyUSB0 ]; then
    ls -l /dev/ttyUSB0
    echo "串口设备存在"
else
    echo "❌ /dev/ttyUSB0 不存在"
    echo "可用串口设备:"
    ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "未找到USB串口设备"
fi
echo ""

echo "6. 检查CANopen节点状态..."
echo "----------------------------------------"
for ns in hoist trolley slewing; do
    echo "检查 $ns 节点:"
    ros2 node info /$ns/canopen_ros2_node 2>/dev/null | head -5 || echo "  $ns 节点未运行"
done
echo ""

echo "7. 检查电机当前位置..."
echo "----------------------------------------"
for topic in /hoist/position /trolley/position /slewing/position; do
    echo "监听 $topic:"
    timeout 2 ros2 topic echo $topic --once 2>/dev/null || echo "  无数据"
done
echo ""

echo "=========================================="
echo "调试完成"
echo "=========================================="
