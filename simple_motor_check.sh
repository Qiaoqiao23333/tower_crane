#!/bin/bash

echo "=========================================="
echo "简单电机状态检查（避免 SDO 超时）"
echo "=========================================="
echo ""

echo "步骤 1: 检查 ROS2 服务是否可用"
echo "----------------------------------------"
for joint in hook_joint trolley_joint slewing_joint; do
    if timeout 1 ros2 service list 2>/dev/null | grep -q "/${joint}/"; then
        echo "  ✅ $joint: 服务可用"
    else
        echo "  ❌ $joint: 服务不可用或超时"
    fi
done

echo ""
echo "步骤 2: 检查关节状态（ROS2 topic）"
echo "----------------------------------------"
echo "这是从 TPDO 读取的位置反馈："
timeout 3 ros2 topic echo /joint_states --once 2>/dev/null | grep -A 10 "position:" || echo "无法读取关节状态"

echo ""
echo "步骤 3: 检查控制器状态"
echo "----------------------------------------"
timeout 2 ros2 control list_controllers 2>/dev/null | head -10 || echo "无法读取控制器状态"

echo ""
echo "步骤 4: 检查 CANopen 节点状态"
echo "----------------------------------------"
timeout 2 ros2 node list 2>/dev/null | grep -E "(canopen|402|device)" | head -10 || echo "无法列出节点"

echo ""
echo "=========================================="
echo "诊断结果"
echo "=========================================="
echo ""
echo "如果关节状态显示位置为 0.0："
echo "  1. 可能是电机真的在位置 0"
echo "  2. 可能是 TPDO 没有传输位置数据"
echo "  3. 可能是位置单位转换问题"
echo ""
echo "如果 SDO 服务超时："
echo "  1. 电机可能没有响应（硬件问题）"
echo "  2. CAN 总线通信可能有问题"
echo "  3. 节点可能处于错误状态"
echo ""
echo "建议："
echo "  1. 检查 CAN 总线连接"
echo "  2. 检查电机是否上电"
echo "  3. 尝试重启 CANopen 系统"
echo "  4. 使用 CAN 工具（candump）直接检查 CAN 通信"
echo ""

