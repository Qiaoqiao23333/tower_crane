#!/bin/bash

echo "=========================================="
echo "检查 CAN 总线通信和节点在线状态"
echo "=========================================="
echo ""

CAN_IFACE="can0"

# 检查 CAN 接口是否存在
echo "步骤 1: 检查 CAN 接口"
echo "----------------------------------------"
if ip link show $CAN_IFACE >/dev/null 2>&1; then
    echo "✅ CAN 接口 $CAN_IFACE 存在"
    ip link show $CAN_IFACE | grep -E "(state|UP|DOWN)" || echo "  状态未知"
else
    echo "❌ CAN 接口 $CAN_IFACE 不存在"
    echo "   请检查 CAN 硬件连接"
    exit 1
fi

echo ""
echo "步骤 2: 检查节点心跳（节点是否在线）"
echo "----------------------------------------"
echo "监听心跳消息 3 秒..."
echo "如果节点在线，应该看到心跳消息（0x701, 0x702, 0x703）"
echo ""

if command -v candump >/dev/null 2>&1; then
    timeout 3 candump -L $CAN_IFACE,701:7FF,$CAN_IFACE,702:7FF,$CAN_IFACE,703:7FF 2>/dev/null | head -20
    if [ ${PIPESTATUS[0]} -eq 0 ]; then
        echo ""
        echo "✅ 检测到心跳消息 - 节点在线"
    else
        echo ""
        echo "❌ 未检测到心跳消息 - 节点可能离线"
    fi
else
    echo "⚠️  candump 工具不可用，无法检查心跳"
fi

echo ""
echo "步骤 3: 检查 ROS2 节点状态"
echo "----------------------------------------"
echo "CANopen 相关节点："
timeout 2 ros2 node list 2>/dev/null | grep -E "(canopen|402|device|master)" | head -10 || echo "无法列出节点"

echo ""
echo "步骤 4: 检查 ROS2 服务"
echo "----------------------------------------"
echo "CANopen 相关服务（前 20 个）："
timeout 2 ros2 service list 2>/dev/null | grep -E "(hook|trolley|slewing|canopen)" | head -20 || echo "无法列出服务"

echo ""
echo "步骤 5: 检查控制器状态"
echo "----------------------------------------"
timeout 2 ros2 control list_controllers 2>/dev/null || echo "无法读取控制器状态"

echo ""
echo "=========================================="
echo "诊断结果和建议"
echo "=========================================="
echo ""

if ! command -v candump >/dev/null 2>&1; then
    echo "⚠️  建议安装 can-utils 工具："
    echo "   sudo apt-get install can-utils"
    echo ""
fi

echo "如果所有 SDO 读取都超时："
echo "  1. 检查 CAN 总线连接"
echo "  2. 检查电机是否上电"
echo "  3. 检查节点 ID 配置是否正确"
echo "  4. 尝试重启 CANopen 系统"
echo ""
echo "如果心跳消息正常但 SDO 超时："
echo "  1. 可能是 SDO 超时时间设置太短"
echo "  2. 可能是 CAN 总线负载过高"
echo "  3. 尝试增加 sdo_timeout_ms"
echo ""
echo "如果完全没有心跳消息："
echo "  1. 电机可能未上电"
echo "  2. CAN 总线可能断开"
echo "  3. 节点 ID 配置可能错误"
echo ""

