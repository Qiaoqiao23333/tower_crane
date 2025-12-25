#!/bin/bash

echo "=========================================="
echo "检查节点激活状态（关键诊断）"
echo "=========================================="
echo ""
echo "ros2_canopen 的 SDO 读取需要节点处于 'activated' 状态"
echo "如果节点未激活，SDO 读取会立即返回 false"
echo ""

echo "步骤 1: 检查节点日志（查看是否有激活消息）"
echo "----------------------------------------"
echo "查看最近的日志，寻找："
echo "  - 'activated'"
echo "  - 'Successful activate'"
echo "  - 'Initialisation successful'"
echo ""
echo "运行以下命令查看日志："
echo "  ros2 topic echo /rosout --once | grep -E '(activated|activate|Initialisation)'"
echo ""

echo "步骤 2: 测试 SDO 读取（带详细输出）"
echo "----------------------------------------"
echo "使用正确的服务格式测试..."
echo ""

# 测试读取状态字
echo "测试读取状态字 (0x6041):"
timeout 10 ros2 service call /hook_joint/sdo_read canopen_interfaces/srv/CORead "{index: 6041, subindex: 0}" 2>&1 | tee /tmp/sdo_test_output.txt

echo ""
echo "检查输出..."
if grep -q "success: true" /tmp/sdo_test_output.txt 2>/dev/null; then
    echo "  ✅ SDO 读取成功！"
    grep "data:" /tmp/sdo_test_output.txt
elif grep -q "Could not read from SDO because driver not activated" /tmp/sdo_test_output.txt 2>/dev/null; then
    echo "  ❌ 节点未激活！"
    echo "     这是问题的根源 - 节点需要激活才能处理 SDO"
elif grep -q "timeout\|Timeout\|TIMEOUT" /tmp/sdo_test_output.txt 2>/dev/null; then
    echo "  ❌ SDO 读取超时"
    echo "     可能原因："
    echo "     1. 电机未响应"
    echo "     2. CAN 总线通信问题"
    echo "     3. Lely 驱动未正确处理响应"
else
    echo "  ⚠️  未知错误"
    cat /tmp/sdo_test_output.txt | tail -5
fi

echo ""
echo "步骤 3: 检查 CAN 总线通信"
echo "----------------------------------------"
if command -v candump >/dev/null 2>&1; then
    echo "监听 CAN 总线 2 秒，检查是否有 SDO 通信..."
    timeout 2 candump -L can0,601:7FF,can0,581:7FF 2>/dev/null | head -10 || echo "  无 SDO 通信"
else
    echo "  ⚠️  candump 不可用"
fi

echo ""
echo "步骤 4: 检查节点状态（通过 NMT）"
echo "----------------------------------------"
echo "检查 NMT 状态话题..."
timeout 2 ros2 topic echo /hook_joint/nmt_state --once 2>/dev/null | head -5 || echo "  无法读取 NMT 状态"

echo ""
echo "=========================================="
echo "关键发现"
echo "=========================================="
echo ""
echo "如果看到 'Could not read from SDO because driver not activated':"
echo "  → 节点未激活，需要检查 launch 文件"
echo "  → 节点可能处于 'inactive' 或 'unconfigured' 状态"
echo ""
echo "如果看到超时："
echo "  → 节点已激活，但电机未响应"
echo "  → 检查 CAN 总线通信"
echo "  → 检查电机是否上电"
echo ""

