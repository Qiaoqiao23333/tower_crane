#!/bin/bash

echo "=========================================="
echo "正确读取电机状态（使用 CORead 服务）"
echo "=========================================="
echo ""
echo "服务类型: canopen_interfaces/srv/CORead"
echo "只需要 index 和 subindex"
echo ""

echo "读取 hook_joint (节点 1) 状态："
echo "----------------------------------------"

echo "1. 状态字 (0x6041 - Statusword):"
result=$(timeout 10 ros2 service call /hook_joint/sdo_read canopen_interfaces/srv/CORead "{index: 6041, subindex: 0}" 2>&1)
if echo "$result" | grep -q "success: true"; then
    data=$(echo "$result" | grep -oP 'data:\s*\K[0-9]+')
    sw_hex=$(printf "0x%04X" $data)
    echo "  ✅ 成功: $sw_hex ($data)"
    if [ $((data & 0x0027)) -eq 39 ]; then
        echo "     ✅ 电机已使能（Operation Enabled）"
    else
        echo "     ❌ 电机未使能"
    fi
else
    echo "  ❌ 失败"
    echo "$result" | grep -E "(error|Error|ERROR|timeout|Timeout)" | head -3
fi

echo ""
echo "2. 操作模式 (0x6061 - Mode Display):"
result=$(timeout 10 ros2 service call /hook_joint/sdo_read canopen_interfaces/srv/CORead "{index: 6061, subindex: 0}" 2>&1)
if echo "$result" | grep -q "success: true"; then
    data=$(echo "$result" | grep -oP 'data:\s*\K[0-9]+')
    echo "  ✅ 成功: $data"
    if [ "$data" -eq 1 ] || [ "$data" -eq 7 ]; then
        echo "     ✅ 位置模式"
    else
        echo "     ⚠️  不是位置模式 (当前: $data)"
    fi
else
    echo "  ❌ 失败"
    echo "$result" | grep -E "(error|Error|ERROR|timeout|Timeout)" | head -3
fi

echo ""
echo "3. 实际位置 (0x6064 - Position Actual):"
result=$(timeout 10 ros2 service call /hook_joint/sdo_read canopen_interfaces/srv/CORead "{index: 6064, subindex: 0}" 2>&1)
if echo "$result" | grep -q "success: true"; then
    data=$(echo "$result" | grep -oP 'data:\s*\K-?[0-9]+')
    echo "  ✅ 成功: $data (编码器计数)"
else
    echo "  ❌ 失败"
    echo "$result" | grep -E "(error|Error|ERROR|timeout|Timeout)" | head -3
fi

echo ""
echo "4. 目标位置 (0x607A - Target Position):"
result=$(timeout 10 ros2 service call /hook_joint/sdo_read canopen_interfaces/srv/CORead "{index: 607A, subindex: 0}" 2>&1)
if echo "$result" | grep -q "success: true"; then
    data=$(echo "$result" | grep -oP 'data:\s*\K-?[0-9]+')
    echo "  ✅ 成功: $data (编码器计数)"
else
    echo "  ❌ 失败"
    echo "$result" | grep -E "(error|Error|ERROR|timeout|Timeout)" | head -3
fi

echo ""
echo "=========================================="
echo "关于 0x1799 错误"
echo "=========================================="
echo ""
echo "0x1799 是 RPDO mapping 相关的对象"
echo "这个错误通常可以忽略，不影响基本功能"
echo "可能是系统在尝试配置 PDO 映射时读取了不存在的对象"
echo ""
echo "关键是要能读取："
echo "  ✅ 0x6041 (Statusword) - 电机状态"
echo "  ✅ 0x6064 (Position) - 当前位置"
echo "  ✅ 0x607A (Target Position) - 目标位置"
echo ""

