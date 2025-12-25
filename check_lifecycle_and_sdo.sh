#!/bin/bash

echo "=========================================="
echo "检查节点生命周期状态和 SDO 通信"
echo "=========================================="
echo ""

echo "步骤 1: 检查节点生命周期状态"
echo "----------------------------------------"
echo "ros2_canopen 使用 lifecycle 节点，需要处于 'active' 状态"
echo ""

# 检查是否有 lifecycle 状态话题
for joint in hook_joint trolley_joint slewing_joint; do
    echo "--- $joint ---"
    
    # 检查是否有 lifecycle 状态话题
    lc_state_topic="/${joint}/transition_event"
    if timeout 1 ros2 topic list 2>/dev/null | grep -q "$lc_state_topic"; then
        echo "  ✅ 生命周期话题存在"
    else
        echo "  ⚠️  生命周期话题不存在（可能不是 lifecycle 节点）"
    fi
    
    # 尝试读取当前状态
    echo "  检查服务响应..."
    timeout 3 ros2 service call /${joint}/sdo_read canopen_interfaces/srv/CORead "{index: 6041, subindex: 0, canopen_datatype: 3}" 2>&1 | head -10
    echo ""
done

echo "步骤 2: 检查 CAN 总线上的实际 SDO 通信"
echo "----------------------------------------"
if command -v candump >/dev/null 2>&1 && command -v cansend >/dev/null 2>&1; then
    echo "使用 cansend 直接发送 SDO 读取请求..."
    echo ""
    echo "发送请求到节点 1 (hook_joint):"
    echo "  601#4060640000000000"
    cansend can0 601#4060640000000000 2>/dev/null || echo "  ❌ 发送失败"
    
    echo ""
    echo "监听响应 1 秒..."
    timeout 1 candump -L can0,581:7FF 2>/dev/null | head -5 || echo "  ⚠️  无响应"
    
    echo ""
    echo "如果看到 0x581 响应，说明电机在响应"
    echo "如果没有响应，说明电机未响应或 CAN 总线问题"
else
    echo "⚠️  cansend/candump 不可用"
    echo "   安装: sudo apt-get install can-utils"
fi

echo ""
echo "步骤 3: 检查服务类型匹配"
echo "----------------------------------------"
echo "从节点信息看，服务类型是: canopen_interfaces/srv/CORead"
echo "检查服务定义..."
for joint in hook_joint trolley_joint slewing_joint; do
    svc_type=$(timeout 1 ros2 service type /${joint}/sdo_read 2>/dev/null)
    if [ -n "$svc_type" ]; then
        echo "  $joint: $svc_type"
        if echo "$svc_type" | grep -q "CORead"; then
            echo "    ✅ 服务类型正确"
        fi
    fi
done

echo ""
echo "步骤 4: 尝试使用正确的服务格式"
echo "----------------------------------------"
echo "根据服务类型 CORead，需要提供 canopen_datatype"
echo ""
echo "尝试读取状态字 (0x6041, uint16_t = type 3):"
timeout 5 ros2 service call /hook_joint/sdo_read canopen_interfaces/srv/CORead "{index: 6041, subindex: 0, canopen_datatype: 3}" 2>&1 | head -15

echo ""
echo "尝试读取位置 (0x6064, int32_t = type 5):"
timeout 5 ros2 service call /hook_joint/sdo_read canopen_interfaces/srv/CORead "{index: 6064, subindex: 0, canopen_datatype: 5}" 2>&1 | head -15

echo ""
echo "=========================================="
echo "关键发现"
echo "=========================================="
echo ""
echo "服务类型是 CORead，不是 COReadID"
echo "CORead 需要指定 canopen_datatype:"
echo "  - uint8_t:  1"
echo "  - int8_t:   2"
echo "  - uint16_t: 3"
echo "  - int16_t:  4"
echo "  - uint32_t: 5"
echo "  - int32_t:  6"
echo ""
echo "0x6041 (Statusword) = uint16_t = type 3"
echo "0x6064 (Position)   = int32_t  = type 6"
echo ""

