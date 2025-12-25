#!/bin/bash

echo "=========================================="
echo "快速检查电机状态（使用 CAN 直接读取）"
echo "=========================================="
echo ""

# 检查 CAN 接口
CAN_IFACE="can0"
if ! ip link show $CAN_IFACE >/dev/null 2>&1; then
    echo "❌ CAN 接口 $CAN_IFACE 不存在"
    exit 1
fi

echo "使用 CAN 工具直接读取电机位置..."
echo ""

# 函数：通过 CAN 读取 SDO
read_sdo_can() {
    local node_id=$1
    local index=$2
    local subindex=${3:-0}
    
    # SDO 请求 COB-ID = 0x600 + node_id
    local cob_id=$(printf "%03X" $((0x600 + node_id)))
    
    # SDO 上传请求：40 + index(2字节) + subindex + 4字节0
    local index_hex=$(printf "%04X" $index)
    local index_low=${index_hex:2:2}
    local index_high=${index_hex:0:2}
    
    # 发送 SDO 读取请求
    local cmd="${cob_id}#40${index_low}${index_high}0${subindex}00000000"
    
    # 监听响应（COB-ID = 0x580 + node_id）
    local resp_cob=$(printf "%03X" $((0x580 + node_id)))
    
    # 使用 timeout 和 candump 读取响应
    timeout 2 candump -L $CAN_IFACE,$resp_cob:7FF 2>/dev/null | head -1 | while read line; do
        # 解析响应数据
        echo "$line" | grep -oP '[0-9A-F]{2}\s+[0-9A-F]{2}\s+[0-9A-F]{2}\s+[0-9A-F]{2}\s+[0-9A-F]{2}\s+[0-9A-F]{2}\s+[0-9A-F]{2}\s+[0-9A-F]{2}' | awk '{print $5$6$7$8}'
    done
    
    # 发送请求
    cansend $CAN_IFACE $cmd >/dev/null 2>&1
    sleep 0.1
}

echo "检查节点 1 (hook_joint)..."
echo "  尝试读取位置 (0x6064)..."
# 简化：直接使用 cansend 和 candump
cansend $CAN_IFACE 601#4060640000000000 >/dev/null 2>&1
sleep 0.2
timeout 1 candump -L $CAN_IFACE,581:7FF 2>/dev/null | head -1 || echo "  ⚠️  无响应"

echo ""
echo "检查节点 2 (trolley_joint)..."
cansend $CAN_IFACE 602#4060640000000000 >/dev/null 2>&1
sleep 0.2
timeout 1 candump -L $CAN_IFACE,582:7FF 2>/dev/null | head -1 || echo "  ⚠️  无响应"

echo ""
echo "检查节点 3 (slewing_joint)..."
cansend $CAN_IFACE 603#4060640000000000 >/dev/null 2>&1
sleep 0.2
timeout 1 candump -L $CAN_IFACE,583:7FF 2>/dev/null | head -1 || echo "  ⚠️  无响应"

echo ""
echo "=========================================="
echo "检查心跳消息（确认节点在线）"
echo "=========================================="
echo "监听心跳 2 秒..."
timeout 2 candump -L $CAN_IFACE,701:7FF,$CAN_IFACE,702:7FF,$CAN_IFACE,703:7FF 2>/dev/null | head -10 || echo "无心跳消息"

echo ""
echo "=========================================="
echo "建议：如果 SDO 读取超时，尝试："
echo "=========================================="
echo "1. 检查电机是否上电"
echo "2. 检查 CAN 总线连接"
echo "3. 检查节点 ID 配置"
echo "4. 尝试重启 CANopen 系统"
echo ""

