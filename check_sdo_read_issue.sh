#!/bin/bash

echo "=========================================="
echo "SDO 读取问题诊断"
echo "=========================================="
echo ""

echo "根据 CANopen 标准，读取 0x6064 的 SDO 消息格式："
echo ""
echo "【请求消息】Client → Server (Node ID 1):"
echo "  COB-ID: 0x601 (0x600 + 1)"
echo "  data[0]: 0x40 (Expedited read request)"
echo "  data[1]: 0x64 (Index low: 0x6064)"
echo "  data[2]: 0x60 (Index high: 0x6064)"
echo "  data[3]: 0x00 (Subindex)"
echo "  data[4-7]: 0x00 (Not used)"
echo "  完整消息: 601#4060640000000000"
echo ""
echo "【成功响应】Server → Client (Node ID 1):"
echo "  COB-ID: 0x581 (0x580 + 1)"
echo "  data[0]: 0x43 (4 bytes data)"
echo "  data[1]: 0x64 (Index low)"
echo "  data[2]: 0x60 (Index high)"
echo "  data[3]: 0x00 (Subindex)"
echo "  data[4-7]: Position value (little-endian)"
echo "  示例: 581#4360640010270000 (位置 = 0x00002710 = 10000)"
echo ""
echo "【错误响应】Server → Client (Node ID 1):"
echo "  COB-ID: 0x581 (0x580 + 1)"
echo "  data[0]: 0x80 (SDO abort)"
echo "  data[1-3]: Index and subindex"
echo "  data[4-7]: Abort code"
echo "  示例: 581#8060640005040000 (超时错误 0x05040000)"
echo ""

echo "=========================================="
echo "检查 ros2_canopen SDO 读取实现问题"
echo "=========================================="
echo ""

echo "步骤 1: 检查节点激活状态"
echo "----------------------------------------"
echo "ros2_canopen 节点必须处于 'activated' 状态才能处理 SDO"
echo "如果节点未激活，SDO 读取会立即返回 false"
echo ""

# 检查节点
for joint in hook_joint trolley_joint slewing_joint; do
    node_name=$(timeout 1 ros2 node list 2>/dev/null | grep "$joint" | head -1)
    if [ -n "$node_name" ]; then
        echo "  $joint: 节点存在 ($node_name)"
    else
        echo "  $joint: ❌ 节点不存在"
    fi
done

echo ""
echo "步骤 2: 检查服务类型"
echo "----------------------------------------"
echo "检查 SDO 服务使用的消息类型..."
for joint in hook_joint trolley_joint slewing_joint; do
    if timeout 1 ros2 service list 2>/dev/null | grep -q "/${joint}/sdo_read"; then
        echo "  ✅ $joint/sdo_read: 服务存在"
        # 尝试获取服务类型
        svc_type=$(timeout 1 ros2 service type /${joint}/sdo_read 2>/dev/null)
        if [ -n "$svc_type" ]; then
            echo "     类型: $svc_type"
            if echo "$svc_type" | grep -q "COReadID"; then
                echo "     ✅ 使用 COReadID (正确)"
            elif echo "$svc_type" | grep -q "CORead"; then
                echo "     ⚠️  使用 CORead (可能不匹配)"
            fi
        fi
    fi
done

echo ""
echo "步骤 3: 检查 Lely 驱动状态"
echo "----------------------------------------"
echo "ros2_canopen 使用 Lely 库进行 SDO 通信"
echo "如果 Lely 驱动未正确初始化，SDO 响应可能丢失"
echo ""
echo "检查 CAN 总线是否有 SDO 通信..."
if command -v candump >/dev/null 2>&1; then
    echo "监听 SDO 通信 2 秒..."
    timeout 2 candump -L can0,601:7FF,can0,581:7FF 2>/dev/null | head -5 || echo "  无 SDO 通信"
else
    echo "  ⚠️  candump 不可用"
fi

echo ""
echo "步骤 4: 检查超时设置"
echo "----------------------------------------"
echo "当前 bus.yml 中的 sdo_timeout_ms 设置："
grep "sdo_timeout_ms" /home/qiaoqiaochen/appdata/canros/src/crane/tower_crane/config/robot_control/bus.yml 2>/dev/null | head -3 || echo "  无法读取配置"

echo ""
echo "=========================================="
echo "可能的原因"
echo "=========================================="
echo ""
echo "1. 【节点未激活】"
echo "   → ros2_canopen 节点需要处于 'active' 状态"
echo "   → 检查 launch 文件是否正确激活节点"
echo "   → 查看日志中是否有 'activated' 消息"
echo ""
echo "2. 【Lely 驱动未初始化】"
echo "   → Lely 驱动可能未正确连接到 CAN 总线"
echo "   → 检查 CAN 接口配置"
echo "   → 检查 master 配置"
echo ""
echo "3. 【电机未响应】"
echo "   → 电机可能未上电或离线"
echo "   → 使用 candump 检查是否有心跳消息"
echo "   → 检查 CAN 总线连接"
echo ""
echo "4. 【SDO 响应丢失】"
echo "   → CAN 总线可能过载"
echo "   → SDO 响应可能被其他消息覆盖"
echo "   → 检查 CAN 总线错误计数"
echo ""
echo "=========================================="
echo "建议的解决方案"
echo "=========================================="
echo ""
echo "1. 重启 CANopen 系统（清除内部状态）"
echo "2. 检查电机是否上电"
echo "3. 使用 candump 直接检查 CAN 通信"
echo "4. 检查 launch 文件中的节点激活"
echo "5. 如果问题持续，检查 Lely 驱动日志"
echo ""

