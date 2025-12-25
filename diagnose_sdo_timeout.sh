#!/bin/bash

echo "=========================================="
echo "SDO 读取超时诊断"
echo "=========================================="
echo ""

echo "根据 CANopen 标准，读取 0x6064 (Actual Position) 的 SDO 请求格式："
echo ""
echo "请求 (Client → Server):"
echo "  COB-ID: 0x601 (0x600 + Node ID 1)"
echo "  data[0]: 0x40 (Expedited read request)"
echo "  data[1]: 0x64 (Index low byte: 0x6064)"
echo "  data[2]: 0x60 (Index high byte: 0x6064)"
echo "  data[3]: 0x00 (Subindex)"
echo "  data[4-7]: 0x00 (Not used)"
echo ""
echo "成功响应 (Server → Client):"
echo "  COB-ID: 0x581 (0x580 + Node ID 1)"
echo "  data[0]: 0x43 (4 bytes data)"
echo "  data[1]: 0x64 (Index low byte)"
echo "  data[2]: 0x60 (Index high byte)"
echo "  data[3]: 0x00 (Subindex)"
echo "  data[4-7]: Position value (little-endian)"
echo ""
echo "错误响应 (Server → Client):"
echo "  COB-ID: 0x581 (0x580 + Node ID 1)"
echo "  data[0]: 0x80 (SDO abort)"
echo "  data[1-3]: Index and subindex"
echo "  data[4-7]: Abort code (e.g., 0x05040000 = timeout)"
echo ""

echo "=========================================="
echo "检查 ros2_canopen SDO 服务实现"
echo "=========================================="
echo ""

# 检查服务是否存在
echo "步骤 1: 检查 SDO 服务是否存在"
echo "----------------------------------------"
for joint in hook_joint trolley_joint slewing_joint; do
    if timeout 1 ros2 service list 2>/dev/null | grep -q "/${joint}/sdo_read"; then
        echo "  ✅ $joint/sdo_read: 服务存在"
    else
        echo "  ❌ $joint/sdo_read: 服务不存在"
    fi
done

echo ""
echo "步骤 2: 检查节点状态"
echo "----------------------------------------"
echo "CANopen 相关节点："
timeout 2 ros2 node list 2>/dev/null | grep -E "(canopen|402|device)" | head -10 || echo "无法列出节点"

echo ""
echo "步骤 3: 检查 CAN 总线通信（使用 candump）"
echo "----------------------------------------"
if command -v candump >/dev/null 2>&1; then
    echo "监听 SDO 通信 3 秒..."
    echo "发送 SDO 读取请求并监听响应..."
    echo ""
    
    # 在后台启动 candump
    timeout 3 candump -L can0,581:7FF,can0,582:7FF,can0,583:7FF 2>/dev/null &
    DUMP_PID=$!
    
    # 发送 SDO 读取请求（通过 ROS2 服务）
    echo "尝试读取 hook_joint 位置..."
    timeout 2 ros2 service call /hook_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6064, subindex: 0}" >/dev/null 2>&1 &
    
    sleep 3
    kill $DUMP_PID 2>/dev/null
    
    echo ""
    echo "如果看到 0x581 响应，说明电机在响应"
    echo "如果没有任何响应，说明电机未响应或通信问题"
else
    echo "⚠️  candump 不可用，无法检查 CAN 通信"
    echo "   安装: sudo apt-get install can-utils"
fi

echo ""
echo "步骤 4: 检查节点激活状态"
echo "----------------------------------------"
echo "ros2_canopen 节点需要处于 'active' 状态才能处理 SDO"
echo "检查节点状态..."

# 尝试检查生命周期状态
for joint in hook_joint trolley_joint slewing_joint; do
    node_name=$(timeout 1 ros2 node list 2>/dev/null | grep "$joint" | head -1)
    if [ -n "$node_name" ]; then
        echo "  $joint: 节点存在"
        # 尝试获取节点信息
        timeout 1 ros2 node info "$node_name" 2>/dev/null | grep -E "(state|lifecycle)" | head -3 || echo "    无法获取状态"
    else
        echo "  $joint: 节点不存在"
    fi
done

echo ""
echo "=========================================="
echo "可能的原因和解决方案"
echo "=========================================="
echo ""
echo "1. 节点未激活"
echo "   → ros2_canopen 节点需要处于 'active' 状态"
echo "   → 检查 launch 文件是否正确激活节点"
echo ""
echo "2. CAN 总线通信问题"
echo "   → 电机可能未上电或离线"
echo "   → 检查 CAN 总线连接"
echo "   → 使用 candump 检查是否有 CAN 消息"
echo ""
echo "3. SDO 超时设置太短"
echo "   → 当前设置: sdo_timeout_ms: 10000 (10秒)"
echo "   → 如果仍然超时，可能是硬件问题"
echo ""
echo "4. Lely 驱动未正确处理响应"
echo "   → ros2_canopen 使用 Lely 库"
echo "   → 如果 Lely 驱动未正确初始化，SDO 响应可能丢失"
echo ""
echo "建议："
echo "  1. 重启 CANopen 系统"
echo "  2. 检查电机是否上电"
echo "  3. 使用 candump 直接检查 CAN 通信"
echo "  4. 检查 launch 文件中的节点激活"
echo ""

