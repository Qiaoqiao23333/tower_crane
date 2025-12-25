#!/bin/bash

echo "=========================================="
echo "检查 TPDO 数据传输"
echo "=========================================="
echo ""

echo "步骤 1: 持续监听 TPDO 数据（5秒）"
echo "----------------------------------------"
for joint in hook_joint trolley_joint slewing_joint; do
    echo "--- $joint ---"
    timeout 5 ros2 topic echo /${joint}/tpdo 2>/dev/null | head -30 || echo "  无数据"
    echo ""
done

echo "步骤 2: 检查 CAN 总线上的 TPDO 消息"
echo "----------------------------------------"
if command -v candump >/dev/null 2>&1; then
    echo "监听 TPDO COB-IDs:"
    echo "  节点 1 (hook): 0x181"
    echo "  节点 2 (trolley): 0x182"
    echo "  节点 3 (slewing): 0x183"
    echo ""
    echo "监听 3 秒..."
    timeout 3 candump -L can0,181:7FF,can0,182:7FF,can0,183:7FF 2>/dev/null | head -20 || echo "  无 TPDO 消息"
else
    echo "  ⚠️  candump 不可用"
fi

echo ""
echo "步骤 3: 检查 RPDO 是否发送位置命令"
echo "----------------------------------------"
if command -v candump >/dev/null 2>&1; then
    echo "监听 RPDO COB-IDs:"
    echo "  节点 1 (hook): 0x201"
    echo "  节点 2 (trolley): 0x202"
    echo "  节点 3 (slewing): 0x203"
    echo ""
    echo "发送一个位置命令，然后监听 3 秒..."
    echo "（在另一个终端运行位置命令）"
    timeout 3 candump -L can0,201:7FF,can0,202:7FF,can0,203:7FF 2>/dev/null | head -20 || echo "  无 RPDO 消息"
else
    echo "  ⚠️  candump 不可用"
fi

echo ""
echo "步骤 4: 检查 SYNC 帧"
echo "----------------------------------------"
if command -v candump >/dev/null 2>&1; then
    echo "SYNC 帧 COB-ID: 0x080"
    echo "监听 2 秒..."
    timeout 2 candump -L can0,080:7FF 2>/dev/null | head -10 || echo "  无 SYNC 帧"
    echo ""
    echo "如果看到 SYNC 帧，说明 master 在发送同步信号"
    echo "如果没看到，RPDO 可能不会传输（transmission type 0x01 需要 SYNC）"
else
    echo "  ⚠️  candump 不可用"
fi

echo ""
echo "=========================================="
echo "关键发现"
echo "=========================================="
echo ""
echo "如果 TPDO 没有数据："
echo "  → 电机可能没有发送位置反馈"
echo "  → 或者 TPDO 没有正确配置"
echo ""
echo "如果 RPDO 没有数据："
echo "  → 位置命令可能没有发送到电机"
echo "  → 或者 SYNC 帧没有发送（RPDO transmission type 0x01 需要 SYNC）"
echo ""

