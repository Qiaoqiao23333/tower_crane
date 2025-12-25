#!/bin/bash

echo "=========================================="
echo "检查 TPDO 和关节状态（不依赖 SDO）"
echo "=========================================="
echo ""

echo "即使 SDO 读取失败，TPDO 可能仍然在工作"
echo "TPDO 通过 CAN 总线周期性发送位置和状态数据"
echo ""

echo "步骤 1: 检查关节状态（从 TPDO 读取）"
echo "----------------------------------------"
echo "这是从 TPDO 读取的位置反馈："
timeout 3 ros2 topic echo /joint_states --once 2>/dev/null | grep -A 15 "position:" || echo "无法读取关节状态"

echo ""
echo "步骤 2: 检查各个关节的 joint_states"
echo "----------------------------------------"
for joint in hook_joint trolley_joint slewing_joint; do
    echo "--- $joint ---"
    timeout 2 ros2 topic echo /${joint}/joint_states --once 2>/dev/null | head -20 || echo "无法读取"
    echo ""
done

echo "步骤 3: 检查 TPDO 原始数据"
echo "----------------------------------------"
for joint in hook_joint trolley_joint slewing_joint; do
    echo "--- $joint ---"
    if timeout 1 ros2 topic list 2>/dev/null | grep -q "/${joint}/tpdo"; then
        echo "  TPDO 话题存在，监听 2 秒..."
        timeout 2 ros2 topic echo /${joint}/tpdo 2>/dev/null | head -15 || echo "  无数据"
    else
        echo "  ⚠️  TPDO 话题不存在"
    fi
    echo ""
done

echo "步骤 4: 检查 NMT 状态"
echo "----------------------------------------"
for joint in hook_joint trolley_joint slewing_joint; do
    echo "--- $joint ---"
    timeout 2 ros2 topic echo /${joint}/nmt_state --once 2>/dev/null | head -5 || echo "无法读取"
    echo ""
done

echo "=========================================="
echo "诊断结果"
echo "=========================================="
echo ""
echo "如果关节状态有数据（位置不是 0.0）："
echo "  → TPDO 在工作，位置反馈正常"
echo "  → 问题可能是位置单位转换或电机未使能"
echo ""
echo "如果关节状态位置是 0.0："
echo "  → 可能是电机真的在位置 0"
echo "  → 或者 TPDO 没有传输位置数据"
echo ""
echo "如果 SDO 读取失败但 TPDO 有数据："
echo "  → 可以使用 TPDO 数据，不依赖 SDO"
echo "  → 但无法通过 SDO 配置电机"
echo ""

