#!/bin/bash

echo "=========================================="
echo "测试位置命令传递"
echo "=========================================="
echo ""

# 检查控制器是否正在发送位置命令
echo "步骤 1: 检查控制器状态"
echo "----------------------------------------"
ros2 control list_controllers 2>/dev/null | grep -A 5 forward_position_controller || echo "无法读取控制器状态"
echo ""

# 检查关节状态，看目标位置是否更新
echo "步骤 2: 检查关节状态（位置命令）"
echo "----------------------------------------"
echo "监听关节状态 5 秒，查看位置命令是否更新..."
timeout 5 ros2 topic echo /joint_states 2>/dev/null | grep -E "(name|position)" | head -20 || echo "无法读取关节状态"
echo ""

# 检查电机实际接收到的目标位置
echo "步骤 3: 检查电机实际目标位置 (0x607A)"
echo "----------------------------------------"
for joint in hook_joint trolley_joint slewing_joint; do
    echo "--- $joint ---"
    result=$(ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 607A, subindex: 0}" 2>/dev/null | grep -oP 'data:\s*\K-?[0-9]+')
    if [ -n "$result" ]; then
        echo "  目标位置 (0x607A): $result"
    else
        echo "  ❌ 无法读取目标位置"
    fi
done
echo ""

# 检查控制字，看 New Setpoint 位是否设置
echo "步骤 4: 检查控制字 (0x6040) - 查看 New Setpoint 位"
echo "----------------------------------------"
echo "控制字解析："
echo "  0x000F = Enable Operation (无 New Setpoint)"
echo "  0x001F = Enable Operation + New Setpoint (应该触发运动)"
echo ""
for joint in hook_joint trolley_joint slewing_joint; do
    echo "--- $joint ---"
    result=$(ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 6040, subindex: 0}" 2>/dev/null | grep -oP 'data:\s*\K[0-9]+')
    if [ -n "$result" ]; then
        cw_hex=$(printf "0x%04X" $result)
        echo "  控制字: $cw_hex ($result)"
        if [ $((result & 0x0010)) -ne 0 ]; then
            echo "  ✅ New Setpoint 位已设置 (应该触发运动)"
        else
            echo "  ❌ New Setpoint 位未设置 (不会触发运动)"
        fi
        if [ $((result & 0x000F)) -eq 15 ]; then
            echo "  ✅ Enable Operation 已设置"
        else
            echo "  ❌ Enable Operation 未正确设置"
        fi
    else
        echo "  ❌ 无法读取控制字"
    fi
done
echo ""

# 检查状态字，看电机是否准备好
echo "步骤 5: 检查状态字 (0x6041) - 查看电机状态"
echo "----------------------------------------"
for joint in hook_joint trolley_joint slewing_joint; do
    echo "--- $joint ---"
    result=$(ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}" 2>/dev/null | grep -oP 'data:\s*\K[0-9]+')
    if [ -n "$result" ]; then
        sw_hex=$(printf "0x%04X" $result)
        echo "  状态字: $sw_hex ($result)"
        
        # 检查关键位
        if [ $((result & 0x0027)) -eq 39 ]; then
            echo "  ✅ 操作已启用 (Operation Enabled)"
        else
            echo "  ❌ 未处于操作已启用状态"
        fi
        
        if [ $((result & 0x0400)) -ne 0 ]; then
            echo "  ✅ 目标已到达"
        else
            echo "  ⚠️  目标未到达"
        fi
        
        if [ $((result & 0x0008)) -ne 0 ]; then
            echo "  ❌ 故障状态！"
        fi
    else
        echo "  ❌ 无法读取状态字"
    fi
done
echo ""

echo "=========================================="
echo "诊断完成"
echo "=========================================="
echo ""
echo "如果 New Setpoint 位未设置，位置命令不会触发运动。"
echo "如果电机未使能，需要先使能电机。"
echo ""

