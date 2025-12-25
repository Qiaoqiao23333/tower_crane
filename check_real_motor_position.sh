#!/bin/bash

echo "=========================================="
echo "检查电机实际位置（原始编码器值）"
echo "=========================================="
echo ""

echo "直接从电机读取位置（0x6064 - Position Actual Value）"
echo "这些是编码器原始值，不是 ROS2 单位"
echo ""

for joint in hook_joint trolley_joint slewing_joint; do
    echo "--- $joint ---"
    
    # 检查服务是否可用
    if ! timeout 2 ros2 service list 2>/dev/null | grep -q "/${joint}/sdo_read"; then
        echo "  ❌ 服务 /${joint}/sdo_read 不可用"
        echo ""
        continue
    fi
    
    # 读取实际位置（编码器值）- 添加超时
    result=$(timeout 5 ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 6064, subindex: 0}" 2>/dev/null | grep -oP 'data:\s*\K-?[0-9]+')
    if [ -n "$result" ]; then
        echo "  实际位置 (0x6064): $result (编码器计数)"
        
        # 如果是 0，可能是真的在 0 位置，或者没有读取到
        if [ "$result" -eq 0 ]; then
            echo "  ⚠️  位置为 0 - 可能是："
            echo "     a) 电机真的在位置 0"
            echo "     b) TPDO 没有传输位置数据"
            echo "     c) 位置读取失败"
        fi
    else
        echo "  ❌ 无法读取位置"
    fi
    
    # 读取目标位置 - 添加超时
    target=$(timeout 5 ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 607A, subindex: 0}" 2>/dev/null | grep -oP 'data:\s*\K-?[0-9]+')
    if [ -n "$target" ]; then
        echo "  目标位置 (0x607A): $target (编码器计数)"
        
        if [ -n "$result" ] && [ "$target" -ne "$result" ]; then
            diff=$((target - result))
            echo "  ✅ 目标位置和实际位置有差异: $diff"
            echo "     电机应该会移动"
        elif [ -n "$result" ] && [ "$target" -eq "$result" ]; then
            echo "  ⚠️  目标位置 = 实际位置，电机不会移动"
        fi
    fi
    
    # 读取状态字 - 添加超时
    sw=$(timeout 5 ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}" 2>/dev/null | grep -oP 'data:\s*\K[0-9]+')
    if [ -n "$sw" ]; then
        sw_hex=$(printf "0x%04X" $sw)
        echo "  状态字 (0x6041): $sw_hex"
        
        # 检查关键位
        if [ $((sw & 0x0027)) -eq 39 ]; then
            echo "  ✅ 操作已启用"
        else
            echo "  ❌ 未使能 - 状态: $sw_hex"
        fi
        
        if [ $((sw & 0x0400)) -ne 0 ]; then
            echo "  ✅ 目标已到达"
        fi
    fi
    
    echo ""
done

echo "=========================================="
echo "检查 ROS2 关节状态"
echo "=========================================="
ros2 topic echo /joint_states --once 2>/dev/null | grep -A 5 "position:" || echo "无法读取关节状态"

echo ""
echo "=========================================="
echo "诊断建议"
echo "=========================================="
echo ""
echo "如果电机实际位置（编码器值）不是 0，但 ROS2 显示 0.0："
echo "  → 位置单位转换可能有问题"
echo "  → 检查 scale_pos_from_dev 配置"
echo ""
echo "如果电机实际位置和目标位置都是 0："
echo "  → 位置命令可能没有发送到电机"
echo "  → 检查 RPDO 配置和传输"
echo ""
echo "如果目标位置和实际位置不同，但电机不移动："
echo "  → 电机可能未使能"
echo "  → 检查状态字和控制字"
echo "  → 检查 New Setpoint 位是否设置"
echo ""

