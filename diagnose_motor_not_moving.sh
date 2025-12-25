#!/bin/bash

echo "=========================================="
echo "电机不移动诊断脚本"
echo "=========================================="
echo ""

echo "步骤 1: 检查电机状态字（Statusword 0x6041）"
echo "状态字解析："
echo "  0x0027 = 操作已启用 (Operation Enabled)"
echo "  0x0023 = 已启用 (Enabled)"
echo "  0x0040 = 操作被禁止 (Operation Disabled)"
echo "  0x0008 = 故障 (Fault)"
echo ""

for joint in hook_joint trolley_joint slewing_joint; do
    echo "--- $joint ---"
    result=$(ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}" 2>/dev/null | grep -oP 'data:\s*\K[0-9]+')
    if [ -n "$result" ]; then
        status_hex=$(printf "0x%04X" $result)
        echo "  状态字: $status_hex ($result)"
        
        # 解析状态字
        if [ $((result & 0x0027)) -eq 39 ]; then
            echo "  ✅ 电机已使能并处于操作状态"
        elif [ $((result & 0x0040)) -ne 0 ]; then
            echo "  ❌ 电机操作被禁止 - 需要使能"
        elif [ $((result & 0x0008)) -ne 0 ]; then
            echo "  ❌ 电机处于故障状态 - 需要恢复"
        else
            echo "  ⚠️  电机状态异常"
        fi
    else
        echo "  ❌ 无法读取状态字"
    fi
    echo ""
done

echo "步骤 2: 检查操作模式（Mode Display 0x6061）"
echo "模式值："
echo "  1 = Profile Position Mode (位置模式)"
echo "  3 = Profile Velocity Mode (速度模式)"
echo "  7 = Interpolated Position Mode (插补位置模式)"
echo ""

for joint in hook_joint trolley_joint slewing_joint; do
    echo "--- $joint ---"
    result=$(ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 6061, subindex: 0}" 2>/dev/null | grep -oP 'data:\s*\K[0-9]+')
    if [ -n "$result" ]; then
        echo "  操作模式: $result"
        if [ "$result" -eq 1 ]; then
            echo "  ✅ 位置模式 (正确)"
        elif [ "$result" -eq 7 ]; then
            echo "  ✅ 插补位置模式 (正确)"
        else
            echo "  ❌ 不是位置模式 - 需要切换到位置模式"
        fi
    else
        echo "  ❌ 无法读取操作模式"
    fi
    echo ""
done

echo "步骤 3: 检查当前位置和目标位置"
echo ""

for joint in hook_joint trolley_joint slewing_joint; do
    echo "--- $joint ---"
    
    # 当前位置
    pos=$(ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 6064, subindex: 0}" 2>/dev/null | grep -oP 'data:\s*\K-?[0-9]+')
    if [ -n "$pos" ]; then
        echo "  当前位置: $pos"
    else
        echo "  当前位置: ❌ 无法读取"
    fi
    
    # 目标位置
    target=$(ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 607A, subindex: 0}" 2>/dev/null | grep -oP 'data:\s*\K-?[0-9]+')
    if [ -n "$target" ]; then
        echo "  目标位置: $target"
        if [ -n "$pos" ]; then
            diff=$((target - pos))
            if [ ${diff#-} -lt 10 ]; then
                echo "  ⚠️  目标位置和当前位置非常接近，可能不会移动"
            else
                echo "  ✅ 目标位置和当前位置有差异，应该会移动"
            fi
        fi
    else
        echo "  目标位置: ❌ 无法读取"
    fi
    echo ""
done

echo "步骤 4: 检查 ROS2 关节状态"
echo ""
ros2 topic echo /joint_states --once 2>/dev/null | head -20 || echo "无法读取关节状态"

echo ""
echo "=========================================="
echo "建议的修复步骤："
echo "=========================================="
echo ""
echo "如果电机未使能，运行："
echo "  ros2 service call /hook_joint/enable std_srvs/srv/Trigger"
echo "  ros2 service call /trolley_joint/enable std_srvs/srv/Trigger"
echo "  ros2 service call /slewing_joint/enable std_srvs/srv/Trigger"
echo ""
echo "如果电机处于故障状态，运行："
echo "  ros2 service call /hook_joint/recover std_srvs/srv/Trigger"
echo "  ros2 service call /trolley_joint/recover std_srvs/srv/Trigger"
echo "  ros2 service call /slewing_joint/recover std_srvs/srv/Trigger"
echo ""
echo "如果操作模式不正确，运行："
echo "  ros2 service call /hook_joint/position_mode std_srvs/srv/Trigger"
echo "  ros2 service call /trolley_joint/position_mode std_srvs/srv/Trigger"
echo "  ros2 service call /slewing_joint/position_mode std_srvs/srv/Trigger"
echo ""

