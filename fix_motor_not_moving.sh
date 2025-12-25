#!/bin/bash

echo "=========================================="
echo "电机不移动 - 综合诊断和修复"
echo "=========================================="
echo ""

# 函数：读取 SDO
read_sdo() {
    local joint=$1
    local index=$2
    local subindex=${3:-0}
    ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: ${index}, subindex: ${subindex}}" 2>/dev/null | grep -oP 'data:\s*\K-?[0-9]+'
}

# 函数：解析状态字
parse_statusword() {
    local sw=$1
    local result=""
    
    if [ $((sw & 0x0027)) -eq 39 ]; then
        result="✅ 操作已启用"
    elif [ $((sw & 0x0040)) -ne 0 ]; then
        result="❌ 操作被禁止"
    elif [ $((sw & 0x0008)) -ne 0 ]; then
        result="❌ 故障状态"
    else
        result="⚠️  状态异常"
    fi
    
    echo "$result"
}

echo "步骤 1: 诊断所有电机状态"
echo "----------------------------------------"
for joint in hook_joint trolley_joint slewing_joint; do
    echo ""
    echo "--- $joint ---"
    
    # 读取状态字
    sw=$(read_sdo $joint 6041)
    if [ -n "$sw" ]; then
        sw_hex=$(printf "0x%04X" $sw)
        echo "  状态字: $sw_hex ($sw) - $(parse_statusword $sw)"
        
        # 如果未使能，尝试使能
        if [ $((sw & 0x0027)) -ne 39 ]; then
            echo "  ⚠️  电机未使能，尝试使能..."
            ros2 service call /${joint}/enable std_srvs/srv/Trigger >/dev/null 2>&1
            sleep 1
            
            # 再次检查
            sw_new=$(read_sdo $joint 6041)
            if [ -n "$sw_new" ]; then
                sw_new_hex=$(printf "0x%04X" $sw_new)
                echo "  使能后状态字: $sw_new_hex ($sw_new) - $(parse_statusword $sw_new)"
            fi
        fi
    else
        echo "  ❌ 无法读取状态字"
    fi
    
    # 读取操作模式
    mode=$(read_sdo $joint 6061)
    if [ -n "$mode" ]; then
        echo "  操作模式: $mode"
        if [ "$mode" -ne 1 ] && [ "$mode" -ne 7 ]; then
            echo "  ⚠️  不是位置模式，尝试切换到位置模式..."
            ros2 service call /${joint}/position_mode std_srvs/srv/Trigger >/dev/null 2>&1
            sleep 1
        fi
    fi
    
    # 读取当前位置和目标位置
    pos=$(read_sdo $joint 6064)
    target=$(read_sdo $joint 607A)
    if [ -n "$pos" ] && [ -n "$target" ]; then
        echo "  当前位置: $pos"
        echo "  目标位置: $target"
        diff=$((target - pos))
        if [ ${diff#-} -lt 10 ]; then
            echo "  ⚠️  目标位置和当前位置非常接近"
        fi
    fi
done

echo ""
echo "步骤 2: 检查控制器状态"
echo "----------------------------------------"
ros2 control list_controllers 2>/dev/null | grep -A 3 forward_position_controller || echo "无法读取控制器状态"

echo ""
echo "步骤 3: 检查关节状态"
echo "----------------------------------------"
echo "当前关节位置（ROS2 单位）："
timeout 2 ros2 topic echo /joint_states --once 2>/dev/null | grep -A 10 "position:" | head -15 || echo "无法读取关节状态"

echo ""
echo "步骤 4: 建议的测试命令"
echo "----------------------------------------"
echo ""
echo "如果电机已使能，尝试直接发送位置命令："
echo ""
echo "# 测试 hook_joint (节点 1)"
echo "ros2 service call /hook_joint/target canopen_interfaces/srv/COTargetDouble \"{target: 1000.0}\""
echo ""
echo "# 测试 trolley_joint (节点 2)"
echo "ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble \"{target: 1000.0}\""
echo ""
echo "# 测试 slewing_joint (节点 3)"
echo "ros2 service call /slewing_joint/target canopen_interfaces/srv/COTargetDouble \"{target: 1000.0}\""
echo ""
echo "或者使用 action 接口："
echo "ros2 action send_goal /forward_position_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \""
echo "trajectory:"
echo "  joint_names: [slewing_joint, trolley_joint, hook_joint]"
echo "  points:"
echo "  - positions: [0.1, 0.1, 0.1]"
echo "    time_from_start:"
echo "      sec: 5"
echo "\" --feedback"
echo ""

