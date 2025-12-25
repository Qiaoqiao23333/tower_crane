#!/bin/bash

echo "=========================================="
echo "检查电机使能状态（关键诊断）"
echo "=========================================="
echo ""

echo "如果电机未使能，位置命令不会执行！"
echo ""

# 检查服务是否可用
check_service() {
    local joint=$1
    if timeout 1 ros2 service list 2>/dev/null | grep -q "/${joint}/"; then
        return 0
    else
        return 1
    fi
}

# 尝试使能电机
enable_motor() {
    local joint=$1
    echo "  尝试使能 $joint..."
    timeout 3 ros2 service call /${joint}/enable std_srvs/srv/Trigger >/dev/null 2>&1
    sleep 1
}

for joint in hook_joint trolley_joint slewing_joint; do
    echo "--- $joint ---"
    
    if ! check_service $joint; then
        echo "  ❌ 服务不可用"
        echo ""
        continue
    fi
    
    # 尝试读取状态字（带超时）
    sw=$(timeout 3 ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}" 2>/dev/null | grep -oP 'data:\s*\K[0-9]+')
    
    if [ -n "$sw" ]; then
        sw_hex=$(printf "0x%04X" $sw)
        echo "  状态字: $sw_hex ($sw)"
        
        # 检查是否使能
        if [ $((sw & 0x0027)) -eq 39 ]; then
            echo "  ✅ 电机已使能（Operation Enabled）"
        else
            echo "  ❌ 电机未使能！"
            echo "     状态分析："
            
            if [ $((sw & 0x0008)) -ne 0 ]; then
                echo "     - 故障状态（Fault）"
                echo "     尝试恢复..."
                timeout 3 ros2 service call /${joint}/recover std_srvs/srv/Trigger >/dev/null 2>&1
                sleep 1
            fi
            
            if [ $((sw & 0x0040)) -ne 0 ]; then
                echo "     - 操作被禁止（Operation Disabled）"
            fi
            
            echo "     尝试使能..."
            enable_motor $joint
            
            # 再次检查
            sw_new=$(timeout 3 ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}" 2>/dev/null | grep -oP 'data:\s*\K[0-9]+')
            if [ -n "$sw_new" ]; then
                sw_new_hex=$(printf "0x%04X" $sw_new)
                echo "     使能后状态字: $sw_new_hex"
                if [ $((sw_new & 0x0027)) -eq 39 ]; then
                    echo "     ✅ 使能成功！"
                else
                    echo "     ❌ 使能失败"
                fi
            fi
        fi
    else
        echo "  ⚠️  无法读取状态字（可能超时）"
        echo "     尝试使能电机..."
        enable_motor $joint
    fi
    
    echo ""
done

echo "=========================================="
echo "检查操作模式"
echo "=========================================="
for joint in hook_joint trolley_joint slewing_joint; do
    if check_service $joint; then
        mode=$(timeout 3 ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 6061, subindex: 0}" 2>/dev/null | grep -oP 'data:\s*\K[0-9]+')
        if [ -n "$mode" ]; then
            echo "$joint: 模式 $mode"
            if [ "$mode" -ne 1 ] && [ "$mode" -ne 7 ]; then
                echo "  ⚠️  不是位置模式，尝试切换..."
                timeout 3 ros2 service call /${joint}/position_mode std_srvs/srv/Trigger >/dev/null 2>&1
            fi
        fi
    fi
done

echo ""
echo "=========================================="
echo "总结"
echo "=========================================="
echo ""
echo "如果电机未使能，位置命令不会执行！"
echo "请确保所有电机的状态字为 0x0027（Operation Enabled）"
echo ""

