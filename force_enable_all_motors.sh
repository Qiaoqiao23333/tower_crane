#!/bin/bash

echo "=========================================="
echo "强制使能所有电机"
echo "=========================================="
echo ""

for joint in hook_joint trolley_joint slewing_joint; do
    echo "--- 处理 $joint ---"
    
    # 步骤 1: 初始化
    echo "  1. 初始化..."
    timeout 5 ros2 service call /${joint}/init std_srvs/srv/Trigger >/dev/null 2>&1
    sleep 1
    
    # 步骤 2: 恢复（如果有故障）
    echo "  2. 恢复（如果有故障）..."
    timeout 5 ros2 service call /${joint}/recover std_srvs/srv/Trigger >/dev/null 2>&1
    sleep 1
    
    # 步骤 3: 切换到位置模式
    echo "  3. 切换到位置模式..."
    timeout 5 ros2 service call /${joint}/position_mode std_srvs/srv/Trigger >/dev/null 2>&1
    sleep 1
    
    # 步骤 4: 使能
    echo "  4. 使能电机..."
    timeout 5 ros2 service call /${joint}/enable std_srvs/srv/Trigger >/dev/null 2>&1
    sleep 2
    
    # 步骤 5: 验证状态字
    echo "  5. 检查状态字..."
    sw=$(timeout 3 ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}" 2>/dev/null | grep -oP 'data:\s*\K[0-9]+')
    if [ -n "$sw" ]; then
        sw_hex=$(printf "0x%04X" $sw)
        echo "     状态字: $sw_hex"
        if [ $((sw & 0x0027)) -eq 39 ]; then
            echo "     ✅ 电机已使能"
        else
            echo "     ❌ 电机未使能，状态: $sw_hex"
        fi
    else
        echo "     ⚠️  无法读取状态字"
    fi
    
    echo ""
done

echo "=========================================="
echo "完成！现在尝试发送位置命令"
echo "=========================================="
echo ""

