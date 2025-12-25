#!/bin/bash

echo "=========================================="
echo "电机状态详细诊断"
echo "=========================================="
echo ""

# 检查所有关节的状态
for joint in hook_joint trolley_joint slewing_joint; do
    echo "--- $joint ---"
    
    # 1. 检查状态字 (0x6041)
    echo "1. 状态字 (Statusword 0x6041):"
    ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}" 2>/dev/null | grep -A 5 "data:" || echo "   ❌ 无法读取状态字"
    echo ""
    
    # 2. 检查操作模式 (0x6061)
    echo "2. 操作模式显示 (Mode Display 0x6061):"
    ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 6061, subindex: 0}" 2>/dev/null | grep -A 5 "data:" || echo "   ❌ 无法读取操作模式"
    echo ""
    
    # 3. 检查当前位置 (0x6064)
    echo "3. 当前位置 (Position 0x6064):"
    ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 6064, subindex: 0}" 2>/dev/null | grep -A 5 "data:" || echo "   ❌ 无法读取位置"
    echo ""
    
    # 4. 检查目标位置 (0x607A)
    echo "4. 目标位置 (Target Position 0x607A):"
    ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 607A, subindex: 0}" 2>/dev/null | grep -A 5 "data:" || echo "   ❌ 无法读取目标位置"
    echo ""
    
    # 5. 检查错误寄存器 (0x1001)
    echo "5. 错误寄存器 (Error Register 0x1001):"
    ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 1001, subindex: 0}" 2>/dev/null | grep -A 5 "data:" || echo "   ❌ 无法读取错误寄存器"
    echo ""
    
    # 6. 检查控制字 (0x6040)
    echo "6. 控制字 (Controlword 0x6040):"
    ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: 6040, subindex: 0}" 2>/dev/null | grep -A 5 "data:" || echo "   ❌ 无法读取控制字"
    echo ""
    
    echo "---"
    echo ""
done

echo "=========================================="
echo "关节状态 (ROS2 Joint States)"
echo "=========================================="
ros2 topic echo /joint_states --once 2>/dev/null || echo "无法读取关节状态"

echo ""
echo "=========================================="
echo "控制器状态"
echo "=========================================="
ros2 control list_controllers 2>/dev/null || echo "无法读取控制器状态"

