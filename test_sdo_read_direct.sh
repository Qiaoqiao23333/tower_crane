#!/bin/bash

echo "=========================================="
echo "直接测试 SDO 读取（使用正确的服务格式）"
echo "=========================================="
echo ""

echo "服务类型: canopen_interfaces/srv/CORead"
echo "只需要 index 和 subindex（不需要 canopen_datatype）"
echo ""

echo "测试 1: 读取状态字 (0x6041)"
echo "----------------------------------------"
echo "命令:"
echo "  ros2 service call /hook_joint/sdo_read canopen_interfaces/srv/CORead \"{index: 6041, subindex: 0}\""
echo ""
timeout 10 ros2 service call /hook_joint/sdo_read canopen_interfaces/srv/CORead "{index: 6041, subindex: 0}" 2>&1

echo ""
echo ""
echo "测试 2: 读取位置 (0x6064)"
echo "----------------------------------------"
echo "命令:"
echo "  ros2 service call /hook_joint/sdo_read canopen_interfaces/srv/CORead \"{index: 6064, subindex: 0}\""
echo ""
timeout 10 ros2 service call /hook_joint/sdo_read canopen_interfaces/srv/CORead "{index: 6064, subindex: 0}" 2>&1

echo ""
echo ""
echo "=========================================="
echo "如果仍然超时，检查："
echo "=========================================="
echo ""
echo "1. 节点是否激活（lifecycle 状态）"
echo "2. Lely 驱动是否正确初始化"
echo "3. CAN 总线通信是否正常"
echo ""
echo "使用 candump 检查 CAN 通信："
echo "  candump can0 | grep -E '(601|581)'"
echo ""

