#!/bin/bash
# 电机通信问题修复脚本
# 用于诊断和修复 CANopen 电机 SDO 通信失败问题

CAN_IF="can0"

echo "==========================================="
echo "电机通信诊断和修复工具"
echo "==========================================="
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 1. 检查 CAN 接口
echo "1. 检查 CAN 接口状态..."
if ip link show $CAN_IF > /dev/null 2>&1; then
    STATE=$(ip link show $CAN_IF | grep -o "state [A-Z]*" | awk '{print $2}')
    if [ "$STATE" = "UP" ]; then
        echo -e "   ${GREEN}✓${NC} CAN 接口 $CAN_IF 已启动 (状态: $STATE)"
    else
        echo -e "   ${RED}✗${NC} CAN 接口状态异常: $STATE"
        echo "   尝试启动接口..."
        sudo ip link set $CAN_IF type can bitrate 500000
        sudo ip link set $CAN_IF up
    fi
else
    echo -e "   ${RED}✗${NC} CAN 接口 $CAN_IF 不存在"
    exit 1
fi
echo ""

# 2. 检查 CAN 总线流量
echo "2. 检查 CAN 总线流量(2秒监听)..."
TRAFFIC=$(timeout 2s candump $CAN_IF 2>/dev/null | wc -l)
if [ "$TRAFFIC" -gt 0 ]; then
    echo -e "   ${GREEN}✓${NC} 检测到 $TRAFFIC 条 CAN 消息"
else
    echo -e "   ${RED}✗${NC} 没有检测到任何 CAN 流量 - 请检查物理连接!"
    echo "     - 检查 CAN 电缆连接"
    echo "     - 检查 CAN 终端电阻 (120Ω)"
    echo "     - 检查电机驱动器电源"
    exit 1
fi
echo ""

# 3. 发送 NMT 复位命令
echo "3. 发送 NMT 复位命令到所有节点..."
# NMT Reset Communication for all nodes
cansend $CAN_IF 000#8200
sleep 0.5
echo "   已发送 NMT Reset Communication"
echo ""

# 4. 发送 NMT 启动命令
echo "4. 发送 NMT 启动命令..."
# NMT Start Remote Node for all nodes (node ID 0 = all nodes)
cansend $CAN_IF 000#0100
sleep 1
echo "   已发送 NMT Start 命令"
echo ""

# 5. 测试每个电机节点
echo "5. 测试各电机节点 SDO 响应..."
for node_id in 1 2 3; do
    case $node_id in
        1) node_name="hook_joint" ;;
        2) node_name="trolley_joint" ;;
        3) node_name="slewing_joint" ;;
    esac
    
    echo -n "   节点 $node_id ($node_name): "
    
    # 发送 SDO 读取请求 (读取 0x6041 状态字)
    REQ_ID=$(printf '%03X' $((0x600 + node_id)))
    RESP_ID=$(printf '%03X' $((0x580 + node_id)))
    
    # 发送请求
    cansend $CAN_IF ${REQ_ID}#4041600000000000
    
    # 等待响应 (增加超时到 2 秒)
    RESPONSE=$(timeout 2s candump $CAN_IF,${RESP_ID}:7FF -n 1 2>/dev/null)
    
    if [ -n "$RESPONSE" ]; then
        echo -e "${GREEN}✓ 响应${NC}"
        echo "      $RESPONSE"
    else
        echo -e "${RED}✗ 无响应${NC}"
    fi
done
echo ""

# 6. 尝试故障复位
echo "6. 尝试发送故障复位命令(针对节点3)..."
# 对 slewing_joint (node 3) 发送 Fault Reset (0x6040 = 0x80)
# SDO Write: 2B 40 60 00 80 00 00 00
cansend $CAN_IF 603#2B40600080000000
sleep 0.5
echo "   已发送 Fault Reset 到节点 3"
echo ""

# 7. 最终测试
echo "7. 最终通信测试..."
cansend $CAN_IF 603#4041600000000000
FINAL_RESP=$(timeout 2s candump $CAN_IF,583:7FF -n 1 2>/dev/null)

if [ -n "$FINAL_RESP" ]; then
    echo -e "${GREEN}✓ 节点 3 现在响应了!${NC}"
    echo "   $FINAL_RESP"
else
    echo -e "${RED}✗ 节点 3 仍然无响应${NC}"
    echo ""
    echo "==================== 进一步排查建议 ===================="
    echo ""
    echo "问题可能原因:"
    echo "  1. ${YELLOW}电机驱动器电源未打开${NC}"
    echo "     → 检查电机驱动器 LED 指示灯"
    echo "     → 检查 24V/48V 电源供电"
    echo ""
    echo "  2. ${YELLOW}电机驱动器处于故障状态${NC}"
    echo "     → 查看驱动器显示屏错误代码"
    echo "     → 可能需要手动复位(按驱动器复位按钮)"
    echo ""
    echo "  3. ${YELLOW}节点 ID 配置错误${NC}"
    echo "     → 验证电机驱动器的 CANopen 节点 ID 设置"
    echo "     → slewing_joint 应配置为节点 ID = 3"
    echo ""
    echo "  4. ${YELLOW}CAN 波特率不匹配${NC}"
    echo "     → 当前设置: 500 kbit/s"
    echo "     → 验证电机驱动器的 CAN 波特率设置"
    echo ""
    echo "  5. ${YELLOW}电机固件不支持 SDO${NC}"
    echo "     → 某些驱动器可能配置为仅 PDO 模式"
    echo "     → 需要修改 bus.yml: 禁用启动配置"
    echo ""
    echo "======================================================="
fi
echo ""

echo "脚本执行完毕。"
echo "如果问题仍未解决,请运行以下命令查看详细的 CAN 流量:"
echo "  candump -l $CAN_IF"
echo ""
