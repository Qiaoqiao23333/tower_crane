#!/bin/bash
# Check communication status of all CANopen nodes

echo "========================================="
echo "CANopen Node Communication Check"
echo "========================================="
echo ""

CAN_IF="can0"

# Check if CAN interface is up
echo "1. Checking CAN interface status..."
if ip link show "$CAN_IF" 2>/dev/null | grep -q "UP"; then
    echo "   ✓ CAN interface $CAN_IF is UP"
else
    echo "   ✗ CAN interface $CAN_IF is DOWN"
    echo "   → Run: sudo ip link set $CAN_IF up type can bitrate 500000"
    exit 1
fi
echo ""

# Check each node
echo "2. Testing node communication..."
for NODE_ID in 1 2 3; do
    NODE_NAME=""
    case $NODE_ID in
        1) NODE_NAME="hook_joint" ;;
        2) NODE_NAME="trolley_joint" ;;
        3) NODE_NAME="slewing_joint" ;;
    esac
    
    echo "   Testing Node $NODE_ID ($NODE_NAME)..."
    
    # Try to read Statusword (0x6041) via SDO
    SDO_REQ=$(printf '%03X#4041600000000000' $((0x600 + NODE_ID)))
    SDO_RESP_ID=$(printf '%03X' $((0x580 + NODE_ID)))
    
    # Send SDO request
    cansend "$CAN_IF" "$SDO_REQ" 2>/dev/null
    
    # Wait for response
    RESPONSE=$(timeout 1s candump -L "$CAN_IF","$SDO_RESP_ID":7FF 2>/dev/null | tail -n 1)
    
    if [ -n "$RESPONSE" ]; then
        echo "      ✓ Node $NODE_ID responded: $RESPONSE"
    else
        echo "      ✗ Node $NODE_ID did NOT respond (TIMEOUT)"
        echo "         → Check: Power, CAN wiring, Node ID configuration"
    fi
    sleep 0.3
done
echo ""

# Check ROS2 topics
echo "3. Checking ROS2 joint states..."
for JOINT in hook_joint trolley_joint slewing_joint; do
    echo "   Checking $JOINT..."
    STATE=$(timeout 2s ros2 topic echo /${JOINT}/joint_states --once 2>/dev/null | grep -A 1 "position:" | tail -1 | awk '{print $2}')
    if [ -n "$STATE" ]; then
        echo "      ✓ $JOINT state available: position=$STATE"
    else
        echo "      ✗ $JOINT state NOT available (timeout or no data)"
    fi
done
echo ""

# Check NMT states
echo "4. Checking NMT states..."
for JOINT in hook_joint trolley_joint slewing_joint; do
    echo "   Checking $JOINT NMT state..."
    NMT=$(timeout 2s ros2 topic echo /${JOINT}/nmt_state --once 2>/dev/null | grep "state:" | awk '{print $2}')
    if [ -n "$NMT" ]; then
        echo "      ✓ $JOINT NMT state: $NMT"
    else
        echo "      ✗ $JOINT NMT state NOT available"
    fi
done
echo ""

echo "========================================="
echo "Summary"
echo "========================================="
echo ""
echo "If nodes are not responding:"
echo "  1. Check power supply to all motor drives"
echo "  2. Verify CAN bus wiring (CANH, CANL, GND)"
echo "  3. Check CAN termination (120Ω at both ends)"
echo "  4. Verify node IDs match configuration (1, 2, 3)"
echo "  5. Check baud rate (should be 500 kbps)"
echo ""
echo "If ROS2 topics are not available:"
echo "  - The CANopen driver may not be running"
echo "  - Check: ros2 node list"
echo "  - Check: ros2 topic list | grep joint"
echo ""



