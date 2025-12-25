#!/bin/bash
# CANopen Diagnostic Script
# This script helps diagnose CAN communication issues

echo "========================================="
echo "CANopen Network Diagnostic Tool"
echo "========================================="
echo ""

CAN_IF="can0"

# 1. Check if CAN interface exists and is UP
echo "1. Checking CAN interface status..."
if ip link show "$CAN_IF" &> /dev/null; then
    echo "   ✓ Interface $CAN_IF exists"
    ip -details link show "$CAN_IF"
    
    if ip link show "$CAN_IF" | grep -q "UP"; then
        echo "   ✓ Interface is UP"
    else
        echo "   ✗ Interface is DOWN"
        echo "   → Run: sudo ip link set $CAN_IF up type can bitrate 500000"
    fi
else
    echo "   ✗ Interface $CAN_IF does not exist"
    echo "   → Check your CAN hardware connection"
    echo "   → For USB-CAN: Check if device is detected with 'lsusb' or 'dmesg | tail'"
fi
echo ""

# 2. Check for CAN traffic
echo "2. Checking for CAN traffic (5 second capture)..."
echo "   Starting candump..."
timeout 5s candump "$CAN_IF" -n 10 2>/dev/null
if [ $? -eq 124 ]; then
    echo "   No frames captured - CAN bus may be silent or disconnected"
else
    echo "   ✓ CAN traffic detected"
fi
echo ""

# 3. Poll each node for NMT state
echo "3. Polling CANopen nodes (NMT state)..."
for NODE_ID in 1 2 3; do
    echo "   Checking Node $NODE_ID..."
    
    # Send NMT state request (0x700 + node_id is the response COB-ID)
    # Node will respond with boot-up message or heartbeat
    
    # Try reading statusword (0x6041) via SDO
    SDO_REQ=$(printf '%03X#4041600000000000' $((0x600 + NODE_ID)))
    SDO_RESP_ID=$(printf '%03X' $((0x580 + NODE_ID)))
    
    echo "      Sending SDO request: $SDO_REQ"
    cansend "$CAN_IF" "$SDO_REQ" 2>/dev/null
    
    RESPONSE=$(timeout 0.5s candump -L "$CAN_IF","$SDO_RESP_ID":7FF 2>/dev/null | tail -n 1)
    
    if [ -n "$RESPONSE" ]; then
        echo "      ✓ Node $NODE_ID responded: $RESPONSE"
    else
        echo "      ✗ Node $NODE_ID did not respond (TIMEOUT)"
        echo "         Possible causes:"
        echo "         - Node is powered off"
        echo "         - Wrong node ID configured"
        echo "         - Node is in error state"
        echo "         - Wrong baud rate (expecting 500 kbps)"
    fi
    sleep 0.2
done
echo ""

# 4. Check error register for each node
echo "4. Reading error register (0x1001) for each node..."
for NODE_ID in 1 2 3; do
    echo "   Node $NODE_ID error register:"
    
    SDO_REQ=$(printf '%03X#4001100000000000' $((0x600 + NODE_ID)))
    SDO_RESP_ID=$(printf '%03X' $((0x580 + NODE_ID)))
    
    cansend "$CAN_IF" "$SDO_REQ" 2>/dev/null
    RESPONSE=$(timeout 0.5s candump -L "$CAN_IF","$SDO_RESP_ID":7FF 2>/dev/null | tail -n 1)
    
    if [ -n "$RESPONSE" ]; then
        echo "      Response: $RESPONSE"
    else
        echo "      No response"
    fi
    sleep 0.2
done
echo ""

echo "========================================="
echo "Diagnostic complete"
echo "========================================="
echo ""
echo "Recommended actions:"
echo "1. If nodes don't respond:"
echo "   - Verify power supply to drives"
echo "   - Check CAN wiring (CANH, CANL, GND)"
echo "   - Verify CAN termination (120Ω at both ends)"
echo "   - Check node ID DIP switches/configuration"
echo "   - Verify baud rate is set to 500 kbps on all nodes"
echo ""
echo "2. To increase SDO timeout in bus.yml:"
echo "   - Edit: crane/tower_crane/config/robot_control/bus.yml"
echo "   - Increase 'sdo_timeout_ms' from 100 to 500 or 1000"
echo ""
echo "3. Use ROS2 built-in diagnostics:"
echo "   ros2 launch tower_crane hardware_bringup_real.launch.py diagnose_canopen:=true"
echo ""



