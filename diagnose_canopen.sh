#!/bin/bash
# CANopen Device Diagnostic Script
# Checks statusword and error codes for all three motor nodes

CAN_IF="can0"

echo "========================================"
echo "CANopen Device Diagnostic Tool"
echo "========================================"
echo ""

# Function to read SDO via cansend/candump
read_sdo() {
    local node_id=$1
    local index=$2
    local subindex=$3
    local name=$4
    
    # Send SDO upload request (0x40 = upload request)
    local tx_cob=$((0x600 + node_id))
    local rx_cob=$((0x580 + node_id))
    
    # Format: 40 [index_low] [index_high] [subindex] 00 00 00 00
    local index_low=$(printf "%02X" $((index & 0xFF)))
    local index_high=$(printf "%02X" $(((index >> 8) & 0xFF)))
    local sub=$(printf "%02X" $subindex)
    
    cansend ${CAN_IF} $(printf "%03X#40%s%s%s00000000" ${tx_cob} ${index_low} ${index_high} ${sub}) 2>/dev/null
    
    # Wait for response with timeout
    local response=$(timeout 0.5s candump -n 1 ${CAN_IF},${rx_cob}:7FF 2>/dev/null | tail -n 1)
    
    if [ -n "$response" ]; then
        echo "[Node ${node_id}] ${name} (0x${index}:${subindex}): ${response}"
    else
        echo "[Node ${node_id}] ${name} (0x${index}:${subindex}): <NO RESPONSE>"
    fi
}

# Check each node
for node_id in 1 2 3; do
    echo "----------------------------------------"
    echo "Checking Node ${node_id}"
    echo "----------------------------------------"
    
    # Read Statusword (0x6041)
    read_sdo ${node_id} 0x6041 0 "Statusword"
    
    # Read Error Register (0x1001)
    read_sdo ${node_id} 0x1001 0 "Error Register"
    
    # Read Error Code (0x603F)
    read_sdo ${node_id} 0x603F 0 "Error Code"
    
    # Read Modes of Operation Display (0x6061)
    read_sdo ${node_id} 0x6061 0 "Mode Display"
    
    # Read Position Actual Value (0x6064)
    read_sdo ${node_id} 0x6064 0 "Position Actual"
    
    echo ""
done

echo "========================================"
echo "Checking CAN bus statistics"
echo "========================================"
ip -s link show can0

echo ""
echo "========================================"
echo "Recent CAN traffic (5 seconds)"
echo "========================================"
timeout 5s candump can0 2>/dev/null || echo "No CAN traffic detected"
