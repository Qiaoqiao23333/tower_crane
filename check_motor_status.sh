#!/bin/bash
# Motor Status Diagnostic Script
# Checks the status of all three CANopen motors

set -e

CAN_IF="${1:-can0}"

echo "============================================"
echo "CANopen Motor Status Diagnostic"
echo "CAN Interface: ${CAN_IF}"
echo "============================================"
echo ""

# Function to read SDO
read_sdo() {
    local node_id=$1
    local index=$2
    local subindex=$3
    local description=$4
    
    echo -n "Node ${node_id} - ${description}: "
    
    # Send SDO upload request
    cansend ${CAN_IF} $(printf '%03X#40%02X%02X%02X00000000' $((0x600+node_id)) $((index & 0xFF)) $((index >> 8)) ${subindex})
    
    # Wait and capture response
    response=$(timeout 0.5s candump -L ${CAN_IF},$(printf '%03X' $((0x580+node_id))):7FF | tail -n 1 || echo "timeout")
    
    if [[ "$response" == "timeout" ]] || [[ -z "$response" ]]; then
        echo "NO RESPONSE (motor may be offline)"
    else
        # Extract data bytes from response
        data=$(echo "$response" | awk '{print $3}' | cut -d'#' -f2)
        echo "0x${data}"
    fi
}

echo "Checking Hook Motor (Node ID 1):"
echo "─────────────────────────────────────────"
read_sdo 1 0x6041 0x00 "Statusword (0x6041)"
read_sdo 1 0x6061 0x00 "Operation Mode (0x6061)"
read_sdo 1 0x603F 0x00 "Error Code (0x603F)"
read_sdo 1 0x6064 0x00 "Position Actual (0x6064)"
echo ""

echo "Checking Trolley Motor (Node ID 2):"
echo "─────────────────────────────────────────"
read_sdo 2 0x6041 0x00 "Statusword (0x6041)"
read_sdo 2 0x6061 0x00 "Operation Mode (0x6061)"
read_sdo 2 0x603F 0x00 "Error Code (0x603F)"
read_sdo 2 0x6064 0x00 "Position Actual (0x6064)"
echo ""

echo "Checking Slewing Motor (Node ID 3):"
echo "─────────────────────────────────────────"
read_sdo 3 0x6041 0x00 "Statusword (0x6041)"
read_sdo 3 0x6061 0x00 "Operation Mode (0x6061)"
read_sdo 3 0x603F 0x00 "Error Code (0x603F)"
read_sdo 3 0x6064 0x00 "Position Actual (0x6064)"
echo ""

echo "============================================"
echo "Statusword Decoder:"
echo "  0x0027 = Operation Enabled (READY)"
echo "  0x0250 = Switch On Disabled"
echo "  0x0237 = Quick Stop Active"
echo "  0x0008 = Fault"
echo "  0x0040 = Switch On Disabled"
echo "============================================"
echo ""
echo "Operation Mode Decoder:"
echo "  1 = Profile Position Mode"
echo "  3 = Profile Velocity Mode"
echo "  7 = Interpolated Position Mode"
echo "============================================"


