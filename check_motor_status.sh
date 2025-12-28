#!/bin/bash
# Motor Status Check - Reads statusword and key parameters via ROS2 services

echo "=========================================="
echo "Motor Status Check"
echo "=========================================="
echo ""

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# Function to read SDO via ROS2 service
read_sdo() {
    local joint=$1
    local index=$2
    local description=$3
    
    local result=$(timeout 3 ros2 service call /${joint}/sdo_read canopen_interfaces/srv/COReadID "{index: ${index}, subindex: 0}" 2>/dev/null)
    
    if echo "$result" | grep -q "success: true"; then
        local data=$(echo "$result" | grep -oP 'data:\s*\K[0-9]+')
        if [ -n "$data" ]; then
            echo "$data"
            return 0
        fi
    fi
    return 1
}

# Check each motor
for joint in hook_joint trolley_joint slewing_joint; do
    echo "--- $joint ---"
    
    # Statusword (0x6041)
    SW=$(read_sdo "$joint" 6041 "Statusword")
    if [ -n "$SW" ]; then
        SW_HEX=$(printf "0x%04X" $SW)
        echo "  Statusword: $SW_HEX ($SW)"
        
        # Decode statusword
        if [ $((SW & 0x0027)) -eq 39 ]; then
            echo -e "    ${GREEN}✓ Operation Enabled${NC}"
        elif [ $((SW & 0x0008)) -ne 0 ]; then
            echo -e "    ${RED}✗ Fault State${NC}"
        elif [ $((SW & 0x0040)) -ne 0 ]; then
            echo -e "    ${YELLOW}⚠ Operation Disabled${NC}"
        else
            echo -e "    ${YELLOW}⚠ State: $SW_HEX${NC}"
        fi
    else
        echo -e "  ${RED}✗ Cannot read statusword${NC}"
    fi
    
    # Operation Mode (0x6061)
    MODE=$(read_sdo "$joint" 6061 "Operation Mode")
    if [ -n "$MODE" ]; then
        case $MODE in
            1) echo "  Operation Mode: 1 (Profile Position)" ;;
            3) echo "  Operation Mode: 3 (Profile Velocity)" ;;
            7) echo "  Operation Mode: 7 (Interpolated Position)" ;;
            *) echo "  Operation Mode: $MODE" ;;
        esac
    fi
    
    # Position (0x6064)
    POS=$(read_sdo "$joint" 6064 "Position")
    if [ -n "$POS" ]; then
        # Handle signed 32-bit
        if [ $POS -gt 2147483647 ]; then
            POS=$((POS - 4294967296))
        fi
        echo "  Position: $POS encoder counts"
    fi
    
    # Error Register (0x1001)
    ERR=$(read_sdo "$joint" 1001 "Error Register")
    if [ -n "$ERR" ]; then
        if [ "$ERR" -eq 0 ]; then
            echo -e "  Error Register: ${GREEN}0 (No errors)${NC}"
        else
            echo -e "  Error Register: ${RED}$ERR${NC}"
        fi
    fi
    
    echo ""
done

echo "=========================================="
echo "Statusword Reference:"
echo "  0x0027 = Operation Enabled (READY)"
echo "  0x0250 = Switch On Disabled"
echo "  0x0237 = Quick Stop Active"
echo "  0x0008 = Fault"
echo "=========================================="
