#!/bin/bash
# Manually enable motors via direct CAN commands
# This replicates what pre_enable_drives does in the launch file

CAN_IF="can0"

echo "=========================================="
echo "MANUAL MOTOR ENABLE VIA CAN"
echo "=========================================="
echo ""
echo "This sends direct CAN commands to enable motors"
echo "Equivalent to launching with pre_enable_drives:=true"
echo ""

# Function to read statusword
read_sw() {
    local id=$1
    # Send SDO read request for 0x6041 (statusword)
    cansend ${CAN_IF} $(printf '%03X#4041600000000000' $((0x600+id))) 2>/dev/null
    timeout 0.2s candump -L ${CAN_IF},$(printf '%03X' $((0x580+id))):7FF 2>/dev/null | tail -n 1 || echo "no_reply"
}

# Enable each motor
for id in 1 2 3; do
    case $id in
        1) joint="hook_joint" ;;
        2) joint="trolley_joint" ;;
        3) joint="slewing_joint" ;;
    esac
    
    echo "=========================================="
    echo "Enabling Node ID ${id} (${joint})"
    echo "=========================================="
    
    # Step 1: Set Profile Position Mode (0x6060 = 1)
    echo "  Setting Profile Position Mode (0x6060 = 1)..."
    cansend ${CAN_IF} $(printf '%03X#2F60600001000000' $((0x600+id))) 2>/dev/null
    sleep 0.1
    
    # Step 2: Send enable sequence (controlword 0x6040: 6 → 7 → 15)
    echo "  Sending enable sequence..."
    
    for attempt in $(seq 1 10); do
        echo "    Attempt ${attempt}/10..."
        
        # Shutdown (6)
        cansend ${CAN_IF} $(printf '%03X#2B40600006000000' $((0x600+id))) 2>/dev/null
        sleep 0.05
        
        # Switch on (7) 
        cansend ${CAN_IF} $(printf '%03X#2B40600007000000' $((0x600+id))) 2>/dev/null
        sleep 0.05
        
        # Enable operation (15)
        cansend ${CAN_IF} $(printf '%03X#2B4060000F000000' $((0x600+id))) 2>/dev/null
        sleep 0.10
        
        # Read statusword
        sw=$(read_sw ${id})
        echo "    Statusword: ${sw}"
        
        # Check if operation enabled (statusword contains 0x27 or 0x1027)
        if echo "${sw}" | grep -qE '27|1027'; then
            echo "  ✓✓✓ Node ${id} (${joint}) is OPERATION ENABLED!"
            break
        fi
        
        if [ $attempt -eq 10 ]; then
            echo "  ✗✗✗ Failed to enable node ${id} after 10 attempts"
        fi
    done
    
    echo ""
done

echo "=========================================="
echo "ENABLE SEQUENCE COMPLETE"
echo "=========================================="
echo ""
echo "Now test movement with:"
echo "  ./simple_trolley_test.py"
echo ""


