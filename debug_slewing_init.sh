#!/bin/bash
# Debug script to monitor slewing motor initialization

echo "=== Slewing Motor Initialization Debug ==="
echo "This script will monitor CAN traffic during system startup"
echo ""

# Start candump in background
echo "Starting CAN traffic monitor..."
candump can0 -t A -L > /tmp/can_debug_$(date +%s).log 2>&1 &
CANDUMP_PID=$!

echo "CAN monitor started (PID: $CANDUMP_PID)"
echo "Log file: /tmp/can_debug_*.log"
echo ""
echo "Now launch your system in another terminal:"
echo "  ros2 launch tower_crane hardware_bringup_real.launch.py can_interface_name:=can0"
echo ""
echo "Press Ctrl+C when the error occurs to stop monitoring"
echo ""

# Wait for user to stop
trap "kill $CANDUMP_PID 2>/dev/null; echo ''; echo 'Analyzing logs...'" EXIT

# Monitor for specific events
echo "Monitoring for key events..."
tail -f /tmp/can_debug_*.log 2>/dev/null | while read line; do
    # Highlight SDO requests to node 3 (603)
    if echo "$line" | grep -q " 603 "; then
        echo "[SDO->Node3] $line"
    fi
    # Highlight SDO responses from node 3 (583)
    if echo "$line" | grep -q " 583 "; then
        echo "[SDO<-Node3] $line"
    fi
    # Highlight RPDO to node 3 (203)
    if echo "$line" | grep -q " 203 "; then
        echo "[RPDO->Node3] $line"
    fi
    # Highlight TPDO from node 3 (283)
    if echo "$line" | grep -q " 283 "; then
        # Only show occasionally to avoid spam
        if [ $((RANDOM % 50)) -eq 0 ]; then
            echo "[TPDO<-Node3] $line"
        fi
    fi
done


