#!/bin/bash
# Read Statusword (0x6041) from Node 3
IFACE="can0"
NODE_ID="3"

echo "Reading Statusword from Node $NODE_ID..."
CMD_ID=$(printf "%03X" $((0x600 + NODE_ID)))
RESP_ID=$(printf "%03X" $((0x580 + NODE_ID)))

cansend $IFACE ${CMD_ID}#4041600000000000
timeout 0.2s candump -L $IFACE,$RESP_ID:7FF -n 1


