#!/bin/bash
# Read Position Actual Value (0x6064) from Node 3 (Slewing)
IFACE="can0"
NODE_ID="3"

echo "Reading Position from Node $NODE_ID..."
# SDO Upload 0x6064 sub 0
# Request: 40 64 60 00 00 00 00 00
CMD_ID=$(printf "%03X" $((0x600 + NODE_ID)))
RESP_ID=$(printf "%03X" $((0x580 + NODE_ID)))

cansend $IFACE ${CMD_ID}#4064600000000000
timeout 0.2s candump -L $IFACE,$RESP_ID:7FF -n 1


