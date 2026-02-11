#!/bin/bash
# Script to check CANopen motor status for node 3 (slewing_joint)

echo "=== Checking slewing_joint (Node ID 3) motor status ==="
echo ""
echo "Reading Statusword (0x6041)..."
cansend can0 "603#40604100"
echo "Waiting for response..."
sleep 0.5
candump can0 -n 1 -T 1000 | grep "583"
echo ""

echo "Reading Error Register (0x1001)..."
cansend can0 "603#40011000"
echo "Waiting for response..."
sleep 0.5
candump can0 -n 1 -T 1000 | grep "583"
echo ""

echo "Reading Error Code (0x603F)..."
cansend can0 "603#403F6000"
echo "Waiting for response..."
sleep 0.5
candump can0 -n 1 -T 1000 | grep "583"
echo ""

echo "=== Sending Fault Reset (if needed) ==="
echo "Sending controlword 0x0080 (Fault Reset)..."
cansend can0 "583#2B40604080000000"
sleep 0.5
echo ""

echo "=== Check if motor is ready after reset ==="
echo "Reading Statusword again..."
cansend can0 "603#40604100"
sleep 0.5
candump can0 -n 1 -T 1000 | grep "583"

