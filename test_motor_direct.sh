#!/bin/bash
# Direct CAN test to move slewing motor (node 3)

echo "=== Direct Motor Movement Test ==="
echo "Testing slewing_joint (Node ID 3)"
echo ""

CAN_IF="can0"
NODE_ID=3

echo "Step 1: Reading current statusword..."
cansend ${CAN_IF} $(printf '%03X#4041600000000000' $((0x600+NODE_ID)))
sleep 0.1
candump ${CAN_IF},$(printf '%03X' $((0x580+NODE_ID))):7FF -n 1

echo ""
echo "Step 2: Reading current position..."
cansend ${CAN_IF} $(printf '%03X#4064600000000000' $((0x600+NODE_ID)))
sleep 0.1
candump ${CAN_IF},$(printf '%03X' $((0x580+NODE_ID))):7FF -n 1

echo ""
echo "Step 3: Setting operation mode to Profile Position (mode=1)..."
cansend ${CAN_IF} $(printf '%03X#2F60600001000000' $((0x600+NODE_ID)))
sleep 0.1

echo ""
echo "Step 4: Sending target position = 1000 (0x3E8)..."
# Write to 0x607A (target position): value = 1000 = 0x000003E8
cansend ${CAN_IF} $(printf '%03X#237A6000E8030000' $((0x600+NODE_ID)))
sleep 0.1

echo ""
echo "Step 5: Triggering movement with controlword = 0x1F (bit 4 set)..."
# Controlword: 0x001F = Operation Enabled + New Set Point
cansend ${CAN_IF} $(printf '%03X#2B4060001F000000' $((0x600+NODE_ID)))
sleep 0.5

echo ""
echo "Step 6: Reading new position..."
cansend ${CAN_IF} $(printf '%03X#4064600000000000' $((0x600+NODE_ID)))
sleep 0.1
candump ${CAN_IF},$(printf '%03X' $((0x580+NODE_ID))):7FF -n 1

echo ""
echo "=== Test Complete ==="
echo "If position changed from 0 to ~1000, motor is working!"


