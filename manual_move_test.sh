#!/bin/bash
# Manual move test for Slewing Joint (Node 3)
# Uses standard CiA 402 Profile Position Mode

IFACE="can0"
NODE_ID="3" # Slewing
TARGET_POS="10000" # Some small movement (counts)

# Node ID to COB-ID
# RXPDO1 (Controlword) = 0x200 + NodeID
# RXPDO2 (Target Pos)  = 0x300 + NodeID (check mapping)
# SDO RX = 0x600 + NodeID
# SDO TX = 0x580 + NodeID

CMD_ID=$(printf "%03X" $((0x600 + NODE_ID)))
PDO1_ID=$(printf "%03X" $((0x200 + NODE_ID)))

echo "Resetting Faults..."
cansend $IFACE ${CMD_ID}#2B40600080000000 # Controlword = 0x80 (Fault Reset)
sleep 0.1

echo "Setting Mode 1 (Profile Position)..."
cansend $IFACE ${CMD_ID}#2F60600001000000 # Mode = 1
sleep 0.1

echo "Setting Profile Parameters..."
# Profile Velocity (0x6081) = 10000 (0x2710)
cansend $IFACE ${CMD_ID}#2381600010270000
# Profile Accel (0x6083) = 10000
cansend $IFACE ${CMD_ID}#2383600010270000
# Profile Decel (0x6084) = 10000
cansend $IFACE ${CMD_ID}#2384600010270000
sleep 0.1

echo "Enabling Drive..."
# Shutdown -> Switch On -> Enable Operation
cansend $IFACE ${CMD_ID}#2B40600006000000
sleep 0.1
cansend $IFACE ${CMD_ID}#2B40600007000000
sleep 0.1
cansend $IFACE ${CMD_ID}#2B4060000F000000
sleep 0.5

echo "Check Status..."
candump -L $IFACE,$(printf "%03X" $((0x580 + NODE_ID))):7FF --timeout 0.2

echo "Setting Target Position to $TARGET_POS..."
# 0x607A Target Position
POS_HEX=$(printf "%08X" $TARGET_POS)
# Little Endian for cansend: 12345678 -> 78 56 34 12
B1=${POS_HEX:6:2}
B2=${POS_HEX:4:2}
B3=${POS_HEX:2:2}
B4=${POS_HEX:0:2}

cansend $IFACE ${CMD_ID}#237A6000${B1}${B2}${B3}${B4}

echo "Executing Move (New Setpoint)..."
# Toggle Bit 4 (New Setpoint) in Controlword
# Enable (0x0F) -> 0x1F (New Setpoint)
cansend $IFACE ${CMD_ID}#2B4060001F000000
sleep 0.1
# Reset to 0x0F
cansend $IFACE ${CMD_ID}#2B4060000F000000

echo "Done. Monitor movement."

