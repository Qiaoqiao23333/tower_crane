#!/bin/bash
# Monitor CAN bus for Target Position (607A), Statusword (6041), and Controlword (6040)
# Usage: ./check_can_traffic.sh [interface]

IFACE=${1:-can0}

echo "Monitoring CAN traffic on $IFACE..."
echo "Looking for: "
echo "  - Controlword (0x6040) -> Index 60 40"
echo "  - Statusword  (0x6041) -> Index 41 60"
echo "  - Target Pos  (0x607A) -> Index 7A 60"
echo ""
echo "Press Ctrl+C to stop."

# We filter for the specific CAN IDs if known, or just grep the content.
# Node IDs are 1 (Hook), 2 (Trolley), 3 (Slewing).
# SDO/PDO COB-IDs:
# Node 1: 601/581 (SDO), 201/181 (PDO1), etc.
# Node 2: 602/582 (SDO), 202/182 (PDO1), etc.
# Node 3: 603/583 (SDO), 203/183 (PDO1), etc.
#
# Since mapping is dynamic/auto in bus.yml, we rely on the data content often being in standard places if mapped.
# But better to just dump and grep for the index if we suspect SDO, or look at PDOs if we know the mapping.
#
# bus.yml RPDO 1 mapping: 6040, 6060, 607A.
# bus.yml TPDO 1 mapping: 6041, 6061, 6064, 606C.
#
# Let's just watch everything for a few seconds and analyze.

candump -L $IFACE | grep -E " (20[1-3]|18[1-3]|60[1-3]|58[1-3])"


