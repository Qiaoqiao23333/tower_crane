#!/bin/bash
# Diagnose motor state to understand why it's not moving

echo "=========================================="
echo "Motor State Diagnostic Tool"
echo "=========================================="
echo ""

echo "1. Reading Statusword (0x6041)..."
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x6041, subindex: 0}"
echo ""

echo "2. Reading Modes of operation display (0x6061)..."
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x6061, subindex: 0}"
echo ""

echo "3. Reading Target position (0x607A)..."
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x607A, subindex: 0}"
echo ""

echo "4. Reading Position actual value (0x6064)..."
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x6064, subindex: 0}"
echo ""

echo "5. Reading Controlword (0x6040)..."
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x6040, subindex: 0}"
echo ""

echo "6. Reading Modes of operation (0x6060) - should be 1 for position mode..."
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x6060, subindex: 0}"
echo ""

echo "7. Checking if motor is already in position mode..."
ros2 service call /trolley_joint/sdo_write canopen_interfaces/srv/COWriteID "{index: 0x6060, subindex: 0, data: 1, type: 2}"
echo ""

echo "8. Current joint_states:"
ros2 topic echo /trolley_joint/joint_states --once
echo ""

echo "=========================================="
echo "Diagnostic complete!"
echo "=========================================="


