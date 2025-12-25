#!/bin/bash
# Diagnostic script to check why trolley motor isn't moving

echo "========================================="
echo "Trolley Motor Diagnostic"
echo "========================================="
echo ""

echo "1. Checking current joint state..."
ros2 topic echo /trolley_joint/joint_states --once
echo ""

echo "2. Checking NMT (Network Management) state..."
ros2 topic echo /trolley_joint/nmt_state --once
echo ""

echo "3. Checking if motor is enabled..."
echo "   (Look for 'enabled' or 'operational' state)"
echo ""

echo "4. Checking motor status via SDO read..."
echo "   Reading Statusword (0x6041) - should show motor state..."
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}" --once
echo ""

echo "5. Checking error register (0x1001)..."
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 1001, subindex: 0}" --once
echo ""

echo "6. Checking operation mode (0x6061)..."
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6061, subindex: 0}" --once
echo ""

echo "========================================="
echo "Recommended Actions:"
echo "========================================="
echo ""
echo "If motor is not enabled, try:"
echo "  ros2 service call /trolley_joint/enable std_srvs/srv/Trigger"
echo ""
echo "If motor is not initialized, try:"
echo "  ros2 service call /trolley_joint/init std_srvs/srv/Trigger"
echo ""
echo "If motor is not in position mode, try:"
echo "  ros2 service call /trolley_joint/position_mode std_srvs/srv/Trigger"
echo ""
echo "To check for errors:"
echo "  ros2 service call /trolley_joint/recover std_srvs/srv/Trigger"
echo ""



