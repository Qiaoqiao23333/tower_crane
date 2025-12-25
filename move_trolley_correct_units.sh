#!/bin/bash
# Move trolley with correct units understanding

echo "=========================================="
echo "Move Trolley with Correct Units"
echo "=========================================="
echo ""

echo "Motor Configuration:"
echo "  - Encoder: 131,072 counts/revolution"
echo "  - scale_pos_to_dev_: 1000.0"
echo "  - If you send {target: X}, motor receives: X * 1000"
echo ""

# Calculate correct target for 90 degrees
# 90 degrees = 90/360 = 0.25 revolutions
# 0.25 rev * 131072 counts/rev = 32768 counts
# To get 32768 counts, send: 32768 / 1000 = 32.768
TARGET_90_DEG=32.768

echo "To move 90 degrees of motor rotation:"
echo "  - Need: 32,768 encoder counts"
echo "  - Send: {target: $TARGET_90_DEG}"
echo ""

echo "Step 1: Initialize motor..."
ros2 service call /trolley_joint/init std_srvs/srv/Trigger
sleep 1
echo ""

echo "Step 2: Manually set operation mode via SDO (bypass position_mode service)..."
echo "  Writing 1 (Position Profile Mode) to 0x6060..."
ros2 service call /trolley_joint/sdo_write canopen_interfaces/srv/COWriteID "{index: 0x6060, subindex: 0, data: 1, type: 2}"
sleep 1
echo ""

echo "Step 3: Verify mode was set..."
ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x6061, subindex: 0}"
echo ""

echo "Step 4: Enable motor..."
ros2 service call /trolley_joint/enable std_srvs/srv/Trigger
sleep 2
echo ""

echo "Step 5: Current position:"
ros2 topic echo /trolley_joint/joint_states --once | grep -A 1 "position:"
echo ""

echo "Step 6: Sending target position for 90 degrees rotation..."
echo "  Command: {target: $TARGET_90_DEG}"
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: $TARGET_90_DEG}"
echo ""

echo "Step 7: Waiting 5 seconds for movement..."
sleep 5
echo ""

echo "Step 8: New position:"
ros2 topic echo /trolley_joint/joint_states --once | grep -A 1 "position:"
echo ""

echo "=========================================="
echo "Alternative: Try smaller test values"
echo "=========================================="
echo "If motor still doesn't move, try these test commands:"
echo ""
echo "  # Small movement (about 3.6 degrees):"
echo "  ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble \"{target: 1.31}\""
echo ""
echo "  # Medium movement (about 18 degrees):"
echo "  ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble \"{target: 6.55}\""
echo ""
echo "  # 90 degrees:"
echo "  ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble \"{target: 32.768}\""
echo ""


