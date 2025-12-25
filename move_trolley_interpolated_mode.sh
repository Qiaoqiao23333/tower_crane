#!/bin/bash
# Move trolley using Interpolated Position Mode (mode 7) - the correct mode!

echo "=========================================="
echo "Move Trolley - Interpolated Position Mode"
echo "=========================================="
echo ""

echo "ℹ️  DSY-C motor supports Interpolated Position Mode (7), NOT Profile Position Mode (1)"
echo ""

# Step 1: Initialize the motor
echo "Step 1: Initializing motor..."
ros2 service call /trolley_joint/init std_srvs/srv/Trigger
sleep 1
echo ""

# Step 2: Set to Interpolated Position Mode (mode 7)
echo "Step 2: Setting Interpolated Position Mode (mode 7)..."
ros2 service call /trolley_joint/interpolated_position_mode std_srvs/srv/Trigger
sleep 1
echo ""

# Step 3: Enable the motor
echo "Step 3: Enabling motor..."
ros2 service call /trolley_joint/enable std_srvs/srv/Trigger
sleep 2
echo ""

# Step 4: Check current position
echo "Step 4: Current position:"
ros2 topic echo /trolley_joint/joint_states --once | grep -A 1 "position:"
echo ""

# Step 5: Send target position
echo "Step 5: Sending target position..."
echo "  Using target: 32.768 (approximately 90 degrees of motor rotation)"
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 32.768}"
echo ""

# Step 6: Wait and check new position
echo "Step 6: Waiting 5 seconds for movement..."
sleep 5
echo ""

echo "Step 7: New position:"
ros2 topic echo /trolley_joint/joint_states --once | grep -A 1 "position:"
echo ""

echo "=========================================="
echo "Movement command complete!"
echo "Check if position changed above."
echo "=========================================="
echo ""
echo "Alternative target values to try:"
echo "  - Small test (3.6°):  ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble \"{target: 1.31}\""
echo "  - Medium test (18°): ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble \"{target: 6.55}\""
echo "  - 90 degrees:        ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble \"{target: 32.768}\""
echo "  - 180 degrees:       ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble \"{target: 65.536}\""
echo ""


