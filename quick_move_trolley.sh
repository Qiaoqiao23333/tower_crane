#!/bin/bash
# Quick script to properly move the trolley motor

echo "=========================================="
echo "Complete Trolley Motor Movement Sequence"
echo "=========================================="
echo ""

# Step 1: Initialize the motor
echo "Step 1: Initializing motor..."
ros2 service call /trolley_joint/init std_srvs/srv/Trigger
sleep 1
echo ""

# Step 2: Set to position mode (Profile Position Mode)
echo "Step 2: Setting position mode..."
ros2 service call /trolley_joint/position_mode std_srvs/srv/Trigger
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
echo "Step 5: Sending target position (90.0)..."
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"
echo ""

# Step 6: Wait and check new position
echo "Step 6: Waiting 5 seconds for movement..."
sleep 5
echo ""

echo "Step 7: New position:"
ros2 topic echo /trolley_joint/joint_states --once | grep -A 1 "position:"
echo ""

echo "=========================================="
echo "Done! Check if position changed above."
echo "=========================================="


