#!/bin/bash
# Test script to control joints in mock mode

echo "=== Initializing and Enabling Joints ==="

# Initialize all joints
echo "Initializing hook_joint..."
ros2 service call /hook_joint/init std_srvs/srv/Trigger
sleep 1

echo "Initializing trolley_joint..."
ros2 service call /trolley_joint/init std_srvs/srv/Trigger
sleep 1

echo "Initializing slewing_joint..."
ros2 service call /slewing_joint/init std_srvs/srv/Trigger
sleep 1

# Enable all joints
echo "Enabling hook_joint..."
ros2 service call /hook_joint/enable std_srvs/srv/Trigger
sleep 1

echo "Enabling trolley_joint..."
ros2 service call /trolley_joint/enable std_srvs/srv/Trigger
sleep 1

echo "Enabling slewing_joint..."
ros2 service call /slewing_joint/enable std_srvs/srv/Trigger
sleep 2

echo ""
echo "=== Sending Position Commands ==="

# Move to initial positions
echo "Moving hook_joint to 0.0..."
ros2 service call /hook_joint/target canopen_interfaces/srv/COTargetDouble "{target: 0.0}"
sleep 2

echo "Moving trolley_joint to 0.0..."
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 0.0}"
sleep 2

echo "Moving slewing_joint to 0.0..."
ros2 service call /slewing_joint/target canopen_interfaces/srv/COTargetDouble "{target: 0.0}"
sleep 3

# Move to test positions
echo ""
echo "Moving to test positions..."
echo "hook_joint -> 90.0"
ros2 service call /hook_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}"
sleep 2

echo "trolley_joint -> 0.1"
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 0.1}"
sleep 2

echo "slewing_joint -> 0.5"
ros2 service call /slewing_joint/target canopen_interfaces/srv/COTargetDouble "{target: 0.5}"
sleep 3

echo ""
echo "=== Checking Joint States ==="
echo "hook_joint:"
ros2 topic echo /hook_joint/joint_states --once
echo ""
echo "trolley_joint:"
ros2 topic echo /trolley_joint/joint_states --once
echo ""
echo "slewing_joint:"
ros2 topic echo /slewing_joint/joint_states --once

echo ""
echo "=== Test Complete ==="

