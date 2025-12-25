#!/bin/bash
# Check if topics exist and are publishing data

echo "========================================="
echo "ROS2 Topics and Data Check"
echo "========================================="
echo ""

echo "1. Checking joint-related topics..."
echo ""
JOINT_TOPICS=$(ros2 topic list 2>/dev/null | grep -E "joint|canopen" | sort)
if [ -n "$JOINT_TOPICS" ]; then
    echo "$JOINT_TOPICS" | while read topic; do
        echo "   ✓ $topic"
    done
else
    echo "   ✗ No joint topics found"
fi
echo ""

echo "2. Testing topic data availability..."
echo ""

# Test hook_joint
echo "   Testing /hook_joint/joint_states..."
if timeout 3s ros2 topic echo /hook_joint/joint_states --once 2>/dev/null > /dev/null; then
    echo "      ✓ /hook_joint/joint_states is publishing"
    DATA=$(timeout 2s ros2 topic echo /hook_joint/joint_states --once 2>/dev/null | grep -A 1 "position:" | tail -1)
    echo "      Data: $DATA"
else
    echo "      ✗ /hook_joint/joint_states NOT publishing (timeout)"
fi
echo ""

# Test trolley_joint
echo "   Testing /trolley_joint/joint_states..."
if timeout 3s ros2 topic echo /trolley_joint/joint_states --once 2>/dev/null > /dev/null; then
    echo "      ✓ /trolley_joint/joint_states is publishing"
    DATA=$(timeout 2s ros2 topic echo /trolley_joint/joint_states --once 2>/dev/null | grep -A 1 "position:" | tail -1)
    echo "      Data: $DATA"
else
    echo "      ✗ /trolley_joint/joint_states NOT publishing (timeout)"
fi
echo ""

# Test slewing_joint
echo "   Testing /slewing_joint/joint_states..."
if timeout 3s ros2 topic echo /slewing_joint/joint_states --once 2>/dev/null > /dev/null; then
    echo "      ✓ /slewing_joint/joint_states is publishing"
    DATA=$(timeout 2s ros2 topic echo /slewing_joint/joint_states --once 2>/dev/null | grep -A 1 "position:" | tail -1)
    echo "      Data: $DATA"
else
    echo "      ✗ /slewing_joint/joint_states NOT publishing (timeout)"
fi
echo ""

# Test aggregated joint_states
echo "   Testing /joint_states (aggregated)..."
if timeout 3s ros2 topic echo /joint_states --once 2>/dev/null > /dev/null; then
    echo "      ✓ /joint_states is publishing"
    echo "      Joints in message:"
    timeout 2s ros2 topic echo /joint_states --once 2>/dev/null | grep "name:" | sed 's/^/        /'
else
    echo "      ✗ /joint_states NOT publishing (timeout)"
fi
echo ""

echo "3. Checking NMT states..."
echo ""
for JOINT in hook_joint trolley_joint slewing_joint; do
    echo "   Testing /${JOINT}/nmt_state..."
    if timeout 2s ros2 topic echo /${JOINT}/nmt_state --once 2>/dev/null > /dev/null; then
        STATE=$(timeout 2s ros2 topic echo /${JOINT}/nmt_state --once 2>/dev/null | grep "state:" | awk '{print $2}')
        echo "      ✓ NMT state: $STATE"
    else
        echo "      ✗ NMT state NOT available"
    fi
done
echo ""

echo "4. Checking controller status..."
echo ""
if ros2 control list_controllers 2>/dev/null > /dev/null; then
    echo "   Controllers:"
    ros2 control list_controllers 2>/dev/null | sed 's/^/      /'
else
    echo "   ✗ Cannot access controller manager"
fi
echo ""

echo "========================================="
echo "Summary"
echo "========================================="
echo ""
echo "If topics exist but show timeout:"
echo "  - Topics may be created but not publishing yet"
echo "  - Wait a few more seconds"
echo "  - Check for errors in the launch terminal"
echo ""
echo "If you want to test motor movement:"
echo "  - Make sure topics are publishing first"
echo "  - Then try: ros2 service call /trolley_joint/target ..."
echo ""



