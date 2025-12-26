#!/bin/bash
# Complete sequence to enable hook_joint and move it

echo "========================================="
echo "Enabling and Moving Hook Joint"
echo "========================================="
echo ""

# Step 1: Initialize
echo "Step 1: Initializing hook_joint..."
ros2 service call /hook_joint/init std_srvs/srv/Trigger
sleep 2

# Step 2: Set position mode
echo "Step 2: Setting position mode..."
ros2 service call /hook_joint/position_mode std_srvs/srv/Trigger
sleep 2

# Step 3: Enable
echo "Step 3: Enabling hook_joint..."
ros2 service call /hook_joint/enable std_srvs/srv/Trigger
sleep 2

# Step 4: Check statusword
echo "Step 4: Checking motor status..."
STATUS=$(ros2 service call /hook_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6041, subindex: 0}" 2>/dev/null | grep -oP 'data:\s*\K[0-9]+')
if [ -n "$STATUS" ]; then
    STATUS_HEX=$(printf "0x%04X" $STATUS)
    echo "   Statusword: $STATUS_HEX"
    # Check if operation enabled (bit 2 set)
    if [ $((STATUS & 0x0004)) -ne 0 ]; then
        echo "   ✓ Motor is operation enabled"
    else
        echo "   ✗ Motor is NOT operation enabled"
    fi
fi

# Step 5: Check operation mode
echo "Step 5: Checking operation mode..."
MODE=$(ros2 service call /hook_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 6061, subindex: 0}" 2>/dev/null | grep -oP 'data:\s*\K[0-9]+')
if [ -n "$MODE" ]; then
    echo "   Operation Mode: $MODE"
    if [ "$MODE" = "1" ]; then
        echo "   ✓ Motor is in Profile Position Mode"
    else
        echo "   ✗ Motor is NOT in position mode (current: $MODE)"
    fi
fi

echo ""
echo "Step 6: Sending target command..."
RESULT=$(ros2 service call /hook_joint/target canopen_interfaces/srv/COTargetDouble "{target: 90.0}" 2>&1)
if echo "$RESULT" | grep -q "success=True"; then
    echo "   ✓ Target command sent successfully!"
else
    echo "   ✗ Target command failed"
    echo "   Response: $RESULT"
fi

echo ""
echo "Step 7: Monitoring joint state..."
echo "   (Press Ctrl+C to stop monitoring)"
timeout 5 ros2 topic echo /hook_joint/joint_states || true

echo ""
echo "========================================="
echo "Complete!"
echo "========================================="

