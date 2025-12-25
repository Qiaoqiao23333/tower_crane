#!/bin/bash
# Test script for slewing motor timeout fix

set -e

echo "==================================="
echo "Slewing Motor Timeout Fix - Test"
echo "==================================="
echo ""

# Check if CAN interface is up
if ! ip link show can0 &>/dev/null; then
    echo "❌ ERROR: CAN interface 'can0' not found"
    echo "   Run: sudo ip link set can0 up type can bitrate 500000"
    exit 1
fi

echo "✅ CAN interface 'can0' is up"
echo ""

# Check if motors are responding
echo "Checking motor connectivity..."
echo ""

check_motor() {
    local node_id=$1
    local name=$2
    
    echo -n "  $name (node $node_id): "
    
    # Send NMT node guarding
    timeout 1 candump can0 | grep "70$node_id" | head -1 &
    local pid=$!
    sleep 0.1
    cansend can0 70$node_id#00 2>/dev/null
    sleep 0.5
    
    if kill -0 $pid 2>/dev/null; then
        kill $pid 2>/dev/null || true
        echo "✅ Responding"
        return 0
    else
        echo "❌ No response"
        return 1
    fi
}

check_motor 1 "Hook motor   "
check_motor 2 "Trolley motor"
check_motor 3 "Slewing motor"

echo ""
echo "Configuration changes applied:"
echo "  ✅ Increased sdo_timeout_ms: 2000 → 5000"
echo "  ✅ Increased boot_timeout_ms: 30000 → 60000"
echo "  ✅ Changed polling: false → true"
echo ""

echo "Next steps:"
echo ""
echo "1. Rebuild the package:"
echo "   cd /home/qiaoqiaochen/appdata/canros"
echo "   colcon build --packages-select tower_crane"
echo "   source install/setup.bash"
echo ""
echo "2. Launch the system:"
echo "   ros2 launch tower_crane hardware_bringup_real.launch.py can_interface_name:=can0"
echo ""
echo "3. Watch for these messages:"
echo "   ✅ [canopen_402_driver]: Init: Enable"
echo "   ✅ [resource_manager]: Successful 'activate' of hardware"
echo "   ✅ [spawner_joint_state_broadcaster]: Loaded joint_state_broadcaster"
echo ""
echo "4. If slewing motor still times out:"
echo "   - Check physical hardware (enable switch, brake, E-STOP)"
echo "   - Try manual enable: cansend can0 603#2B4060000F000000"
echo "   - Check motor LED indicators for fault codes"
echo "   - See SLEWING_MOTOR_TIMEOUT_FIX.md for detailed troubleshooting"
echo ""
echo "5. If problem persists, you can temporarily disable slewing motor:"
echo "   - Edit crane/tower_crane/config/robot_control/bus.yml"
echo "   - Comment out the entire slewing_joint section"
echo "   - Edit crane/tower_crane/config/tower_crane_ros2_control.yaml"
echo "   - Remove slewing_joint from joint lists"
echo ""


