#!/bin/bash
# Deep check of motor state using CANopen SDO reads

echo "=========================================="
echo "DEEP MOTOR DIAGNOSTIC"
echo "=========================================="
echo ""

echo "--- 1. Raw CANopen TPDO (Transmit PDO from motor) ---"
ros2 topic echo /trolley_joint/tpdo --once 2>/dev/null | head -30
echo ""

echo "--- 2. Motor Statusword (0x6041) ---"
echo "Expected values:"
echo "  0x0027 (39) = Operation enabled"
echo "  0x0021 (33) = Ready to switch on"
echo "  0x0023 (35) = Switched on"
result=$(ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x6041, subindex: 0}" 2>/dev/null)
echo "$result"
# Extract the data value
data=$(echo "$result" | grep -oP 'data: \K\d+')
if [ ! -z "$data" ]; then
    echo "  → Statusword value: $data (0x$(printf '%04X' $data))"
    echo ""
fi

echo "--- 3. Operation Mode Display (0x6061) ---"
echo "Expected: 1 = Profile Position Mode"
result=$(ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x6061, subindex: 0}" 2>/dev/null)
echo "$result"
data=$(echo "$result" | grep -oP 'data: \K\d+')
if [ ! -z "$data" ]; then
    echo "  → Operation mode: $data"
    echo ""
fi

echo "--- 4. Position Actual Value (0x6064) ---"
result=$(ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x6064, subindex: 0}" 2>/dev/null)
echo "$result"
data=$(echo "$result" | grep -oP 'data: \K\d+')
if [ ! -z "$data" ]; then
    echo "  → Current position: $data encoder counts"
    echo ""
fi

echo "--- 5. Target Position (0x607A) ---"
result=$(ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x607A, subindex: 0}" 2>/dev/null)
echo "$result"
data=$(echo "$result" | grep -oP 'data: \K\d+')
if [ ! -z "$data" ]; then
    echo "  → Target position: $data encoder counts"
    echo ""
fi

echo "--- 6. Error Register (0x1001) ---"
echo "Expected: 0 = No error"
result=$(ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x1001, subindex: 0}" 2>/dev/null)
echo "$result"
data=$(echo "$result" | grep -oP 'data: \K\d+')
if [ ! -z "$data" ]; then
    echo "  → Error register: $data"
    if [ "$data" -ne 0 ]; then
        echo "  ⚠ ERROR DETECTED!"
    fi
    echo ""
fi

echo "--- 7. Controlword (0x6040) ---"
result=$(ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x6040, subindex: 0}" 2>/dev/null)
echo "$result"
data=$(echo "$result" | grep -oP 'data: \K\d+')
if [ ! -z "$data" ]; then
    echo "  → Controlword: $data (0x$(printf '%04X' $data))"
    echo ""
fi

echo "=========================================="
echo "TESTING WITH LARGER TARGET VALUE"
echo "=========================================="
echo ""

echo "Sending target: 10000 (much larger)"
ros2 service call /trolley_joint/target canopen_interfaces/srv/COTargetDouble "{target: 10000.0}" 2>/dev/null
echo ""

echo "Waiting 3 seconds..."
sleep 3

echo "Checking position again..."
result=$(ros2 service call /trolley_joint/sdo_read canopen_interfaces/srv/COReadID "{index: 0x6064, subindex: 0}" 2>/dev/null)
echo "$result"
data=$(echo "$result" | grep -oP 'data: \K\d+')
if [ ! -z "$data" ]; then
    echo "  → Position after large target: $data encoder counts"
fi

echo ""
echo "=========================================="
echo "DONE"
echo "=========================================="


