#!/usr/bin/env python3
"""
Manual diagnostic and reset tool for slewing_joint (Node ID 3).
Run this BEFORE launching the hardware to clear any stuck states.
"""

import subprocess
import time
import sys

NODE_ID = 3
CAN_INTERFACE = "can0"

def send_sdo_read(index, subindex):
    """Send SDO read request"""
    cmd = f"cansend {CAN_INTERFACE} \"{600 + NODE_ID:03X}#40{index:04X}{subindex:02X}\""
    try:
        subprocess.run(cmd, shell=True, check=True, capture_output=True)
    except:
        pass

def send_sdo_write_u16(index, subindex, value):
    """Send SDO write request for 16-bit value"""
    cmd = f"cansend {CAN_INTERFACE} \"{580 + NODE_ID:03X}#2B{index:04X}{subindex:02X}{value:04X}0000\""
    try:
        subprocess.run(cmd, shell=True, check=True, capture_output=True)
        time.sleep(0.1)
    except Exception as e:
        print(f"Error sending SDO write: {e}")

def read_candump(timeout_ms=500):
    """Read candump output"""
    cmd = f"timeout {timeout_ms/1000:.2f}s candump {CAN_INTERFACE} 2>/dev/null | grep {580 + NODE_ID:03X}"
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.stdout.strip()
    except:
        return ""

print("=== Slewing Motor (Node 3) Diagnostic Tool ===\n")

print("Step 1: Reading Statusword (0x6041)...")
send_sdo_read(0x6041, 0)
time.sleep(0.5)
output = read_candump(200)
if output:
    print(f"  Response: {output}")
    # Parse statusword if possible
    if len(output) > 20:
        try:
            data_part = output.split("#")[1] if "#" in output else ""
            print(f"  Statusword indicates motor state")
        except:
            pass
else:
    print("  No response - motor may not be communicating!")
    sys.exit(1)

print("\nStep 2: Reading Error Register (0x1001)...")
send_sdo_read(0x1001, 0)
time.sleep(0.2)
output = read_candump(200)
if output:
    print(f"  Response: {output}")

print("\nStep 3: Reading Error Code (0x603F)...")
send_sdo_read(0x603F, 0)
time.sleep(0.2)
output = read_candump(200)
if output:
    print(f"  Response: {output}")

print("\nStep 4: Sending Fault Reset (0x6040 = 0x0080)...")
send_sdo_write_u16(0x6040, 0, 0x0080)
time.sleep(0.3)

print("\nStep 5: Sending Shutdown command (0x6040 = 0x0006)...")
send_sdo_write_u16(0x6040, 0, 0x0006)
time.sleep(0.3)

print("\nStep 6: Reading final Statusword...")
send_sdo_read(0x6041, 0)
time.sleep(0.2)
output = read_candump(200)
if output:
    print(f"  Response: {output}")

print("\n=== Diagnostic Complete ===")
print("If motor is still stuck, check:")
print("1. Physical enable switch/signal on motor drive")
print("2. Safety circuit connections")
print("3. Motor drive display/error codes")
print("4. Power supply to motor drive")

