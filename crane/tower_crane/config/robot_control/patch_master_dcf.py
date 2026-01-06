#!/usr/bin/env python3
"""Patch master.dcf to disable vendor ID verification"""
import sys

def patch_master_dcf(filepath):
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    in_verification_section = None
    modified_lines = []
    
    for line in lines:
        # Patch 0x1F81 values (NMT assignment) to 0x05 = mandatory(0x01) + boot(0x04)
        if line.startswith('1=0x00000305') or line.startswith('2=0x00000305') or line.startswith('3=0x00000305'):
            modified_lines.append(line.replace('0x00000305', '0x00000005'))
            continue
        
        # Track when we're in verification sections (1F84=DeviceType, 1F85=VendorID, 1F86=ProductCode)
        if line.strip() in ['[1F84Value]', '[1F85Value]', '[1F86Value]']:
            in_verification_section = line.strip()
            modified_lines.append(line)
            continue
        
        # Exit verification section when we hit a new section
        if in_verification_section and line.startswith('['):
            in_verification_section = None
        
        # Patch all verification values to 0x00000000 (disable checks)
        if in_verification_section and (line.startswith('1=') or line.startswith('2=') or line.startswith('3=')):
            # Replace any hex value with 0x00000000
            parts = line.split('=')
            modified_lines.append(f"{parts[0]}=0x00000000\n")
            continue
        
        modified_lines.append(line)
    
    with open(filepath, 'w') as f:
        f.writelines(modified_lines)
    
    print(f"Patched {filepath}")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: patch_master_dcf.py <master.dcf>")
        sys.exit(1)
    patch_master_dcf(sys.argv[1])

