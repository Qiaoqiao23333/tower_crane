#!/usr/bin/env python3
"""Patch master.dcf to disable vendor ID verification and empty ManufacturerObjects"""
import sys
import re

def patch_master_dcf(filepath):
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    # First pass: identify and remove all manufacturer objects
    # Manufacturer objects are in range 0x2000-0x5FFF
    in_manufacturer_object = False
    filtered_lines = []
    
    for line in lines:
        line_stripped = line.strip()
        
        # Check if this is a manufacturer object section header
        if line.startswith('['):
            # Match [2xxx], [3xxx], [4xxx], [5xxx] (including sub and Value suffixes)
            if re.match(r'^\[(2[0-9A-Fa-f]{3}|[3-5][0-9A-Fa-f]{3})', line_stripped):
                in_manufacturer_object = True
                continue  # Skip this line
            else:
                # Not a manufacturer object, we've exited
                in_manufacturer_object = False
        
        # Skip lines that are part of manufacturer objects
        if in_manufacturer_object:
            continue
        
        # Keep this line
        filtered_lines.append(line)
    
    # Second pass: patch remaining content
    in_verification_section = None
    in_manufacturer_objects_section = False
    modified_lines = []
    
    for line in filtered_lines:
        # Patch 0x1F81 values (NMT assignment) to 0x05 = mandatory(0x01) + boot(0x04)
        if line.startswith('1=0x00000305') or line.startswith('2=0x00000305') or line.startswith('3=0x00000305'):
            modified_lines.append(line.replace('0x00000305', '0x00000005'))
            continue
        
        # Replace ManufacturerObjects section content
        if line.strip() == '[ManufacturerObjects]':
            modified_lines.append(line)
            in_manufacturer_objects_section = True
            continue
        
        # If we're in ManufacturerObjects section, handle it specially
        if in_manufacturer_objects_section:
            # Exit when we hit a new section
            if line.startswith('['):
                in_manufacturer_objects_section = False
                modified_lines.append(line)
                continue
            
            # Replace SupportedObjects value with 0
            if line.startswith('SupportedObjects='):
                modified_lines.append("SupportedObjects=0\n")
                continue
            
            # Skip all numbered entries (1=0x2000, 2=0x2001, etc.)
            if re.match(r'^\d+=0x[0-9A-Fa-f]+', line.strip()):
                continue
            
            # Keep any other lines (shouldn't be any, but just in case)
            modified_lines.append(line)
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
    
    print(f"Patched {filepath}: disabled verification and emptied ManufacturerObjects")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: patch_master_dcf.py <master.dcf>")
        sys.exit(1)
    patch_master_dcf(sys.argv[1])

