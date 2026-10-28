#!/usr/bin/env python3
"""Replace the s_s==1 block in pilot.cpp with fina2's version."""
import sys

# Read rjsd pilot.cpp
with open(sys.argv[1], 'r') as f:
    rjsd_lines = f.readlines()

# Read fina2 PILOT.CPP  
with open(sys.argv[2], 'r') as f:
    fina2_lines = f.readlines()

# rjsd: lines 109-244 (1-based) → indices 108-243
# fina2: lines 231-567 (1-based) → indices 230-566

# Verify anchors
assert 's_s == 1' in rjsd_lines[108], f"rjsd line 109 mismatch: {rjsd_lines[108]}"
assert 's_s == 2' in rjsd_lines[244], f"rjsd line 245 mismatch: {rjsd_lines[244]}"
assert 's_s == 1' in fina2_lines[230], f"fina2 line 231 mismatch: {fina2_lines[230]}"
assert 's_s == 2' in fina2_lines[603], f"fina2 line 604 mismatch: {fina2_lines[603]}"

# Extract fina2's s_s==1 block (lines 231-567, indices 230-566)
fina2_block = fina2_lines[230:567]

# Build new file: lines 1-108 + fina2 block + lines 245+
new_lines = rjsd_lines[:108] + fina2_block + rjsd_lines[244:]

# Write
with open(sys.argv[1], 'w') as f:
    f.writelines(new_lines)

print(f"OK: replaced rjsd lines 109-244 ({244-109+1} lines) with fina2 lines 231-567 ({567-231+1} lines)")
print(f"Old file: {len(rjsd_lines)} lines -> New file: {len(new_lines)} lines")
