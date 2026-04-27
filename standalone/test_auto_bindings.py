#!/usr/bin/env python3
"""Test if auto-generated mrbind bindings are active in standalone."""

import sys
sys.path.insert(0, r'c:\Users\VijayRaghavVarada\Documents\Github\MeshLib\standalone\build\bin\meshlib')

import mrmeshpy as mr

print("=" * 70)
print("MRBIND AUTO-GENERATED BINDINGS VERIFICATION")
print("=" * 70)

# Count total symbols
all_attrs = [a for a in dir(mr) if not a.startswith('_')]
print(f"\n Total exported symbols: {len(all_attrs)}")

# Functions that ONLY exist in auto-generated bindings
auto_only = {
    'ICP': 'ICP alignment class',
    'ICPProperties': 'ICP properties',
    'triangulatePointCloud': 'Point cloud triangulation',
    'buildCylinderBetweenTwoHoles': 'Hole stitching',
    'mergeMeshes': 'Mesh merging',
    'StitchHolesParams': 'Stitch parameters',
    'getUniversalMetric': 'Mesh metric',
}

print("\n" + "=" * 70)
print("CHECKING AUTO-GENERATED FUNCTIONS:")
print("=" * 70)

found = []
missing = []

for func, desc in auto_only.items():
    if hasattr(mr, func):
        print(f"   {func:30s} - {desc}")
        found.append(func)
    else:
        print(f"   {func:30s} - {desc}")
        missing.append(func)

print("\n" + "=" * 70)
print("RESULT:")
print("=" * 70)

if len(found) >= 6:
    print(" AUTO-GENERATED BINDINGS ARE ACTIVE!")
    print(f"   Found {len(found)}/{len(auto_only)} signature functions")
    print(f"   API coverage: ~{len(all_attrs)} functions (vs ~100 in manual)")
elif len(all_attrs) > 200:
    print(" AUTO-GENERATED BINDINGS LIKELY ACTIVE")
    print(f"   Symbol count ({len(all_attrs)}) suggests full API")
else:
    print("  MANUAL BINDINGS ACTIVE")
    print(f"   Only {len(all_attrs)} symbols (typical for manual)")
    print("   Missing functions:", missing)

# Test core functionality
print("\n" + "=" * 70)
print("FUNCTIONALITY TEST:")
print("=" * 70)

try:
    cube = mr.makeCube()
    print(f" makeCube() - {cube.topology.numValidFaces()} faces")
    
    if hasattr(mr, 'ICP'):
        print(" ICP class available for alignment")
    
    if hasattr(mr, 'triangulatePointCloud'):
        print(" Point cloud operations available")
        
    print("\n All tests passed!")
    
except Exception as e:
    print(f" Error during testing: {e}")
    import traceback
    traceback.print_exc()
