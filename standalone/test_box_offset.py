"""
Simple test script for MeshLib box creation and offset operation.
Tests that the offset function works correctly on a cube primitive.
"""

import sys
import os

# Add the build directory to path for mrmeshpy and mrvoxelspy
BUILD_BIN_PATH = os.path.join(os.path.dirname(__file__), 'build', 'bin')
sys.path.insert(0, os.path.abspath(BUILD_BIN_PATH))

try:
    import mrmeshpy
    import mrvoxelspy
except ImportError as e:
    print(f"ERROR: Failed to import MeshLib modules: {e}")
    print(f"Make sure mrmeshpy.pyd and mrvoxelspy.pyd are in: {BUILD_BIN_PATH}")
    sys.exit(1)


def test_box_offset():
    """Test creating a box and applying offset operation"""
    print("=" * 60)
    print("MeshLib Box + Offset Test")
    print("=" * 60)
    
    # Step 1: Create a cube
    print("\n[1/5] Creating cube...")
    try:
        size = mrmeshpy.Vector3f(2.0, 2.0, 2.0)
        base = mrmeshpy.Vector3f(-1.0, -1.0, -1.0)
        cube = mrmeshpy.makeCube(size, base)
        print(f" Cube created successfully")
    except Exception as e:
        print(f" Failed to create cube: {e}")
        return False
    
    # Step 2: Get initial mesh statistics
    print("\n[2/5] Analyzing initial mesh...")
    try:
        initial_verts = cube.topology.numValidVerts()
        initial_faces = cube.topology.numValidFaces()
        initial_volume = cube.volume()
        initial_area = cube.area()
        
        print(f"  Vertices: {initial_verts}")
        print(f"  Faces: {initial_faces}")
        print(f"  Volume: {initial_volume:.6f}")
        print(f"  Surface Area: {initial_area:.6f}")
    except Exception as e:
        print(f" Failed to analyze mesh: {e}")
        return False
    
    # Step 3: Calculate suggested voxel size
    print("\n[3/5] Calculating voxel size...")
    try:
        voxel_size = mrvoxelspy.suggestVoxelSize(cube, 1000000)
        print(f"  Suggested voxel size: {voxel_size:.6f}")
    except Exception as e:
        print(f" Failed to calculate voxel size: {e}")
        return False
    
    # Step 4: Apply positive offset (expand)
    print("\n[4/5] Applying positive offset (+0.2)...")
    offset_distance = 0.2
    try:
        params = mrvoxelspy.OffsetParameters()
        params.voxelSize = voxel_size
        
        offset_cube = mrvoxelspy.offsetMesh(cube, offset_distance, params)
        
        offset_verts = offset_cube.topology.numValidVerts()
        offset_faces = offset_cube.topology.numValidFaces()
        offset_volume = offset_cube.volume()
        offset_area = offset_cube.area()
        
        print(f"  Vertices: {offset_verts}")
        print(f"  Faces: {offset_faces}")
        print(f"  Volume: {offset_volume:.6f}")
        print(f"  Surface Area: {offset_area:.6f}")
    except Exception as e:
        print(f" Failed to apply offset: {e}")
        return False
    
    # Step 5: Verify results
    print("\n[5/5] Verifying results...")
    success = True
    
    # Volume should increase with positive offset
    volume_ratio = offset_volume / initial_volume
    print(f"  Volume ratio: {volume_ratio:.4f}")
    if volume_ratio > 1.0:
        print(f"   Volume increased as expected (expanded by offset)")
    else:
        print(f"   Volume should increase with positive offset!")
        success = False
    
    # Check mesh validity (basic checks)
    try:
        # Check if mesh has reasonable geometry
        if offset_verts > 0 and offset_faces > 0:
            print(f"  ✓ Mesh has valid vertex and face counts")
        else:
            print(f"  ✗ Invalid mesh: verts={offset_verts}, faces={offset_faces}")
            success = False
            
        # Check volume is positive
        if offset_volume > 0:
            print(f"  ✓ Volume is positive (valid closed mesh)")
        else:
            print(f"  ⚠ Volume is not positive: {offset_volume}")
    except Exception as e:
        print(f"  ⚠ Could not validate mesh: {e}")
    
    # Test negative offset (shrink)
    print("\n[BONUS] Testing negative offset (-0.1)...")
    try:
        shrink_cube = mrvoxelspy.offsetMesh(cube, -0.1, params)
        shrink_volume = shrink_cube.volume()
        shrink_ratio = shrink_volume / initial_volume
        print(f"  Volume ratio: {shrink_ratio:.4f}")
        if shrink_ratio < 1.0:
            print(f"   Volume decreased as expected (shrunk by offset)")
        else:
            print(f"   Volume should decrease with negative offset!")
            success = False
    except Exception as e:
        print(f"   Negative offset test failed: {e}")
    
    return success


def main():
    """Main test runner"""
    print("\nMeshLib Standalone - Box Offset Test")
    print(f"Python version: {sys.version}")
    print(f"Build path: {BUILD_BIN_PATH}\n")
    
    # Run the test
    success = test_box_offset()
    
    # Print final result
    print("\n" + "=" * 60)
    if success:
        print("TEST RESULT:  PASSED")
        print("=" * 60)
        return 0
    else:
        print("TEST RESULT:  FAILED")
        print("=" * 60)
        return 1


if __name__ == "__main__":
    exit_code = main()
    sys.exit(exit_code)
