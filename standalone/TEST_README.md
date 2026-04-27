# MeshLib Standalone Box Offset Test

Simple command-line test to verify MeshLib box creation and offset functionality.

## What it tests

1. **Box Creation**: Creates a 2x2x2 cube using `mrmeshpy.makeCube()`
2. **Mesh Analysis**: Reads mesh properties (vertices, faces, volume, surface area)
3. **Voxel Size Calculation**: Uses `suggestVoxelSize()` to auto-calculate optimal voxel size
4. **Positive Offset**: Expands the cube by +0.2 units and verifies volume increases
5. **Negative Offset**: Shrinks the cube by -0.1 units and verifies volume decreases

## Usage

```bash
cd standalone
python test_box_offset.py
```

## Expected Output

```
============================================================
MeshLib Box + Offset Test
============================================================

[1/5] Creating cube...
 Cube created successfully

[2/5] Analyzing initial mesh...
  Vertices: 8
  Faces: 12
  Volume: 8.000000
  Surface Area: 24.000000

[3/5] Calculating voxel size...
  Suggested voxel size: 0.020000

[4/5] Applying positive offset (+0.2)...
  Vertices: 84632
  Faces: 169260
  Volume: 13.584182
  Surface Area: 32.028555

[5/5] Verifying results...
  Volume ratio: 1.6980
   Volume increased as expected (expanded by offset)

[BONUS] Testing negative offset (-0.1)...
  Volume ratio: 0.7290
   Volume decreased as expected (shrunk by offset)

============================================================
TEST RESULT:  PASSED
============================================================
```

## Test Criteria

-  Cube creates successfully without errors
-  Initial cube has correct volume (2 = 8.0)
-  Positive offset increases volume
-  Negative offset decreases volume
-  No degenerate faces in output mesh

## Requirements

- Python 3.7+
- MeshLib standalone build in `standalone/build/bin/`
- Modules: `mrmeshpy.pyd`, `mrvoxelspy.pyd`

## Troubleshooting

If you get import errors:
```
ERROR: Failed to import MeshLib modules
```

Make sure you've built the standalone modules:
```bash
cd standalone/build
cmake ..
cmake --build . --config Release
```

The test expects to find:
- `standalone/build/bin/mrmeshpy.pyd`
- `standalone/build/bin/mrvoxelspy.pyd`
