# MRBind Auto-Generation Setup for Standalone

## Status: ✅ **Python Bindings Working!**

### What's Working:
1. ✅ MSYS2 + Clang 18.1.8 installed
2. ✅ mrbind parser built (78MB executable at `thirdparty/mrbind/build/mrbind.exe`)
3. ✅ mrbind generates bindings with `--format=macros` (73,306 lines!)
4. ✅ MRMesh.dll built successfully
5. ✅ MRIOExtras.dll built successfully  
6. ✅ **mrmeshpy.pyd built and working!**
7. ✅ Python imports and tests passing!

### Manual Bindings (Currently Active)
The manual bindings in `source/MRPythonBindings/` provide a curated subset of the API:

**Types Available:**
- `Mesh`, `MeshTopology`, `Vector2f`, `Vector3f`, `Vector3i`
- `Box3f`, `Box3i`, `AffineXf3f`
- `FaceId`, `VertId`, `EdgeId`, `FaceBitSet`, `VertCoords`

**Operations Available:**
- Primitive creation: `makeCube()`, `makeTorus()`, `makeUVSphere()`
- Boolean: `boolean()` with Union, Intersection, DifferenceAB, DifferenceBA
- Decimation: `decimateMesh()` with configurable settings
- Convex hull: `makeConvexHull()`
- Collision: `findCollidingTriangles()`
- File I/O: `loadStl()`
- Repair: `findDegenerateFaces()`, `findMultipleEdges()`, `fixMultipleEdges()`
- Math: `cross()`, `dot()`

### Testing the Bindings
```python
# From standalone directory:
import sys
sys.path.insert(0, 'dist/meshlib')
import meshlib as mr

# Create a cube
cube = mr.makeCube()
print(f"Cube: {cube.topology.numValidFaces()} faces")

# Boolean union
cube2 = mr.makeCube(mr.Vector3f(1,1,1), mr.Vector3f(0.5,0.5,0.5))
union = mr.boolean(cube, cube2, mr.BooleanOperation.Union)
```

### MRBind Auto-Generation (For Full API Coverage)

The mrbind parser has successfully generated 73,306 lines of bindings in `temp_bindings/bindings_all.cpp`.
This can be enabled by:

1. Set `MESHLIB_USE_MRBIND_AUTO=ON` in CMake
2. The auto-generated bindings will provide complete API coverage

**Key Fixes Applied:**
- Fixed `offsetof` constexpr error by adding to combined header:
  ```cpp
  #include <stddef.h>
  #undef offsetof  
  #define offsetof(s,m) __builtin_offsetof(s,m)
  ```
- Changed mrbind format from `--format=json` to `--format=macros`
- Added `MB_FUNC` fallback macro in `MRPch/MRBindingMacros.h`

### Files Structure

```
standalone/
├── build/bin/
│   ├── MRMesh.dll            # Core mesh library
│   ├── MRIOExtras.dll        # I/O extras
│   ├── mrmeshpy.pyd          # Python module
│   └── pybind11nonlimitedapi_meshlib_standalone_3.13.dll
├── dist/meshlib/             # Distribution package
│   ├── __init__.py
│   ├── mrmeshpy.pyd
│   └── *.dll                 # All runtime dependencies
├── source/MRPythonBindings/  # Manual binding source
│   ├── mrmeshpy.cpp
│   ├── MRMeshBindings.cpp
│   ├── MRVectorBindings.cpp
│   ├── MRBoxBindings.cpp
│   └── MRMeshOperationsBindings.cpp
└── temp_bindings/            # Auto-generated bindings
    ├── all_headers.h         # Combined header
    └── bindings_all.cpp      # 73K lines of macros!
```

### Build Commands

```powershell
# Clean build
cd standalone/build
cmake -S .. -B . -G "Visual Studio 17 2022" -A x64 \
    -DCMAKE_TOOLCHAIN_FILE="C:/vcpkg/scripts/buildsystems/vcpkg.cmake" \
    -DMESHLIB_BUILD_PYTHON=ON \
    -DMESHLIB_USE_MRBIND_AUTO=OFF  # Use manual bindings
    
# Build Python module
cmake --build . --target mrmeshpy --config Release

# Build pybind11 shim for Python 3.13
cmake --build . --target pybind11nonlimitedapi_meshlib_standalone_3.13 --config Release
```


