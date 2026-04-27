# Missing API Functions in Standalone Build

## Issue

The standalone Python bindings (`mrmeshpy.pyd`, `mrvoxelspy.pyd`) are **manually created** and don't automatically include all C++ APIs that the main MeshLib build has. This is why functions like `findDegenerateFaces()` were missing.

## Root Cause

The standalone build in `standalone/source/MRPythonBindings/` uses **explicit pybind11 bindings** rather than the automatic code generation (mrbind) that the main MeshLib uses. This means:

- **Main MeshLib**: Uses `mrbind` tool to automatically generate Python bindings from C++ headers
- **Standalone**: Manually written pybind11 bindings in `.cpp` files

## Solution Applied

Added `findDegenerateFaces()` and other mesh fixing functions to:
- File: `standalone/source/MRPythonBindings/MRMeshOperationsBindings.cpp`

Added functions:
- `findDegenerateFaces(mesh, criticalAspectRatio)` - Find degenerate faces
- `findMultipleEdges(topology)` - Find multiple edges
- `fixMultipleEdges(mesh)` - Fix multiple edges
- `hasMultipleEdges(topology)` - Check for multiple edges

## How to Add More Missing APIs

### Step 1: Identify the Missing Function

Check where it's defined in the C++ code:
```bash
# Example: Search for the function
grep -r "functionName" source/MRMesh/*.h
```

### Step 2: Add the Python Binding

Edit the appropriate binding file in `standalone/source/MRPythonBindings/`:

- **MRMeshBindings.cpp** - Basic mesh data structures
- **MRMeshOperationsBindings.cpp** - Mesh operations (primitives, booleans, decimation, etc.)
- **MRVoxelsBindings.cpp** - Voxel operations (offset, etc.)
- **MRVectorBindings.cpp** - Vector/math types
- **MRBoxBindings.cpp** - Bounding box types

### Step 3: Include Required Headers

Add the header at the top of the binding file:
```cpp
#include "MRMesh/MRMeshFixer.h"
```

### Step 4: Add the Binding

In the appropriate `bind*()` function, add:
```cpp
m.def("functionName", [](args...) {
    auto result = functionName(args...);
    if (!result.has_value())  // For Expected<T> returns
        throw std::runtime_error(result.error());
    return result.value();
}, py::arg("argName1"), py::arg("argName2") = defaultValue,
   "Function description");
```

### Step 5: Rebuild

```bash
cd standalone/build
cmake --build . --config Release
```

## Complete API Parity Goal

To achieve complete API parity with the main MeshLib build, we would need to either:

1. **Option A (Current)**: Manually add bindings as needed
   - Pros: Full control, smaller binary
   - Cons: Maintenance overhead, missing functions

2. **Option B (Future)**: Integrate mrbind code generation
   - Pros: Automatic, complete API coverage
   - Cons: More complex build, larger binary

For now, we're using **Option A** and adding functions as they're discovered to be missing.

## Testing

After adding new bindings, test with:
```python
import mrmeshpy
print(dir(mrmeshpy))  # List all available functions
```

## Next Steps

1. Rebuild the standalone Python modules (see below)
2. Run `test_box_offset.py` to verify `findDegenerateFaces()` now works
3. Document any other missing APIs you encounter

## Rebuilding Instructions

```bash
# Navigate to standalone build directory
cd C:\Users\VijayRaghavVarada\Documents\Github\MeshLib\standalone\build

# Rebuild Python bindings
cmake --build . --config Release --target mrmeshpy

# Or rebuild everything
cmake --build . --config Release
```
