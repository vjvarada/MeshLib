# Python Bindings for MeshLib Standalone

## Overview

MeshLib Standalone provides Python bindings via pybind11, enabling mesh processing from Python scripts.

## Building

```bash
cd standalone/build
cmake .. -DMESHLIB_BUILD_PYTHON=ON
cmake --build . --target mrmeshpy --config Release
```

## Usage

```python
import mrmeshpy

# Create primitive meshes
cube = mrmeshpy.makeCube()
sphere = mrmeshpy.makeUVSphere(1.0, 32, 32)
torus = mrmeshpy.makeTorus(1.0, 0.3, 32, 16)
cylinder = mrmeshpy.makeCylinder(0.5, 2.0, 24)

# Query mesh properties
print(f"Vertices: {cube.topology.numValidVerts()}")
print(f"Faces: {cube.topology.numValidFaces()}")
print(f"Volume: {cube.volume()}")
print(f"Area: {cube.area()}")

# Boolean operations
union = mrmeshpy.boolean(meshA, meshB, mrmeshpy.BooleanOperation.Union)
intersection = mrmeshpy.boolean(meshA, meshB, mrmeshpy.BooleanOperation.Intersection)
diff = mrmeshpy.boolean(meshA, meshB, mrmeshpy.BooleanOperation.DifferenceAB)

# Mesh decimation
settings = mrmeshpy.DecimateSettings()
settings.maxDeletedFaces = 1000
result = mrmeshpy.decimateMesh(mesh, settings)

# Mesh subdivision
settings = mrmeshpy.SubdivideSettings()
settings.maxEdgeLen = 0.1
numSplits = mrmeshpy.subdivideMesh(mesh, settings)

# Convex hull
hull = mrmeshpy.makeConvexHull(mesh)

# Fill holes
numFilled = mrmeshpy.fillAllHoles(mesh)

# Save/Load
mrmeshpy.saveMesh(mesh, "output.stl")
mesh = mrmeshpy.loadMesh("input.obj")

# Transformations
xf = mrmeshpy.AffineXf3f.translation(mrmeshpy.Vector3f(1, 2, 3))
mesh.transform(xf)
```

## Available Types

- `Vector3f` - 3D vector
- `Box3f` - Axis-aligned bounding box
- `Mesh` - Triangle mesh
- `MeshTopology` - Mesh connectivity
- `FaceId`, `VertId`, `EdgeId` - Mesh element identifiers
- `FaceBitSet` - Bit set for faces
- `AffineXf3f` - Affine transformation
- `DecimateSettings`, `SubdivideSettings` - Algorithm parameters

## MSVC Limitation

The auto-generated mrbind bindings (which provide full API coverage) do not compile with MSVC due to template specialization issues in `rebind_container.h`. This is a known limitation documented in the mrbind source code.

The main MeshLib project uses **MSYS2 Clang** on Windows to build its Python bindings.

For the standalone version on MSVC, we provide comprehensive manual bindings that cover the most important functionality:
- Primitive mesh creation (cube, sphere, torus, cylinder)
- Mesh loading/saving (STL, OBJ, PLY, OFF, CTM)
- Boolean operations (union, intersection, difference)
- Mesh decimation and subdivision
- Convex hull computation
- Hole filling
- Mesh relaxation/smoothing
- Component analysis

To use auto-generated bindings, you would need to either:
1. Install MSYS2 with Clang and use the main MeshLib build system
2. Use a different compiler (GCC or Clang-based)

## File Locations

- Module: `standalone/build/bin/mrmeshpy.pyd` (Windows)
- Source: `standalone/source/MRPythonBindings/`
