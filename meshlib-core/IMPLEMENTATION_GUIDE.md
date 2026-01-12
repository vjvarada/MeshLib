# Implementation Guide: Extracting MeshLib Core from MeshLib

This guide provides step-by-step instructions for extracting and adapting the core algorithms from the original MeshLib repository into the standalone `meshlib-core` library.

## Overview

The `meshlib-core` library is a standalone, minimal-dependency version of MeshLib's core 3D mesh processing algorithms. It's designed to be:

1. **Portable** - Compilable to native code, WebAssembly, and Python modules
2. **Minimal** - Only essential algorithms, no GUI or visualization dependencies
3. **Clean** - Simplified API with clear module boundaries

---

## Step 1: Copy Core Data Structures

### Source Files to Copy

From `source/MRMesh/`:

```
Core Math:
├── MRVector2.h → include/meshlib/core/Vector.h (combine 2D/3D/4D)
├── MRVector3.h
├── MRVector4.h
├── MRMatrix2.h → include/meshlib/core/Matrix.h
├── MRMatrix3.h
├── MRMatrix4.h
├── MRAffineXf3.h → include/meshlib/core/AffineXf.h
├── MRBox.h → include/meshlib/core/Box.h

Core Types:
├── MRId.h → include/meshlib/core/Id.h
├── MRId.cpp → src/core/Id.cpp
├── MRBitSet.h → include/meshlib/core/BitSet.h
├── MRBitSet.cpp → src/core/BitSet.cpp
├── MRVector.h → include/meshlib/core/Vector.h
├── MRExpected.h → include/meshlib/core/Expected.h
```

### Modifications Required

1. **Namespace Change**: Replace `namespace MR` with `namespace meshlib`
2. **Export Macro**: Replace `MRMESH_API` with `MESHLIB_API`
3. **Remove Dependencies**: Remove `MRPch/`, use standard includes

---

## Step 2: Copy Mesh Types

### Source Files to Copy

```
Mesh:
├── MRMesh.h → include/meshlib/mesh/Mesh.h
├── MRMesh.cpp → src/mesh/Mesh.cpp
├── MRMeshTopology.h → include/meshlib/mesh/MeshTopology.h
├── MRMeshTopology.cpp → src/mesh/MeshTopology.cpp
├── MRMeshBuilder.h → include/meshlib/mesh/MeshBuilder.h
├── MRMeshBuilder.cpp → src/mesh/MeshBuilder.cpp

Point Cloud:
├── MRPointCloud.h → include/meshlib/mesh/PointCloud.h
├── MRPointCloud.cpp → src/mesh/PointCloud.cpp
```

### Key Classes

- `Mesh` - Main mesh class with topology and vertex coordinates
- `MeshTopology` - Half-edge connectivity structure
- `PointCloud` - Point cloud with optional normals

---

## Step 3: Copy Spatial Indexing

### Source Files to Copy

```
AABB Trees:
├── MRAABBTree.h → include/meshlib/spatial/AABBTree.h
├── MRAABBTree.cpp → src/spatial/AABBTree.cpp
├── MRAABBTreeBase.h
├── MRAABBTreeMaker.h
├── MRAABBTreeNode.h
├── MRAABBTreePoints.h → include/meshlib/spatial/AABBTreePoints.h
├── MRAABBTreePoints.cpp → src/spatial/AABBTreePoints.cpp
```

---

## Step 4: Copy Core Algorithms

### Boolean Operations

```
From MRMesh/:
├── MRBooleanOperation.h
├── MRMeshBoolean.h → include/meshlib/algorithms/Boolean.h
├── MRMeshBoolean.cpp → src/algorithms/MeshBoolean.cpp
├── MRContoursCut.h
├── MRContoursCut.cpp
```

### Mesh Simplification

```
From MRMesh/:
├── MRMeshDecimate.h → include/meshlib/algorithms/Decimate.h
├── MRMeshDecimate.cpp → src/algorithms/MeshDecimate.cpp
├── MRQuadraticForm.h
├── MRQuadraticForm.cpp
```

### Mesh Subdivision

```
From MRMesh/:
├── MRMeshSubdivide.h → include/meshlib/algorithms/Subdivide.h
├── MRMeshSubdivide.cpp → src/algorithms/MeshSubdivide.cpp
├── MRMeshDelone.h
├── MRMeshDelone.cpp
```

### Hole Filling

```
From MRMesh/:
├── MRMeshFillHole.h → include/meshlib/algorithms/FillHole.h
├── MRMeshFillHole.cpp → src/algorithms/MeshFillHole.cpp
├── MRFillHoleNicely.h
├── MRFillHoleNicely.cpp
├── MRMeshMetrics.h
├── MRMeshMetrics.cpp
```

### ICP Alignment

```
From MRMesh/:
├── MRICP.h → include/meshlib/algorithms/ICP.h
├── MRICP.cpp → src/algorithms/ICP.cpp
├── MRICPEnums.h
├── MRMeshOrPoints.h
├── MRMeshOrPoints.cpp
├── MRPointToPointAligningTransform.h
├── MRPointToPointAligningTransform.cpp
├── MRPointToPlaneAligningTransform.h
├── MRPointToPlaneAligningTransform.cpp
```

### Collision Detection

```
From MRMesh/:
├── MRMeshCollide.h → include/meshlib/algorithms/Collision.h
├── MRMeshCollide.cpp → src/algorithms/MeshCollide.cpp
├── MRMeshCollidePrecise.h
├── MRMeshCollidePrecise.cpp
```

### Mesh Relaxation

```
From MRMesh/:
├── MRMeshRelax.h → include/meshlib/algorithms/Relax.h
├── MRMeshRelax.cpp → src/algorithms/MeshRelax.cpp
├── MRMeshRelax.hpp
├── MRRelaxParams.h
```

### Laplacian Deformation

```
From MRMesh/:
├── MRLaplacian.h → include/meshlib/algorithms/Laplacian.h
├── MRLaplacian.cpp → src/algorithms/Laplacian.cpp
```

### Self-Intersection Fixing

```
From MRMesh/:
├── MRFixSelfIntersections.h → include/meshlib/algorithms/FixSelfIntersections.h
├── MRFixSelfIntersections.cpp → src/algorithms/FixSelfIntersections.cpp
```

### Point Cloud Triangulation

```
From MRMesh/:
├── MRPointCloudTriangulation.h → include/meshlib/algorithms/PointCloudTriangulation.h
├── MRPointCloudTriangulation.cpp → src/algorithms/PointCloudTriangulation.cpp
├── MRPointCloudTriangulationHelpers.h
├── MRPointCloudTriangulationHelpers.cpp
├── MRLocalTriangulations.h
├── MRLocalTriangulations.cpp
```

---

## Step 5: Copy File I/O

### Source Files to Copy

```
From MRMesh/:
├── MRMeshLoad.h → include/meshlib/io/MeshLoad.h
├── MRMeshLoad.cpp → src/io/MeshLoad.cpp
├── MRMeshSave.h → include/meshlib/io/MeshSave.h
├── MRMeshSave.cpp → src/io/MeshSave.cpp
├── MRMeshLoadObj.h
├── MRMeshLoadObj.cpp → src/io/MeshLoadOBJ.cpp
├── MRMeshSaveObj.h
├── MRMeshSaveObj.cpp → src/io/MeshSaveOBJ.cpp
├── MRPly.h
├── MRPly.cpp → src/io/MeshLoadPLY.cpp, MeshSavePLY.cpp
├── MRIOParsing.h
├── MRIOParsing.cpp
├── MRIOFilters.h
├── MRIOFilters.cpp
```

---

## Step 6: Copy Primitive Creation Functions

### Source Files

```
From MRMesh/:
├── MRMakeSphereMesh.h
├── MRMakeSphereMesh.cpp
├── MRCube.h
├── MRCube.cpp
├── MRCylinder.h
├── MRCylinder.cpp
├── MRTorus.h
├── MRTorus.cpp
```

Create unified header: `include/meshlib/mesh/Primitives.h`

---

## Step 7: Copy Voxel Operations (Optional)

### Source Files (from MRVoxels/)

```
├── MROffset.h → include/meshlib/voxels/Offset.h
├── MROffset.cpp → src/voxels/Offset.cpp
├── MRMarchingCubes.h → include/meshlib/voxels/MarchingCubes.h
├── MRMarchingCubes.cpp → src/voxels/MarchingCubes.cpp
├── MRVDBConversions.h
├── MRVDBConversions.cpp
├── MRFloatGrid.h
├── MRFloatGrid.cpp
```

**Note:** Voxel operations require OpenVDB and should be optional.

---

## Step 8: Build Configuration

### Dependencies to Configure

| Dependency | Required | Purpose | Notes |
|------------|----------|---------|-------|
| Eigen | Yes | Math | Header-only, easy to bundle |
| parallel-hashmap | Yes | Hash maps | Header-only |
| tl-expected | Yes | Error handling | Header-only |
| TBB | Optional | Parallelism | Disable for WASM |
| OpenVDB | Optional | Voxels | Disable for WASM |
| pybind11 | Python only | Bindings | Header-only |

### Conditional Compilation

```cpp
// For parallel algorithms
#ifdef MESHLIB_HAS_TBB
    #include <tbb/parallel_for.h>
#endif

// For voxel operations
#ifdef MESHLIB_HAS_OPENVDB
    #include <openvdb/openvdb.h>
#endif
```

---

## Step 9: Testing

### Copy Relevant Tests

From `source/MRTest/`:
- Mesh construction tests
- Boolean operation tests
- Decimation tests
- I/O tests

Adapt to use Google Test or similar.

---

## Step 10: WebAssembly Specifics

### WASM Limitations

1. **No filesystem access** - Use memory buffers for I/O
2. **No threads by default** - Disable TBB, use single-threaded versions
3. **Memory constraints** - Be mindful of mesh sizes

### Required WASM Helpers

```cpp
// In bindings/wasm/bindings.cpp

// Load mesh from JavaScript ArrayBuffer
Mesh loadMeshFromBuffer(const uint8_t* data, size_t size, const std::string& format);

// Save mesh to JavaScript ArrayBuffer  
std::vector<uint8_t> saveMeshToBuffer(const Mesh& mesh, const std::string& format);

// Get mesh data for Three.js
val getMeshVertices(const Mesh& mesh);
val getMeshIndices(const Mesh& mesh);
```

---

## Estimated Timeline

| Phase | Tasks | Duration |
|-------|-------|----------|
| 1 | Core data structures | 2-3 days |
| 2 | Mesh types | 2-3 days |
| 3 | AABB trees | 1-2 days |
| 4 | Core algorithms | 5-7 days |
| 5 | File I/O | 2-3 days |
| 6 | WASM bindings | 3-4 days |
| 7 | Python bindings | 2-3 days |
| 8 | Testing & docs | 3-4 days |
| **Total** | | **20-29 days** |

---

## Files Created in This Plan

```
meshlib-core/
├── README.md                           ✅ Created
├── CMakeLists.txt                      ✅ Created
├── CMakePresets.json                   ✅ Created
├── include/meshlib/
│   ├── config.h.in                     ✅ Created
│   └── meshlib.h                       ✅ Created
├── src/
│   ├── core/BitSet.cpp                 ✅ Placeholder
│   └── mesh/Mesh.cpp                   ✅ Placeholder
├── bindings/
│   ├── wasm/
│   │   ├── CMakeLists.txt              ✅ Created
│   │   ├── bindings.cpp                ✅ Created
│   │   ├── meshlib.d.ts                ✅ Created
│   │   └── three_integration.ts        ✅ Created
│   ├── python/
│   │   ├── CMakeLists.txt              ✅ Created
│   │   ├── bindings.cpp                ✅ Created
│   │   └── __init__.py                 ✅ Created
│   └── c/
│       ├── CMakeLists.txt              ✅ Created
│       ├── meshlib_c.h                 ✅ Created
│       └── meshlib_c.cpp               ✅ Created
├── examples/
│   ├── cpp/
│   │   ├── CMakeLists.txt              ✅ Created
│   │   ├── boolean_example.cpp         ✅ Created
│   │   ├── decimate_example.cpp        ✅ Created
│   │   └── icp_example.cpp             ✅ Created
│   └── web/
│       └── index.html                  ✅ Created
└── cmake/
    └── meshlib_core-config.cmake.in    ✅ Created
```

---

## Next Steps

1. **Begin extraction** - Start copying source files according to this guide
2. **Resolve dependencies** - Set up Eigen, parallel-hashmap, tl-expected
3. **Test native build** - Ensure C++ library compiles and works
4. **Test WASM build** - Verify Emscripten compilation
5. **Test Python build** - Verify pybind11 bindings work
6. **Create Three.js demo** - Build working web example
