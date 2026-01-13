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

---

## Implementation Progress

### ✅ Completed Phases

#### Phase 1-5: Core Infrastructure (COMPLETE)
| Component | File(s) | Status |
|-----------|---------|--------|
| Core ID Types | `Id.h` | ✅ EdgeId with sym(), undirected(), even(), odd() |
| BitSet | `BitSet.h` | ✅ 64-bit blocks, TypedBitSet<I>, SetBitIterator |
| IdVector | `IdVector.h` | ✅ Vector<T,I> type-safe containers |
| MeshTopology | `MeshTopology.h/.cpp` | ✅ Half-edge data structure |
| AABB Tree | `AABBTree.h/.cpp`, `AABBTreeNode.h` | ✅ Spatial indexing |

#### Phase 6: Core Foundation (COMPLETE)
| Component | File(s) | Status |
|-----------|---------|--------|
| Quaternion | `Quaternion.h` | ✅ Full quaternion with SLERP, rotation |
| AffineXf3 | `AffineXf.h` | ✅ 3D affine transforms, decomposition |
| Line3 | `Line.h` | ✅ Infinite lines and line segments (2D/3D) |
| Plane3 | `Plane.h` | ✅ Plane with intersection, projection |
| Sphere3 | `Sphere.h` | ✅ Sphere, Cylinder, Cone, Capsule |
| UnionFind | `UnionFind.h` | ✅ Disjoint set with path compression |
| Expected | `Expected.h` | ✅ Result<T,E> error handling |

#### Phase 7: Essential Queries (COMPLETE)
| Component | File(s) | Status |
|-----------|---------|--------|
| MeshTriPoint | `MeshTriPoint.h` | ✅ Barycentric coordinates, interpolation |
| MeshEdgePoint | `MeshEdgePoint.h` | ✅ Parametric edge positions, projection |
| MeshProject | `MeshProject.h` | ✅ AABB-accelerated point-to-mesh projection |
| MeshComponents | `MeshComponents.h` | ✅ Connected component analysis via UnionFind |
| MeshNormals | `MeshNormals.h` | ✅ Face/vertex normals, dihedral angles, sharp edges |
| MeshDistance | `MeshDistance.h` | ✅ Signed distance (pseudo-normal, winding, ray-cast) |

### 🔄 Current Phase

#### Phase 8: Mesh Repair (NEXT)
| Component | Description | Priority |
|-----------|-------------|----------|
| MeshFixer | Degenerate faces, duplicate edges | HIGH |
| CloseVertices | Merge close vertices | HIGH |
| FixSelfIntersections | Self-intersection repair | HIGH |
| MeshBoundary | Boundary detection and tracing | HIGH |
| MeshHoles | Hole detection and enumeration | HIGH |

### ⏳ Pending Phases

#### Phase 9: Point Cloud Complete
| Component | Description | Priority |
|-----------|-------------|----------|
| PointCloudTriangulation | Point-to-mesh conversion | HIGH |
| PointCloudMakeNormals | Normal estimation | HIGH |
| UniformSampling | Point subsampling | MEDIUM |

#### Phase 10: Advanced Algorithms
| Component | Description | Priority |
|-----------|-------------|----------|
| Laplacian | Smooth deformation | MEDIUM |
| SurfaceDistance | Geodesic distances | MEDIUM |
| ConvexHull | Convex hull computation | MEDIUM |
| Complete Boolean | Exact predicates | MEDIUM |

---

## Coverage Statistics

| Category | MRMesh Files | meshlib-core | Coverage |
|----------|--------------|--------------|----------|
| Core Infrastructure | ~40 | 17 | ~43% |
| Mesh Algorithms | ~60 | 6 | ~10% |
| Point Cloud | ~25 | 1 | ~4% |
| Boolean/CSG | ~15 | 1 | ~7% |
| Mesh Repair | ~20 | 1 | ~5% |
| Distance/Projection | ~15 | 5 | ~33% |
| I/O | ~25 | 3 | ~12% |
| **Total** | **~230** | **~30** | **~13%** |

---

## Industrial-Strength Roadmap: Phases 8-12

The following phases focus on the algorithms that make MeshLib "industrial-strength" - robust mesh processing that handles real-world edge cases, self-intersections, degenerate geometry, and complex repair scenarios.

---

## Phase 8: Mesh Repair (CRITICAL for Industrial Use)

**Priority: HIGHEST** - These are the most critical algorithms for processing real-world meshes that come from 3D scanning, CAD exports, or automated generation.

### 8.1 MeshFixer - Core Repair Infrastructure

**Source Files:**
- `MRMeshFixer.h/.cpp` - Main repair orchestration

**Key Functions:**
```cpp
// Vertex topology issues
int duplicateMultiHoleVertices(Mesh& mesh);
VertBitSet findRepeatedVertsOnHoleBd(const MeshTopology& topology);

// Edge issues  
Expected<std::vector<MultipleEdge>> findMultipleEdges(const MeshTopology& topology);
void fixMultipleEdges(Mesh& mesh, const std::vector<MultipleEdge>& multipleEdges);
void fixMultipleEdges(Mesh& mesh);  // finds + fixes

// Face quality issues
Expected<FaceBitSet> findDegenerateFaces(const MeshPart& mp, float criticalAspectRatio);
Expected<UndirectedEdgeBitSet> findShortEdges(const MeshPart& mp, float criticalLength);

// Comprehensive repair
Expected<void> fixMeshDegeneracies(Mesh& mesh, const FixMeshDegeneraciesParams& params);

// Topology simplification
bool isEdgeBetweenDoubleTris(const MeshTopology& topology, EdgeId e);
EdgeId eliminateDoubleTris(MeshTopology& topology, EdgeId e, FaceBitSet* region = nullptr);
void eliminateDoubleTrisAround(MeshTopology& topology, VertId v, FaceBitSet* region = nullptr);
bool isDegree3Dest(const MeshTopology& topology, EdgeId e);
EdgeId eliminateDegree3Dest(MeshTopology& topology, EdgeId e, FaceBitSet* region = nullptr);
int eliminateDegree3Vertices(MeshTopology& topology, VertBitSet& region, FaceBitSet* fs = nullptr);

// Crease/fold repair
void fixMeshCreases(Mesh& mesh, const FixCreasesParams& params);

// Orientation issues
Expected<FaceBitSet> findDisorientedFaces(const Mesh& mesh, const FindDisorientationParams& params);
```

**Parameters Structure:**
```cpp
struct FixMeshDegeneraciesParams {
    float maxDeviation = 0.0f;           // max surface deviation allowed
    float tinyEdgeLength = 0.0f;         // edges shorter than this are collapsed
    float criticalTriAspectRatio = 1e4f; // triangles with worse ratio are candidates
    float maxAngleChange = PI_F / 3;     // max dihedral angle change for flips
    float stabilizer = 1e-6f;            // numerical stability on planar regions
    FaceBitSet* region = nullptr;        // optional region to fix
    enum class Mode { Decimate, Remesh, RemeshPatch } mode = Mode::Remesh;
    ProgressCallback cb;
};
```

**Dependencies:** MeshDecimate, MeshSubdivide, MeshFillHole, RegionBoundary

### 8.2 FixSelfIntersections - Self-Intersection Repair

**Source Files:**
- `MRFixSelfIntersections.h/.cpp`

**Key Functions:**
```cpp
namespace SelfIntersections {
    Expected<FaceBitSet> getFaces(const Mesh& mesh, bool touchIsIntersection = true);
    Expected<void> fix(Mesh& mesh, const Settings& settings);
}
```

**Parameters:**
```cpp
struct SelfIntersections::Settings {
    bool touchIsIntersection = true;
    enum class Method { Relax, CutAndFill } method = Method::Relax;
    int relaxIterations = 5;
    int maxExpand = 3;               // edge steps from self-intersecting faces
    float subdivideEdgeLen = 0.0f;   // for hole covers (0 = auto, FLT_MAX = disable)
    ProgressCallback callback;
};
```

**Dependencies:** MeshCollide, MeshRelax, MeshFillHole

### 8.3 CloseVertices - Vertex Merging

**Source Files:**
- `MRCloseVertices.h/.cpp`

**Key Functions:**
```cpp
std::optional<VertMap> findSmallestCloseVertices(const Mesh& mesh, float closeDist);
std::optional<VertBitSet> findCloseVertices(const Mesh& mesh, float closeDist);
std::vector<EdgePair> findTwinEdgePairs(const Mesh& mesh, float closeDist);
EdgeBitSet findTwinEdges(const Mesh& mesh, float closeDist);
UndirectedEdgeHashMap findTwinUndirectedEdgeHashMap(const Mesh& mesh, float closeDist);
```

**Dependencies:** AABBTreePoints

### 8.4 RegionBoundary - Boundary Analysis

**Source Files:**
- `MRRegionBoundary.h/.cpp`
- `MRMeshBoundary.h/.cpp`

**Key Functions:**
```cpp
// Loop tracing
EdgeLoop trackLeftBoundaryLoop(const MeshTopology& topology, EdgeId e0, const FaceBitSet* region = nullptr);
EdgeLoop trackRightBoundaryLoop(const MeshTopology& topology, EdgeId e0, const FaceBitSet* region = nullptr);
std::vector<EdgeLoop> findLeftBoundary(const MeshTopology& topology, const FaceBitSet* region = nullptr);

// Region operations  
std::vector<EdgeLoop> delRegionKeepBd(Mesh& mesh, const FaceBitSet* region = nullptr);
std::vector<EdgePath> findLeftBoundaryInsideMesh(const MeshTopology& topology, const FaceBitSet& region);
UndirectedEdgeBitSet findRegionBoundaryUndirectedEdgesInsideMesh(const MeshTopology& topology, const FaceBitSet& region);

// Vertex/face sets from regions
VertBitSet getIncidentVerts(const MeshTopology& topology, const FaceBitSet& faces);
VertBitSet getInnerVerts(const MeshTopology& topology, const FaceBitSet* region = nullptr);
VertBitSet getBoundaryVerts(const MeshTopology& topology, const FaceBitSet* region = nullptr);
FaceBitSet getIncidentFaces(const MeshTopology& topology, const VertBitSet& verts);
FaceBitSet getInnerFaces(const MeshTopology& topology, const VertBitSet& verts);
FaceBitSet findRegionOuterFaces(const MeshTopology& topology, const FaceBitSet& region);

// Edge analysis
EdgeBitSet getRegionEdges(const MeshTopology& topology, const FaceBitSet& faces);
UndirectedEdgeBitSet getIncidentEdges(const MeshTopology& topology, const FaceBitSet& faces);
UndirectedEdgeBitSet getInnerEdges(const MeshTopology& topology, const VertBitSet& verts);
```

**Dependencies:** MeshTopology, BitSet

### 8.5 Hole Detection and Filling

**Source Files:**
- `MRMeshFillHole.h/.cpp` - Core hole filling
- `MRFillHoleNicely.h/.cpp` - High-quality hole filling
- `MRMeshMetrics.h/.cpp` - Triangulation quality metrics

**Key Functions:**
```cpp
// Basic hole filling
void fillHole(Mesh& mesh, EdgeId a, const FillHoleParams& params = {});
void fillHoles(Mesh& mesh, const std::vector<EdgeId>& as, const FillHoleParams& params = {});
void fillHoleTrivially(Mesh& mesh, EdgeId a, FaceBitSet* outNewFaces = nullptr);
bool isHoleBd(const MeshTopology& topology, const EdgeLoop& loop);

// Stitching two holes
void buildCylinderBetweenTwoHoles(Mesh& mesh, EdgeId a, EdgeId b, const StitchHolesParams& params = {});
bool buildCylinderBetweenTwoHoles(Mesh& mesh, const StitchHolesParams& params = {});

// High-quality filling with subdivision
FaceBitSet fillHoleNicely(Mesh& mesh, EdgeId holeEdge, const FillHoleNicelySettings& settings);

// Parallel planning
HoleFillPlan getHoleFillPlan(const Mesh& mesh, EdgeId e, const FillHoleParams& params = {});
std::vector<HoleFillPlan> getHoleFillPlans(const Mesh& mesh, const std::vector<EdgeId>& holeRepresentativeEdges);
void executeHoleFillPlan(Mesh& mesh, EdgeId a0, HoleFillPlan& plan, FaceBitSet* outNewFaces = nullptr);
```

**Parameters:**
```cpp
struct FillHoleParams {
    FillHoleMetric metric;                    // triangulation quality metric
    bool smoothBd = true;                     // minimize metric on boundary edges too
    FaceBitSet* outNewFaces = nullptr;
    enum class MultipleEdgesResolveMode { None, Simple, Strong } multipleEdgesResolveMode = Simple;
    bool makeDegenerateBand = false;
    int maxPolygonSubdivisions = 20;
    bool* stopBeforeBadTriangulation = nullptr;
};

struct FillHoleNicelySettings {
    FillHoleParams triangulateParams;
    bool triangulateOnly = false;
    UndirectedEdgeBitSet* notFlippable = nullptr;
    float maxEdgeLen = 0;
    int maxEdgeSplits = 1000;
    float maxAngleChangeAfterFlip = 30 * PI_F / 180.0f;
    bool smoothCurvature = true;
    bool naturalSmooth = false;
    EdgeWeights edgeWeights = EdgeWeights::Cotan;
    VertexMass vmass = VertexMass::Unit;
    VertUVCoords* uvCoords = nullptr;
    VertColors* colorMap = nullptr;
    FaceColors* faceColors = nullptr;
};
```

**Triangulation Metrics:**
```cpp
FillHoleMetric getCircumscribedMetric(const Mesh& mesh);       // minimize circumcircle radii
FillHoleMetric getPlaneFillMetric(const Mesh& mesh, EdgeId e); // prefer coplanar triangles
FillHoleMetric getPlaneNormalizedFillMetric(const Mesh& mesh, EdgeId e);
FillHoleMetric getComplexStitchMetric(const Mesh& mesh);       // aspect ratio + dihedral angle
FillHoleMetric getEdgeLengthFillMetric(const Mesh& mesh);      // minimize edge lengths
FillHoleMetric getVerticalStitchMetric(const Mesh& mesh, const Vector3f& upDir);
FillHoleMetric getComplexFillMetric(const Mesh& mesh, EdgeId e);
FillHoleMetric getMaxDihedralAngleMetric(const Mesh& mesh);
```

**Dependencies:** MeshDelone, MeshSubdivide, Laplacian, PositionVertsSmoothly

---

## Phase 9: Mesh Modification

**Priority: HIGH** - Essential algorithms for geometry processing pipelines.

### 9.1 MeshDecimate - Mesh Simplification (QEM)

**Source Files:**
- `MRMeshDecimate.h/.cpp`
- `MRMeshDecimateCallbacks.h/.cpp`
- `MRQuadraticForm.h/.cpp`

**Key Functions:**
```cpp
DecimateResult decimateMesh(Mesh& mesh, const DecimateSettings& settings = {});
```

**Parameters:**
```cpp
struct DecimateSettings {
    DecimateStrategy strategy = MinimizeError;  // or ShortestEdgeFirst
    float maxError = FLT_MAX;                   // max distance from original
    float maxEdgeLen = FLT_MAX;                 // max edge length after decimation
    float maxBdShift = FLT_MAX;                 // max boundary movement
    float maxTriangleAspectRatio = 20;          // max triangle aspect ratio
    float criticalTriAspectRatio = FLT_MAX;     // ignore angle check for worse
    float tinyEdgeLength = -1;                  // force collapse short edges
    float stabilizer = 0.001f;                  // for planar regions
    bool angleWeightedDistToPlane = false;      // weight by vertex angle
    bool optimizeVertexPos = true;              // optimize collapsed vertex position
    int maxDeletedVertices = INT_MAX;
    int maxDeletedFaces = INT_MAX;
    FaceBitSet* region = nullptr;               // restrict to region
    UndirectedEdgeBitSet* notFlippable = nullptr;
    UndirectedEdgeBitSet* edgesToCollapse = nullptr;
    UndirectedEdgeHashMap* twinMap = nullptr;   // for symmetric decimation
    bool touchNearBdEdges = true;
    bool touchBdVerts = true;
    VertBitSet* bdVerts = nullptr;
    float maxAngleChange = -1;                  // for edge flips (negative = no flips)
    PreCollapseCallback preCollapse;            // veto individual collapses
    std::function<void(UndirectedEdgeId, float&, Vector3f&)> adjustCollapse;
    Vector<QuadraticForm3f, VertId>* vertForms = nullptr;  // cache/init QEM
    bool packMesh = false;
    ProgressCallback progressCallback;
    int subdivideParts = 1;                     // parallel decimation
    bool decimateBetweenParts = true;
};

struct DecimateResult {
    int vertsDeleted = 0;
    int facesDeleted = 0;
    float errorIntroduced = 0;
    bool cancelled = true;
};
```

**QuadraticForm (QEM Core):**
```cpp
template <typename V>
struct QuadraticForm {
    using SM = typename V::SymMatrixType;
    SM A;
    typename V::ValueType c = 0;
    
    auto eval(const V& x) const;  // f(x) = x^T * A * x + c
    void addDistToOrigin(T weight);
    void addDistToPlane(const V& planeUnitNormal, T weight = 1);
    void addDistToLine(const V& lineUnitDir, T weight = 1);
};

std::pair<QuadraticForm<V>, V> sum(const QuadraticForm<V>& q0, const V& x0,
                                   const QuadraticForm<V>& q1, const V& x1,
                                   bool minAmong01 = false);
```

**Dependencies:** AABBTree, MeshDelone, QuadraticForm

### 9.2 MeshSubdivide - Mesh Subdivision

**Source Files:**
- `MRMeshSubdivide.h/.cpp`
- `MRMeshSubdivideCallbacks.h/.cpp`

**Key Functions:**
```cpp
int subdivideMesh(Mesh& mesh, const SubdivideSettings& settings = {});
Expected<Mesh> copySubdividePackMesh(const MeshPart& mp, float voxelSize, const ProgressCallback& cb = {});
```

**Parameters:**
```cpp
struct SubdivideSettings {
    float maxEdgeLen = 0;                     // target edge length
    int maxEdgeSplits = 1000;
    float maxDeviationAfterFlip = 1;          // surface deviation limit for flips
    float maxAngleChangeAfterFlip = FLT_MAX;
    float criticalAspectRatioFlip = 1000.0f;
    FaceBitSet* region = nullptr;
    FaceBitSet* maintainRegion = nullptr;
    UndirectedEdgeBitSet* notFlippable = nullptr;
    VertBitSet* newVerts = nullptr;
    bool subdivideBorder = true;
    float maxTriAspectRatio = 0;
    float maxSplittableTriAspectRatio = FLT_MAX;
    bool smoothMode = false;                  // smooth surface interpolation
    float minSharpDihedralAngle = PI_F / 6;
    bool projectOnOriginalMesh = false;
    std::function<bool(EdgeId)> beforeEdgeSplit;
    std::function<void(VertId)> onVertCreated;
    std::function<void(EdgeId, EdgeId)> onEdgeSplit;
    ProgressCallback progressCallback;
};
```

**Dependencies:** MeshDelone

### 9.3 MeshDelone - Delaunay Triangulation Improvement

**Source Files:**
- `MRMeshDelone.h/.cpp`

**Key Functions:**
```cpp
bool checkDeloneQuadrangle(const Vector3f& a, const Vector3f& b, const Vector3f& c, const Vector3f& d, float maxAngleChange = FLT_MAX);
bool checkDeloneQuadrangleInMesh(const Mesh& mesh, EdgeId edge, const DeloneSettings& settings = {});
bool bestQuadrangleDiagonal(const Vector3f& a, const Vector3f& b, const Vector3f& c, const Vector3f& d);
void makeDeloneOriginRing(Mesh& mesh, EdgeId e, const DeloneSettings& settings = {});
int makeDeloneEdgeFlips(Mesh& mesh, const DeloneSettings& settings = {}, int numIters = 1, const ProgressCallback& cb = {});
```

**Parameters:**
```cpp
struct DeloneSettings {
    float maxDeviationAfterFlip = FLT_MAX;
    float maxAngleChange = FLT_MAX;
    float criticalTriAspectRatio = FLT_MAX;
    const FaceBitSet* region = nullptr;
    const UndirectedEdgeBitSet* notFlippable = nullptr;
    const VertBitSet* vertRegion = nullptr;
};
```

### 9.4 MeshRelax - Mesh Smoothing

**Source Files:**
- `MRMeshRelax.h/.cpp`
- `MRRelaxParams.h`

**Key Functions:**
```cpp
bool relax(Mesh& mesh, const MeshRelaxParams& params = {}, const ProgressCallback& cb = {});
bool relaxKeepVolume(Mesh& mesh, const MeshRelaxParams& params = {}, const ProgressCallback& cb = {});
bool relaxApprox(Mesh& mesh, const MeshApproxRelaxParams& params = {}, const ProgressCallback& cb = {});
bool equalizeTriAreas(Mesh& mesh, const MeshEqualizeTriAreasParams& params = {}, const ProgressCallback& cb = {});
void removeSpikes(Mesh& mesh, int maxIterations, float minSumAngle, const VertBitSet* region = nullptr);
void smoothRegionBoundary(Mesh& mesh, const FaceBitSet& regionFaces, int numIters = 4);
void hardSmoothTetrahedrons(Mesh& mesh, const VertBitSet* region = nullptr);
```

**Parameters:**
```cpp
struct MeshRelaxParams {
    int iterations = 1;
    float force = 0.5f;                  // displacement multiplier
    const VertBitSet* region = nullptr;
    bool hardSmoothTetrahedrons = false;
    const VertScalars* weights = nullptr;
};

struct MeshApproxRelaxParams : MeshRelaxParams {
    float surfaceDilateRadius = 0.0f;    // neighborhood radius (0 = auto)
    RelaxApproxType type = RelaxApproxType::Planar;
};
```

### 9.5 PositionVertsSmoothly - Laplacian Smoothing

**Source Files:**
- `MRPositionVertsSmoothly.h/.cpp`
- `MRLaplacian.h/.cpp`

**Key Functions:**
```cpp
// Basic smooth positioning
void positionVertsSmoothly(Mesh& mesh, const VertBitSet& verts, EdgeWeights = EdgeWeights::Cotan, VertexMass = VertexMass::Unit);
void positionVertsSmoothlySharpBd(Mesh& mesh, const PositionVertsSmoothlyParams& params);

// Laplacian deformation (preserves details)
class Laplacian {
    void init(const VertBitSet& freeVerts, EdgeWeights weights, VertexMass vmass, RememberShape rem);
    void fixVertex(VertId v, bool smooth = true);
    void fixVertex(VertId v, const Vector3f& fixedPos, bool smooth = true);
    void updateSolver();
    void apply();
    void applyToScalar(VertScalars& scalarField);
};

// Inflation
void inflate(Mesh& mesh, const VertBitSet& verts, const InflateSettings& settings);
```

**Parameters:**
```cpp
struct PositionVertsSmoothlyParams {
    const VertBitSet* region = nullptr;
    const Vector<Vector3f, VertId>* vertShifts = nullptr;
    float stabilizer = 0;
    VertMetric vertStabilizers;
    UndirectedEdgeMetric edgeWeights;
};

struct InflateSettings {
    float pressure = 0;               // positive = outward, negative = inward
    int iterations = 3;
    bool preSmooth = true;
    bool gradualPressureGrowth = true;
};
```

**Dependencies:** Eigen (sparse solvers)

---

## Phase 10: Boolean Operations

**Priority: HIGH** - Essential for CAD-like operations.

### 10.1 MeshBoolean - CSG Operations

**Source Files:**
- `MRMeshBoolean.h/.cpp`
- `MRBooleanOperation.h/.cpp`
- `MRMeshBooleanFacade.h/.cpp`

**Key Functions:**
```cpp
BooleanResult boolean(const Mesh& meshA, const Mesh& meshB, BooleanOperation operation,
                      const BooleanParameters& params = {});
BooleanResult boolean(Mesh&& meshA, Mesh&& meshB, BooleanOperation operation,
                      const BooleanParameters& params = {});
Expected<Mesh> selfBoolean(const Mesh& mesh);
Contours3f findIntersectionContours(const Mesh& meshA, const Mesh& meshB, const AffineXf3f* rigidB2A = nullptr);
Expected<BooleanResultPoints> getBooleanPoints(const Mesh& meshA, const Mesh& meshB, BooleanOperation operation,
                                                const AffineXf3f* rigidB2A = nullptr);
```

**Types:**
```cpp
enum class BooleanOperation { 
    Inside, Outside, Union, Intersection, DifferenceAB, DifferenceBA 
};

struct BooleanResult {
    Mesh mesh;
    FaceBitSet meshABadContourFaces;
    FaceBitSet meshBBadContourFaces;
    std::string errorString;
    bool valid() const;
};

struct BooleanParameters {
    const AffineXf3f* rigidB2A = nullptr;
    BooleanResultMapper* mapper = nullptr;
    BooleanPreCutResult* outPreCutA = nullptr;
    BooleanPreCutResult* outPreCutB = nullptr;
    std::vector<EdgeLoop>* outCutEdges = nullptr;
    bool mergeAllNonIntersectingComponents = false;
    bool forceCut = false;
    ProgressCallback cb;
};
```

**Dependencies:** ContoursCut, MeshCollide, PrecisePredicates

### 10.2 ContoursCut - Contour-based Mesh Cutting

**Source Files:**
- `MRContoursCut.h/.cpp`
- `MROneMeshContours.h/.cpp`
- `MRContoursStitch.h/.cpp`

**Key Functions:**
```cpp
CutMeshResult cutMesh(Mesh& mesh, const OneMeshContours& contours, const CutMeshParameters& params = {});
Expected<FaceBitSet> cutMeshByContour(Mesh& mesh, const Contour3f& contour, const AffineXf3f& xf = {});
Expected<FaceBitSet> cutMeshByContours(Mesh& mesh, const Contours3f& contours, const AffineXf3f& xf = {});
```

### 10.3 MeshCollide - Collision Detection

**Source Files:**
- `MRMeshCollide.h/.cpp`
- `MRMeshCollidePrecise.h/.cpp`

**Key Functions:**
```cpp
std::vector<FaceFace> findCollidingTriangles(const MeshPart& a, const MeshPart& b, 
    const AffineXf3f* rigidB2A = nullptr, bool firstIntersectionOnly = false);
std::pair<FaceBitSet, FaceBitSet> findCollidingTriangleBitsets(const MeshPart& a, const MeshPart& b,
    const AffineXf3f* rigidB2A = nullptr);
Expected<bool> findSelfCollidingTriangles(const MeshPart& mp, std::vector<FaceFace>* outCollidingPairs, ...);
Expected<FaceBitSet> findSelfCollidingTrianglesBS(const MeshPart& mp, ProgressCallback cb = {});
bool isInside(const MeshPart& a, const MeshPart& b, const AffineXf3f* rigidB2A = nullptr);
bool isNonIntersectingInside(const MeshPart& a, const MeshPart& b, const AffineXf3f* rigidB2A = nullptr);
```

---

## Phase 11: Point Cloud Complete

**Priority: MEDIUM-HIGH** - Essential for 3D scanning workflows.

### 11.1 PointCloudTriangulation - Surface Reconstruction

**Source Files:**
- `MRPointCloudTriangulation.h/.cpp`
- `MRPointCloudTriangulationHelpers.h/.cpp`
- `MRLocalTriangulations.h/.cpp`

**Key Functions:**
```cpp
std::optional<Mesh> triangulatePointCloud(const PointCloud& pointCloud,
    const TriangulationParameters& params = {}, const ProgressCallback& progressCb = {});
bool fillHolesWithExtraPoints(Mesh& mesh, PointCloud& extraPoints,
    const FillHolesWithExtraPointsParams& params = {}, const ProgressCallback& progressCb = {});
```

**Parameters:**
```cpp
struct TriangulationParameters {
    int numNeighbours = 16;           // neighbors for local triangulation
    float radius = 0;                 // alternative to numNeighbours
    float critAngle = PI2_F;          // max angle between triangles in fan
    float boundaryAngle = 0.9f * PI_F;// vertex considered boundary if ring > this
    float critHoleLength = -FLT_MAX;  // auto-fill holes smaller than this
    bool automaticRadiusIncrease = true;
    const PointCloud* searchNeighbors = nullptr;
};
```

**Dependencies:** AABBTreePoints, LocalTriangulations

### 11.2 PointCloudMakeNormals - Normal Estimation

**Source Files:**
- `MRPointCloudMakeNormals.h/.cpp`

**Key Functions:**
```cpp
std::optional<VertNormals> makeUnorientedNormals(const PointCloud& pointCloud, float radius, ...);
std::optional<VertNormals> makeUnorientedNormals(const PointCloud& pointCloud, const AllLocalTriangulations& triangs, ...);
bool orientNormals(const PointCloud& pointCloud, VertNormals& normals, float radius, ...);
bool orientNormals(const PointCloud& pointCloud, VertNormals& normals, const AllLocalTriangulations& triangs, ...);
std::optional<VertNormals> makeOrientedNormals(const PointCloud& pointCloud, float radius, ...);
VertNormals makeNormals(const PointCloud& pointCloud, int avgNeighborhoodSize = 48);
```

### 11.3 PointCloudRelax - Point Cloud Smoothing

**Source Files:**
- `MRPointCloudRelax.h/.cpp`

### 11.4 Sampling Algorithms

**Source Files:**
- `MRUniformSampling.h/.cpp`
- `MRGridSampling.h/.cpp`
- `MRIterativeSampling.h/.cpp`
- `MRImproveSampling.h/.cpp`

---

## Phase 12: Advanced Algorithms

**Priority: MEDIUM** - Specialized algorithms for advanced use cases.

### 12.1 ICP - Iterative Closest Point Registration

**Source Files:**
- `MRICP.h/.cpp`
- `MRICPEnums.h`
- `MRMultiwayICP.h/.cpp`
- `MRPointToPointAligningTransform.h/.cpp`
- `MRPointToPlaneAligningTransform.h/.cpp`

**Key Functions:**
```cpp
class ICP {
    ICP(const MeshOrPointsXf& flt, const MeshOrPointsXf& ref, const VertBitSet& fltSamples = {}, const VertBitSet& refSamples = {});
    AffineXf3f calculateTransformation();
    float getMeanSqDistToPoint() const;
    float getMeanSqDistToPlane() const;
    size_t getNumActivePairs() const;
    void setParams(const ICPProperties& props);
    void updatePointPairs();
    void setXf(const AffineXf3f& fltXf);
};

class MultiwayICP {
    MultiwayICP(const ICPObjects& objects, const MultiwayICPSamplingParameters& samplingParams);
    Vector<AffineXf3f, ObjId> calculateTransformations(const ProgressCallback& cb = {});
    Vector<AffineXf3f, ObjId> calculateTransformationsFixFirst(const ProgressCallback& cb = {});
};
```

**Parameters:**
```cpp
struct ICPProperties {
    ICPMethod method = ICPMethod::PointToPlane;
    float p2plAngleLimit = PI_F / 6.0f;
    float p2plScaleLimit = 2;
    float cosThreshold = 0.7f;
    float distThresholdSq = 1.f;
    float farDistFactor = 3.f;
    ICPMode icpMode = ICPMode::AnyRigidXf;
    Vector3f fixedRotationAxis;
    int iterLimit = 10;
    int badIterStopCount = 3;
    float exitVal = 0;
    bool mutualClosest = false;
};
```

### 12.2 SurfaceDistance - Geodesic Distances

**Source Files:**
- `MRSurfaceDistance.h/.cpp`
- `MRSurfaceDistanceBuilder.h/.cpp`
- `MRSurfacePath.h/.cpp`
- `MRGeodesicPath.h/.cpp`

**Key Functions:**
```cpp
VertScalars computeSurfaceDistances(const Mesh& mesh, const VertBitSet& startVertices, float maxDist = FLT_MAX, ...);
VertScalars computeSurfaceDistances(const Mesh& mesh, const MeshTriPoint& start, const MeshTriPoint& end, ...);
```

### 12.3 ConvexHull

**Source Files:**
- `MRConvexHull.h/.cpp`

**Key Functions:**
```cpp
Mesh makeConvexHull(const VertCoords& points, const VertBitSet& validPoints);
Mesh makeConvexHull(const Mesh& in);
Mesh makeConvexHull(const PointCloud& in);
Contour2f makeConvexHull(Contour2f points);  // 2D
```

### 12.4 TunnelDetector - Topology Analysis

**Source Files:**
- `MRTunnelDetector.h/.cpp`

**Key Functions:**
```cpp
Expected<std::vector<EdgeLoop>> detectBasisTunnels(const MeshPart& mp, EdgeMetric metric = {}, ProgressCallback cb = {});
Expected<FaceBitSet> detectTunnelFaces(const MeshPart& mp, const DetectTunnelSettings& settings = {});
```

### 12.5 Edge Metrics

**Source Files:**
- `MREdgeMetric.h/.cpp`

**Key Functions:**
```cpp
EdgeMetric identityMetric();
EdgeMetric edgeLengthMetric(const Mesh& mesh);
EdgeMetric discreteAbsMeanCurvatureMetric(const Mesh& mesh);
EdgeMetric discreteMinusAbsMeanCurvatureMetric(const Mesh& mesh);
EdgeMetric edgeCurvMetric(const Mesh& mesh, float angleSinFactor = 2, float angleSinForBoundary = 0);
EdgeMetric edgeTableSymMetric(const MeshTopology& topology, const EdgeMetric& metric);
```

---

## Dependency Graph

```
Phase 8 (Repair)
├── MeshFixer ──────────────┬── MeshDecimate (Phase 9)
│   └── RegionBoundary      ├── MeshSubdivide (Phase 9)
│       └── MeshTopology    └── FillHole
│                               └── MeshMetrics
├── FixSelfIntersections ───┬── MeshCollide (Phase 10)
│                           └── MeshRelax (Phase 9)
├── CloseVertices ──────────── AABBTreePoints
└── FillHole ───────────────┬── MeshDelone (Phase 9)
                            └── PositionVertsSmoothly (Phase 9)
                                └── Laplacian
                                    └── Eigen (sparse)

Phase 9 (Modification)
├── MeshDecimate ───────────┬── QuadraticForm
│                           ├── AABBTree
│                           └── MeshDelone
├── MeshSubdivide ──────────── MeshDelone
├── MeshDelone ─────────────── (no dependencies)
├── MeshRelax ──────────────── (no dependencies)
└── PositionVertsSmoothly ──── Laplacian
                               └── Eigen

Phase 10 (Boolean)
├── MeshBoolean ────────────┬── ContoursCut
│                           ├── MeshCollide
│                           └── PrecisePredicates
├── ContoursCut ────────────── OneMeshContours
└── MeshCollide ────────────── AABBTree

Phase 11 (Point Cloud)
├── PointCloudTriangulation ── LocalTriangulations
│                              └── AABBTreePoints
└── PointCloudMakeNormals ──── AABBTreePoints

Phase 12 (Advanced)
├── ICP ────────────────────┬── PointToPointAligningTransform
│                           └── PointToPlaneAligningTransform
├── MultiwayICP ────────────── ICP
├── SurfaceDistance ────────── (Fast Marching)
├── ConvexHull ─────────────── (Quickhull)
└── TunnelDetector ─────────── EdgeMetric
```

---

## Implementation Priority Matrix

| Algorithm | Industrial Impact | Complexity | Dependencies | Priority Score |
|-----------|------------------|------------|--------------|----------------|
| MeshFixer | 10 | Medium | Low | **P0** |
| FillHole | 10 | Medium | Medium | **P0** |
| FixSelfIntersections | 10 | High | Medium | **P0** |
| MeshDecimate | 9 | High | Medium | **P1** |
| MeshSubdivide | 8 | Medium | Low | **P1** |
| MeshRelax | 8 | Low | Low | **P1** |
| MeshBoolean | 9 | Very High | High | **P1** |
| PointCloudTriangulation | 8 | High | Medium | **P2** |
| ICP | 8 | Medium | Medium | **P2** |
| SurfaceDistance | 6 | Medium | Low | **P3** |
| ConvexHull | 5 | Medium | None | **P3** |

---

## What Makes MeshLib "Industrial Strength"

1. **Robust Edge Case Handling**
   - Degenerate face detection and repair
   - Multiple edge resolution
   - Self-intersection detection and repair
   - Boundary vertex duplication for complex holes

2. **Numerical Stability**
   - Configurable stabilizers for planar regions
   - Quadratic error metrics with proper handling of singular cases
   - Precise predicates for boolean operations

3. **Progressive Algorithms**
   - Decimation with error bounds
   - Subdivision with quality constraints
   - Relaxation with volume preservation

4. **Quality Metrics**
   - Triangle aspect ratio monitoring
   - Dihedral angle constraints
   - Surface deviation tracking

5. **Regional Operations**
   - All major algorithms support FaceBitSet/VertBitSet regions
   - Boundary-aware processing
   - Edge constraint propagation

6. **Parallel Processing**
   - Multi-threaded decimation with subdivideParts
   - Parallel hole filling plans
   - TBB-based acceleration
