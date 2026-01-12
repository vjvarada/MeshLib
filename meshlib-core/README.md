# MeshLib Core - Standalone 3D Mesh Processing Library

A standalone, cross-platform C++ library for 3D mesh processing that can be:
- **Used directly in C++** applications
- **Compiled to WebAssembly** for browser-based 3D apps (Three.js integration)
- **Wrapped for Python** via pybind11

## Project Overview

This is a clean, standalone extraction of MeshLib's core algorithms designed for maximum reusability and minimal dependencies.

---

## Implementation Roadmap

### Phase 1: Core Data Structures (Foundation)

**Goal:** Establish the fundamental data structures for 3D mesh representation.

| Component | Description | Source Reference |
|-----------|-------------|------------------|
| `Vector2/3/4` | Basic vector math types | `MRMesh/MRVector2.h`, `MRVector3.h`, `MRVector4.h` |
| `Matrix3/4` | Transformation matrices | `MRMesh/MRMatrix3.h`, `MRMatrix4.h` |
| `AffineXf3` | Affine transformations | `MRMesh/MRAffineXf3.h` |
| `Box3` | Bounding boxes | `MRMesh/MRBox.h` |
| `Id` types | Vertex, Face, Edge identifiers | `MRMesh/MRId.h` |
| `BitSet` | Efficient bit sets for selections | `MRMesh/MRBitSet.h` |
| `Vector<T, Id>` | Id-indexed containers | `MRMesh/MRVector.h` |

**Dependencies:** Eigen (header-only), parallel-hashmap (header-only)

---

### Phase 2: Mesh Topology & Geometry

**Goal:** Core mesh representation with half-edge data structure.

| Component | Description | Source Reference |
|-----------|-------------|------------------|
| `MeshTopology` | Half-edge connectivity | `MRMesh/MRMeshTopology.h` |
| `Mesh` | Complete mesh with points & topology | `MRMesh/MRMesh.h` |
| `PointCloud` | Point cloud representation | `MRMesh/MRPointCloud.h` |
| `Polyline` | Polyline representation | `MRMesh/MRPolyline.h` |
| `MeshBuilder` | Mesh construction utilities | `MRMesh/MRMeshBuilder.h` |

**Dependencies:** TBB (optional for parallelism)

---

### Phase 3: Spatial Indexing (AABB Trees)

**Goal:** Enable fast spatial queries for collision detection and nearest-neighbor searches.

| Component | Description | Source Reference |
|-----------|-------------|------------------|
| `AABBTree` | AABB tree for meshes | `MRMesh/MRAABBTree.h` |
| `AABBTreePoints` | AABB tree for point clouds | `MRMesh/MRAABBTreePoints.h` |
| `AABBTreePolyline` | AABB tree for polylines | `MRMesh/MRAABBTreePolyline.h` |

---

### Phase 4: Core Algorithms - Mesh Processing

**Goal:** Implement essential mesh manipulation algorithms.

#### 4.1 Boolean Operations
| Algorithm | Description | Source Reference |
|-----------|-------------|------------------|
| `boolean()` | CSG operations (Union, Intersection, Difference) | `MRMesh/MRMeshBoolean.h` |
| `BooleanOperation` enum | Operation types | `MRMesh/MRBooleanOperation.h` |

#### 4.2 Mesh Repair
| Algorithm | Description | Source Reference |
|-----------|-------------|------------------|
| `fixSelfIntersections()` | Fix self-intersecting meshes | `MRMesh/MRFixSelfIntersections.h` |
| `fillHole()` | Fill mesh holes | `MRMesh/MRMeshFillHole.h` |
| `fillHoleNicely()` | Advanced hole filling | `MRMesh/MRFillHoleNicely.h` |
| `MeshFixer` | General mesh fixing | `MRMesh/MRMeshFixer.h` |

#### 4.3 Mesh Simplification
| Algorithm | Description | Source Reference |
|-----------|-------------|------------------|
| `decimateMesh()` | Edge collapse simplification | `MRMesh/MRMeshDecimate.h` |
| `subdivideMesh()` | Mesh subdivision | `MRMesh/MRMeshSubdivide.h` |
| `remesh()` | Isotropic remeshing | `MRMesh/MRMeshRelax.h` |

#### 4.4 Mesh Deformation
| Algorithm | Description | Source Reference |
|-----------|-------------|------------------|
| `Laplacian` | Laplacian deformation | `MRMesh/MRLaplacian.h` |
| `FreeFormDeformer` | FFD cage deformation | `MRMesh/MRFreeFormDeformer.h` |
| `relaxMesh()` | Mesh smoothing/relaxation | `MRMesh/MRMeshRelax.h` |

#### 4.5 Point Cloud Processing
| Algorithm | Description | Source Reference |
|-----------|-------------|------------------|
| `triangulatePointCloud()` | Point cloud to mesh | `MRMesh/MRPointCloudTriangulation.h` |
| `makeNormals()` | Normal estimation | `MRMesh/MRPointCloudMakeNormals.h` |

#### 4.6 Alignment (ICP)
| Algorithm | Description | Source Reference |
|-----------|-------------|------------------|
| `ICP` | Iterative Closest Point | `MRMesh/MRICP.h` |
| `MultiwayICP` | Multi-mesh alignment | `MRMesh/MRMultiwayICP.h` |

#### 4.7 Collision Detection
| Algorithm | Description | Source Reference |
|-----------|-------------|------------------|
| `findCollidingTriangles()` | Triangle-triangle collision | `MRMesh/MRMeshCollide.h` |
| `findSelfCollidingTriangles()` | Self-intersection detection | `MRMesh/MRMeshCollide.h` |

---

### Phase 5: Voxel Operations (Optional Module)

**Goal:** Volume-based operations for advanced mesh processing.

| Component | Description | Source Reference |
|-----------|-------------|------------------|
| `offsetMesh()` | Mesh offsetting via voxels | `MRVoxels/MROffset.h` |
| `mcOffsetMesh()` | Marching cubes offset | `MRVoxels/MROffset.h` |
| `MarchingCubes` | Isosurface extraction | `MRVoxels/MRMarchingCubes.h` |
| `VDBConversions` | OpenVDB integration | `MRVoxels/MRVDBConversions.h` |

**Dependencies:** OpenVDB (optional, can be disabled for WASM)

---

### Phase 6: File I/O

**Goal:** Support common 3D file formats.

| Format | Load | Save | Source Reference |
|--------|------|------|------------------|
| STL (binary/ASCII) | ✅ | ✅ | `MRMesh/MRMeshLoad.h`, `MRMesh/MRMeshSave.h` |
| OBJ | ✅ | ✅ | `MRMesh/MRMeshLoadObj.h`, `MRMesh/MRMeshSaveObj.h` |
| PLY | ✅ | ✅ | `MRMesh/MRMeshLoad.h`, `MRMesh/MRMeshSave.h` |
| OFF | ✅ | ✅ | `MRMesh/MRMeshLoad.h`, `MRMesh/MRMeshSave.h` |
| MRMESH (native) | ✅ | ✅ | `MRMesh/MRMeshLoad.h`, `MRMesh/MRMeshSave.h` |

---

### Phase 7: Bindings Layer

#### 7.1 C API (for cross-language interop)
Based on the existing `MRMeshC` module - provides a C-compatible interface.

#### 7.2 Python Bindings (pybind11)
Based on the existing `MRPython` module structure.

#### 7.3 WebAssembly/JavaScript Bindings
Using Embind or direct Emscripten bindings for Three.js integration.

---

## Directory Structure

```
meshlib-core/
├── CMakeLists.txt                 # Main CMake configuration
├── CMakePresets.json              # Build presets (native, wasm, python)
├── README.md                      # This file
│
├── include/                       # Public headers
│   └── meshlib/
│       ├── core/                  # Data structures
│       │   ├── Vector.h
│       │   ├── Matrix.h
│       │   ├── AffineXf.h
│       │   ├── Box.h
│       │   ├── Id.h
│       │   ├── BitSet.h
│       │   └── ...
│       ├── mesh/                  # Mesh types
│       │   ├── Mesh.h
│       │   ├── MeshTopology.h
│       │   ├── PointCloud.h
│       │   └── ...
│       ├── algorithms/            # Processing algorithms
│       │   ├── Boolean.h
│       │   ├── Decimate.h
│       │   ├── FillHole.h
│       │   ├── ICP.h
│       │   ├── Laplacian.h
│       │   ├── Collision.h
│       │   └── ...
│       ├── spatial/               # Spatial indexing
│       │   ├── AABBTree.h
│       │   └── ...
│       ├── io/                    # File I/O
│       │   ├── MeshLoad.h
│       │   ├── MeshSave.h
│       │   └── ...
│       └── voxels/                # Voxel operations (optional)
│           ├── Offset.h
│           └── ...
│
├── src/                           # Implementation files
│   ├── core/
│   ├── mesh/
│   ├── algorithms/
│   ├── spatial/
│   ├── io/
│   └── voxels/
│
├── bindings/                      # Language bindings
│   ├── c/                         # C API
│   │   ├── CMakeLists.txt
│   │   ├── meshlib_c.h
│   │   └── meshlib_c.cpp
│   ├── python/                    # Python bindings (pybind11)
│   │   ├── CMakeLists.txt
│   │   └── bindings.cpp
│   └── wasm/                      # WebAssembly bindings
│       ├── CMakeLists.txt
│       ├── bindings.cpp           # Embind bindings
│       ├── meshlib.d.ts           # TypeScript definitions
│       └── three_integration.ts   # Three.js helpers
│
├── examples/                      # Usage examples
│   ├── cpp/
│   ├── python/
│   └── web/                       # Three.js examples
│       ├── index.html
│       └── app.js
│
├── tests/                         # Unit tests
│
└── thirdparty/                    # Dependencies (submodules/headers)
    ├── eigen/
    ├── parallel-hashmap/
    ├── pybind11/
    └── tbb/ (optional)
```

---

## Build Configuration

### Native C++ Build
```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

### WebAssembly Build
```bash
emcmake cmake -B build-wasm \
    -DMESHLIB_WASM=ON \
    -DMESHLIB_VOXELS=OFF
cmake --build build-wasm
```

### Python Module Build
```bash
cmake -B build-python \
    -DMESHLIB_PYTHON=ON \
    -DPython_EXECUTABLE=$(which python)
cmake --build build-python
```

---

## Three.js Integration Example

```javascript
import { MeshLibCore } from './meshlib.js';

async function processMesh() {
    const meshlib = await MeshLibCore();
    
    // Load mesh from ArrayBuffer (e.g., fetched STL file)
    const mesh = meshlib.loadMeshFromBuffer(stlBuffer, 'stl');
    
    // Simplify mesh
    const decimateSettings = new meshlib.DecimateSettings();
    decimateSettings.maxError = 0.01;
    meshlib.decimateMesh(mesh, decimateSettings);
    
    // Convert to Three.js geometry
    const geometry = new THREE.BufferGeometry();
    const vertices = mesh.getVertices();  // Float32Array
    const indices = mesh.getIndices();    // Uint32Array
    
    geometry.setAttribute('position', new THREE.BufferAttribute(vertices, 3));
    geometry.setIndex(new THREE.BufferAttribute(indices, 1));
    
    // Create Three.js mesh
    const threeMesh = new THREE.Mesh(geometry, material);
    scene.add(threeMesh);
}
```

---

## Implementation Priority

### MVP (Minimum Viable Product)
1. ✅ Core data structures (Phase 1)
2. ✅ Mesh topology & geometry (Phase 2)
3. ✅ AABB trees (Phase 3)
4. ✅ Basic I/O (STL, OBJ) (Phase 6)
5. ✅ Decimation & subdivision (Phase 4.3)
6. ✅ WebAssembly bindings (Phase 7.3)

### Extended Features
7. Boolean operations (Phase 4.1)
8. Hole filling & repair (Phase 4.2)
9. ICP alignment (Phase 4.6)
10. Collision detection (Phase 4.7)
11. Python bindings (Phase 7.2)

### Advanced Features
12. Laplacian deformation (Phase 4.4)
13. Point cloud triangulation (Phase 4.5)
14. Voxel operations (Phase 5) - requires OpenVDB

---

## Key Design Decisions

### 1. Header Organization
- Public API in `include/meshlib/`
- Implementation details hidden in `src/`
- Single `meshlib.h` umbrella header for convenience

### 2. Memory Management
- RAII-based design
- Smart pointers for ownership
- Raw pointers for non-owning references
- WebAssembly: careful memory management with explicit cleanup

### 3. Error Handling
- `Expected<T>` (tl::expected) for fallible operations
- No exceptions by default (configurable)
- WASM-friendly error reporting

### 4. Threading
- TBB for parallel algorithms (optional)
- Single-threaded fallback for WASM
- Thread-safe caches (AABB trees)

### 5. Platform Specifics
- **Native:** Full feature set, parallel processing
- **WASM:** No threading (or SharedArrayBuffer), no OpenVDB, minimal I/O
- **Python:** NumPy integration, zero-copy where possible

---

## Dependencies Summary

| Dependency | Type | Required | Notes |
|------------|------|----------|-------|
| Eigen | Header-only | Yes | Math operations |
| parallel-hashmap | Header-only | Yes | Hash maps |
| tl-expected | Header-only | Yes | Error handling |
| TBB | Library | Optional | Parallelism |
| pybind11 | Header-only | Python only | Python bindings |
| OpenVDB | Library | Voxels only | Volume operations |
| libzip | Library | I/O only | ZIP archive support |
| spdlog | Library | Optional | Logging |
| fmt | Library | Optional | String formatting |

---

## Next Steps

1. **Create the directory structure** as outlined above
2. **Extract and adapt core headers** from MRMesh
3. **Set up CMake configuration** with proper targets
4. **Implement WASM bindings** with Embind
5. **Create Three.js integration layer**
6. **Add Python bindings** via pybind11
7. **Write examples and documentation**

---

## License

This library is derived from MeshLib (https://github.com/MeshInspector/MeshLib).
See the original LICENSE file for licensing terms.
