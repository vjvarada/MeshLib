# MeshLib Standalone Implementation - Overview

## Philosophy: Full Feature Parity

**This standalone implementation must maintain 100% feature parity with the main MeshLib library.**

We are NOT creating a "lite" version. We are creating a **standalone distribution** that:
- Has the SAME algorithms
- Has the SAME precision
- Has the SAME file format support  
- Has the SAME performance
- Just WITHOUT the GUI (MRViewer, imgui)

---

## CRITICAL: Copy, Don't Rewrite

**The entire implementation is based on COPYING existing MeshLib source code.**

### ⚠️ GOLDEN RULE: Never Reimplement Algorithms

Every mesh processing algorithm (boolean, decimation, offset, ICP, etc.) already exists in MeshLib and is **tested, optimized, and production-ready**. Do not write your own version.

### What to COPY (existing code):
| Source | Files | Purpose | Action |
|--------|-------|---------|--------|
| `source/MRMesh/` | ~500 | All core algorithms | **COPY ENTIRELY** |
| `source/MRMeshC/` | ~150 | C API (for WASM bindings) | **COPY ENTIRELY** |
| `source/MRVoxels/` | ~60 | OpenVDB, mesh offset | **COPY ENTIRELY** |
| `source/MRIOExtras/` | ~25 | Extended file formats | **COPY ENTIRELY** |
| `source/MRPython/` | ~30 | Python binding infrastructure | **COPY ENTIRELY** |
| `source/mrmeshnumpy/` | ~5 | NumPy integration | **COPY ENTIRELY** |
| `scripts/mrbind/` | ~10 | Python binding generator | **COPY ENTIRELY** |
| `scripts/mrbind-pybind11/` | - | Modified pybind11 fork | **COPY ENTIRELY** |
| `scripts/thirdparty/` | ~20 | Dependency build scripts | **COPY ENTIRELY** |

### What to WRITE (new code - minimal):
| File | Purpose | Lines |
|------|---------|-------|
| `CMakeLists.txt` (root) | Standalone CMake config | ~200 |
| `CMakePresets.json` | Build presets | ~100 |
| `source/MRWasm/MRWasm.cpp` | Embind wrapper for MRMeshC | ~300 |
| `packages/meshlib-threejs/` | TypeScript wrapper | ~1000 |

### Code Statistics:
```
┌─────────────────────────────────────┐
│  COPIED CODE:    ~50,000+ lines     │  97%
│  NEW CODE:       ~1,600 lines       │   3%
└─────────────────────────────────────┘
```

### Why Copy Instead of Rewrite?
1. **Correctness**: MeshLib's code is tested and proven in production
2. **Maintenance**: Updates to MeshLib can be re-copied to get fixes/features
3. **Feature parity**: Guaranteed - nothing is missing
4. **No bugs**: Avoid introducing new bugs from reimplementation
5. **Performance**: MeshLib is highly optimized with SIMD, threading, etc.

---

## What We're Building

### Target Platforms
| Platform | Build Output | Use Case |
|----------|-------------|----------|
| **Native (Windows/Linux/macOS)** | `.dll/.so/.dylib` | C++ applications |
| **Python** | `.pyd/.so` wheels | Python scripts, ML pipelines |
| **WebAssembly** | `.wasm + .js` | Browser apps, three.js |

### Core Functionality (ALL must work on ALL platforms)

| Category | Features | MeshLib Module |
|----------|----------|----------------|
| **Mesh Operations** | Boolean, decimation, subdivision, hole filling, smoothing, repair | MRMesh |
| **Point Clouds** | Loading, normals estimation, triangulation, filtering | MRMesh |
| **Voxel/Volume** | Mesh↔VDB conversion, marching cubes, **mesh offset**, volume boolean | MRVoxels |
| **Distance Fields** | Signed/unsigned distance, fast winding number | MRMesh + MRVoxels |
| **ICP Alignment** | Point-to-point, point-to-plane, multi-mesh alignment | MRMesh |
| **File I/O** | STL, OBJ, PLY, OFF, GLTF, 3MF, E57, LAS/LAZ, DICOM | MRMesh + MRIOExtras |
| **Spatial Queries** | AABB trees, ray casting, closest point, collision detection | MRMesh |

---

## Architecture Comparison

### Main MeshLib
```
┌─────────────────────────────────────────────────────────┐
│                    MRViewerApp                          │ ← GUI Application
├─────────────────────────────────────────────────────────┤
│         MRViewer + MRCommonPlugins + imgui              │ ← GUI Framework
├─────────────────────────────────────────────────────────┤
│     MRVoxels  │  MRIOExtras  │  MRSymbolMesh  │ MRCuda │ ← Extended Libs
├─────────────────────────────────────────────────────────┤
│                       MRMesh                            │ ← Core Library
├─────────────────────────────────────────────────────────┤
│  Boost │ Eigen │ TBB │ OpenVDB │ fmt │ spdlog │ etc.   │ ← Dependencies
└─────────────────────────────────────────────────────────┘
```

### Standalone MeshLib
```
┌─────────────────────────────────────────────────────────┐
│  Python (meshlib)  │  WASM + three.js  │  Native C++   │ ← Bindings
├─────────────────────────────────────────────────────────┤
│     MRVoxels  │  MRIOExtras  │  MRSymbolMesh (opt)     │ ← Extended Libs
├─────────────────────────────────────────────────────────┤
│                       MRMesh                            │ ← Core Library
├─────────────────────────────────────────────────────────┤
│  Boost │ Eigen │ TBB │ OpenVDB │ fmt │ spdlog │ etc.   │ ← Dependencies
└─────────────────────────────────────────────────────────┘

❌ EXCLUDED: MRViewer, MRViewerApp, MRCommonPlugins, imgui, GLFW, OpenGL
❌ EXCLUDED: MRCuda (not portable to WASM)
```

---

## Phase Breakdown

| Phase | Duration | Module | Deliverable |
|-------|----------|--------|-------------|
| **Phase 1** | 1 week | Setup | Project structure, CMake, build presets |
| **Phase 2** | 2 weeks | MRMesh | Core library compiling standalone |
| **Phase 3** | 1.5 weeks | MRVoxels | OpenVDB, mesh offset, marching cubes |
| **Phase 4** | 2 weeks | Python | mrmeshpy + mrmeshnumpy wheels |
| **Phase 5** | 2-3 weeks | WASM | Browser-ready .wasm with full features |
| **Phase 6** | 2 weeks | Three.js | TypeScript API, npm packages |

**Total: 10-12 weeks**

---

## Critical Success Criteria

### Phase 2 (MRMesh) Must Pass:
- [ ] All ~500 source files compile
- [ ] Unit tests from MRTest pass
- [ ] Boolean operations work identically
- [ ] ICP alignment works identically
- [ ] File I/O works for core formats (STL, OBJ, PLY)

### Phase 3 (MRVoxels) Must Pass:
- [ ] OpenVDB links correctly
- [ ] `generalOffsetMesh()` produces identical results to main lib
- [ ] `marchingCubes()` produces identical meshes
- [ ] DICOM loading works (if GDCM enabled)

### Phase 4 (Python) Must Pass:
- [ ] `import meshlib.mrmeshpy` works
- [ ] All public APIs accessible from Python
- [ ] NumPy array conversion works
- [ ] Test scripts from `test_python/` pass

### Phase 5 (WASM) Must Pass:
- [ ] Module loads in browser
- [ ] Threading works (SharedArrayBuffer)
- [ ] `generalOffsetMesh()` works in browser
- [ ] Memory doesn't leak
- [ ] Performance within 3x of native

### Phase 6 (Three.js) Must Pass:
- [ ] TypeScript compiles
- [ ] Mesh conversion to BufferGeometry works
- [ ] Operations return valid three.js objects
- [ ] Works in React/Vue/vanilla JS

---

## File Structure

```
meshlib-standalone/
├── CMakeLists.txt                    # Root build config
├── CMakePresets.json                 # Build presets
├── vcpkg.json                        # Dependencies
│
├── cmake/
│   └── Modules/                      # CMake helpers
│       ├── CompilerOptions.cmake
│       ├── ConfigureEmscripten.cmake
│       ├── ConfigureVcpkg.cmake
│       ├── DefaultOptions.cmake
│       └── DetectPlatform.cmake
│
├── source/
│   ├── MRPch/                        # Precompiled headers (~5 files)
│   ├── MRMesh/                       # Core library (~500 files)
│   ├── MRVoxels/                     # Voxel operations (~60 files)
│   ├── MRIOExtras/                   # Extra I/O formats (~25 files)
│   ├── MRPython/                     # Python binding infra (~8 files)
│   └── mrmeshnumpy/                  # NumPy integration (~5 files)
│
├── thirdparty/
│   ├── parallel-hashmap/             # Header-only
│   ├── expected/                     # Header-only (tl::expected)
│   ├── openvdb/                      # OpenVDB v10
│   ├── c-blosc/                      # Blosc compression
│   ├── onetbb/                       # Intel TBB
│   ├── mrbind-pybind11/              # Modified pybind11
│   └── ...                           # Other deps
│
├── wasm/
│   ├── bindings/                     # Embind bindings
│   │   ├── MeshLibBindings.cpp
│   │   ├── MeshLibThree.ts
│   │   └── MeshLib.d.ts
│   └── examples/
│
├── python/
│   ├── meshlib/
│   │   ├── __init__.py
│   │   └── py.typed
│   └── setup.py
│
├── tests/
│   ├── cpp/                          # C++ unit tests
│   ├── python/                       # Python tests
│   └── wasm/                         # Browser tests
│
└── scripts/
    ├── build_thirdparty.sh
    ├── build_wasm.sh
    └── thirdparty/
        ├── openvdb.sh
        ├── blosc.sh
        └── ...
```

---

## Dependency Map

```
                    ┌────────────────────────────────────────┐
                    │           External Dependencies        │
                    ├────────────────────────────────────────┤
                    │ Boost::headers      (header-only)      │
                    │ Eigen3::Eigen       (header-only)      │
                    │ TBB::tbb            (threading)        │
                    │ fmt::fmt            (formatting)       │
                    │ spdlog::spdlog      (logging)          │
                    │ JsonCpp::JsonCpp    (JSON parsing)     │
                    │ libzip::zip         (ZIP archives)     │
                    │ tl::expected        (error handling)   │
                    │ OpenVDB::openvdb    (voxel grids)      │
                    │ blosc               (compression)      │
                    │ GDCM                (DICOM - optional) │
                    │ E57Format           (E57 - optional)   │
                    └────────────────────────────────────────┘
                                        │
                                        ▼
┌──────────────────────────────────────────────────────────────────┐
│                           MRMesh                                  │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │
│  │ Data Structures │  │   Algorithms    │  │      I/O        │  │
│  │ • Mesh          │  │ • Boolean       │  │ • STL           │  │
│  │ • Topology      │  │ • Decimate      │  │ • OBJ           │  │
│  │ • PointCloud    │  │ • FillHole      │  │ • PLY           │  │
│  │ • Polyline      │  │ • ICP           │  │ • OFF           │  │
│  │ • AABBTree      │  │ • Distance      │  │ • MeshLoad/Save │  │
│  │ • BitSet        │  │ • Relax         │  │ • PointsLoad    │  │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘  │
└──────────────────────────────────────────────────────────────────┘
                                        │
                    ┌───────────────────┼───────────────────┐
                    ▼                   ▼                   ▼
        ┌──────────────────┐ ┌──────────────────┐ ┌──────────────────┐
        │    MRVoxels      │ │   MRIOExtras     │ │   MRSymbolMesh   │
        │ • OpenVDB grids  │ │ • GLTF           │ │ • Text→Mesh      │
        │ • Offset         │ │ • 3MF            │ │   (optional)     │
        │ • MarchingCubes  │ │ • E57            │ │                  │
        │ • VDB Boolean    │ │ • LAS/LAZ        │ │                  │
        │ • DICOM          │ │ • STEP           │ │                  │
        │ • SDF            │ │ • JPEG/PNG       │ │                  │
        └──────────────────┘ └──────────────────┘ └──────────────────┘
                    │                   │
                    └───────────────────┼───────────────────┐
                                        ▼                   ▼
                            ┌──────────────────┐ ┌──────────────────┐
                            │    MRPython      │ │   WASM Module    │
                            │ • pybind11       │ │ • Embind         │
                            │ • NumPy          │ │ • three.js       │
                            │ • Wheels         │ │ • TypeScript     │
                            └──────────────────┘ └──────────────────┘
```

---

## Quality Gates

Before proceeding to the next phase, ALL items in the checklist must pass:

### Automated Tests
- Unit tests must pass
- Memory leak checks (Valgrind/ASan)
- Performance benchmarks within tolerance

### Manual Verification
- Compare output with main MeshLib
- Binary comparison of exported meshes
- Visual inspection of complex operations

### Documentation
- API changes documented
- Build instructions verified
- Example code tested

---

## Next Steps

1. Read **PHASE_1_PROJECT_SETUP.md** to begin
2. Execute steps in order
3. Don't skip verification steps
4. Report issues in this document

---

## Phase 0 Completion Checklist

### Architecture Understanding ✅
- [x] Main MeshLib structure analyzed (source/, cmake/, scripts/)
- [x] Core modules identified (MRMesh: 729 files, MRVoxels: 85 files, MRMeshC: 149 files)
- [x] Extended modules verified (MRIOExtras: 30, MRPython: 9, mrmeshnumpy: 5)
- [x] Existing CMake build system understood
- [x] Emscripten configuration verified in cmake/Modules/ConfigureEmscripten.cmake
- [x] OpenVDB WASM flags confirmed: `-D OPENVDB_USE_DELAYED_LOADING=OFF -D USE_EXPLICIT_INSTANTIATION=OFF`

### Module Inventory Verified ✅
| Module | Files | Status |
|--------|-------|--------|
| MRPch | 20 | ✅ Verified |
| MRMesh | 729 | ✅ Verified (larger than estimated 500) |
| MRMeshC | 149 | ✅ Verified |
| MRVoxels | 85 | ✅ Verified |
| MRIOExtras | 30 | ✅ Verified |
| MRPython | 9 | ✅ Verified |
| mrmeshnumpy | 5 | ✅ Verified |
| MRSymbolMesh | 12 | ✅ Verified |

### Scripts & Tools Verified ✅
| Location | Purpose | Status |
|----------|---------|--------|
| scripts/mrbind/ | Python binding generator | ✅ Present |
| scripts/mrbind-pybind11/ | Modified pybind11 fork | ✅ Present |
| scripts/thirdparty/ | Dependency build scripts | ✅ Present |
| scripts/build_thirdparty.sh | Main orchestration | ✅ Present |

### Test Infrastructure Verified ✅
- [x] C++ unit tests: source/MRTest/ (52 test files)
- [x] Python tests: test_python/ (58 test files)
- [x] Test categories cover: boolean, ICP, decimation, offset, voxels, file I/O

### CMake Build Options Understood ✅
| Option | Default | Standalone |
|--------|---------|------------|
| MESHLIB_BUILD_MRVIEWER | ON | **OFF** |
| MESHLIB_BUILD_MESHVIEWER | ON | **OFF** |
| MESHLIB_BUILD_VOXELS | ON | ON |
| MESHLIB_BUILD_EXTRA_IO_FORMATS | ON | ON |
| MESHLIB_BUILD_PYTHON_MODULES | ON | ON |
| MESHLIB_EXPERIMENTAL_BUILD_C_BINDING | ON | ON |
| MESHLIB_BUILD_MRCUDA | ON | **OFF** |

### Dependency Map Verified ✅
Core dependencies from cmake/Modules/:
- Boost (headers)
- Eigen3
- TBB (threading)
- fmt, spdlog (logging)
- JsonCpp, libzip (utilities)
- OpenVDB + blosc (voxels)

### Ready for Phase 1 ✅
All Phase 0 objectives completed. Proceed to PHASE_1_PROJECT_SETUP.md

---

*Document Version: 1.1*
*Created: January 13, 2026*
*Phase 0 Completed: January 13, 2026*
*Philosophy: Full feature parity, no compromises*