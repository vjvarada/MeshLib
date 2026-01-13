# MeshLib Standalone Library

## Full-Featured Python and WebAssembly Build

**Version 3.0 - January 13, 2026**

---

## What This Project Creates

A **standalone distribution** of MeshLib that produces two fully-functional libraries:

| Output | Format | Use Case |
|--------|--------|----------|
| **Python Package** | `meshlib-*.whl` | Python scripts, ML pipelines, automation |
| **WebAssembly Module** | `meshlib.wasm` + `meshlib.js` | Browser apps, three.js integration |

Both outputs have **100% feature parity** with the main MeshLib library (excluding GUI components).

---

## Key Features Included

### Core Mesh Operations (MRMesh)
- Boolean operations (union, difference, intersection)
- Mesh decimation and subdivision
- Hole filling and mesh repair
- Smoothing and remeshing
- ICP alignment
- File I/O (STL, OBJ, PLY, OFF, GLTF, 3MF)

### Voxel Operations (MRVoxels + OpenVDB)
- **Mesh Offset** (expand/shrink meshes) â† Critical feature
- Marching cubes
- Mesh to signed distance field
- Volume boolean operations
- DICOM medical imaging support

### What's Excluded
- MRViewer (GUI)
- MRCommonPlugins (GUI plugins)
- imgui, GLFW, OpenGL
- MRCuda (not portable to WASM)

---

## CRITICAL PRINCIPLE: Copy, Don't Rewrite

**This project copies existing MeshLib source code - it does NOT rewrite functionality.**

| Action | Source | Destination |
|--------|--------|-------------|
| **COPY** | `source/MRMesh/` (500+ files) | Core algorithms |
| **COPY** | `source/MRVoxels/` (~60 files) | OpenVDB integration |
| **COPY** | `source/MRMeshC/` (76 headers) | C API for WASM bindings |
| **COPY** | `source/MRPython/` + `mrmeshnumpy/` | Python bindings |
| **COPY** | `scripts/mrbind/` + `mrbind-pybind11/` | Binding generators |
| **WRITE** | `source/MRWasm/MRWasm.cpp` (~300 lines) | Embind wrapper only |
| **WRITE** | `packages/meshlib-threejs/` (~1000 lines) | TypeScript wrapper |

**New code: ~1600 lines | Copied code: ~50,000+ lines**

---

## Critical Discovery: OpenVDB Works in WASM

MeshLib has **already solved** the OpenVDB WebAssembly compilation:

```bash
# From scripts/thirdparty/openvdb.sh
-D OPENVDB_USE_DELAYED_LOADING=OFF  # Required for WASM
-D USE_EXPLICIT_INSTANTIATION=OFF    # Reduces binary size
```

This means **full mesh offset functionality works in the browser!**

---

## Implementation Phases

Detailed instructions are in the `phases/` folder:

| Phase | File | Duration | Description |
|-------|------|----------|-------------|
| 0 | [PHASE_0_OVERVIEW.md](phases/PHASE_0_OVERVIEW.md) | - | Philosophy, architecture, dependency map |
| 1 | [PHASE_1_PROJECT_SETUP.md](phases/PHASE_1_PROJECT_SETUP.md) | 1 week | Directory structure, CMake configuration |
| 2 | [PHASE_2_MRMESH_CORE.md](phases/PHASE_2_MRMESH_CORE.md) | 2 weeks | Core library (~500 files), native build |
| 3 | [PHASE_3_MRVOXELS_OPENVDB.md](phases/PHASE_3_MRVOXELS_OPENVDB.md) | 1.5 weeks | OpenVDB integration, mesh offset |
| 4 | [PHASE_4_PYTHON_BINDINGS.md](phases/PHASE_4_PYTHON_BINDINGS.md) | 1.5 weeks | Python wheels with pybind11 |
| 5 | [PHASE_5_WEBASSEMBLY.md](phases/PHASE_5_WEBASSEMBLY.md) | 2-3 weeks | Emscripten build, Embind bindings |
| 6 | [PHASE_6_THREEJS_INTEGRATION.md](phases/PHASE_6_THREEJS_INTEGRATION.md) | 1-2 weeks | TypeScript API, npm package |

**Total: 9-12 weeks**

---

## Quick Start

### 1. Read Phase 0 First
Understand the architecture and philosophy:
```
phases/PHASE_0_OVERVIEW.md
```

### 2. Follow Phases Sequentially
Each phase builds on the previous. Don't skip phases.

### 3. Use the Checklists
Every phase ends with a completion checklist. Don't proceed until all items pass.

---

## Final Deliverables

### Python Package
```python
# Install
pip install meshlib-standalone-*.whl

# Use
import meshlib
mesh = meshlib.load_mesh("model.stl")
offset_mesh = meshlib.offset_mesh(mesh, 0.1)
meshlib.save_mesh(offset_mesh, "offset.stl")
```

### WebAssembly + Three.js
```javascript
import { MeshLib } from '@meshlib/threejs';

const meshLib = new MeshLib();
await meshLib.init();

const geometry = new THREE.BoxGeometry(1, 1, 1);
const result = await meshLib.offset(geometry, { offset: 0.1 });
scene.add(new THREE.Mesh(result.geometry, material));
```

---

## Architecture

```
+-----------------------------------------------------------+
|  Python (meshlib)  |  WASM + three.js  |  Native C++      | <- Bindings
+-----------------------------------------------------------+
|     MRVoxels  |  MRIOExtras  |  MRSymbolMesh (opt)        | <- Extended Libs
+-----------------------------------------------------------+
|                       MRMesh                              | <- Core Library
+-----------------------------------------------------------+
|  Boost | Eigen | TBB | OpenVDB | fmt | spdlog | etc.      | <- Dependencies
+-----------------------------------------------------------+
```

---

## Source Modules

### CRITICAL: Copy ALL Source Code From MeshLib

**DO NOT write implementations from scratch.** Copy the following modules entirely:

| Module | Location | Files | Purpose | MUST COPY |
|--------|----------|-------|---------|-----------|
| MRPch | `source/MRPch/` | ~5 | Precompiled headers | ✅ YES |
| MRMesh | `source/MRMesh/` | ~500 | Core algorithms (boolean, decimate, ICP, etc.) | ✅ YES |
| MRMeshC | `source/MRMeshC/` | ~150 | C API (76 headers + implementations) | ✅ YES |
| MRVoxels | `source/MRVoxels/` | ~60 | OpenVDB operations, mesh offset | ✅ YES |
| MRIOExtras | `source/MRIOExtras/` | ~25 | GLTF, 3MF, E57, LAS, STEP, JPEG, PNG | ✅ YES |
| MRPython | `source/MRPython/` | ~30 | Python bindings infrastructure | ✅ YES |
| mrmeshnumpy | `source/mrmeshnumpy/` | ~5 | NumPy array integration | ✅ YES |
| MRSymbolMesh | `source/MRSymbolMesh/` | ~10 | Text-to-mesh conversion | Optional |

### Copy Build Scripts & Tools

| Script/Tool | Location | Purpose | MUST COPY |
|-------------|----------|---------|-----------|
| mrbind | `scripts/mrbind/` | Python binding generator | ✅ YES |
| mrbind-pybind11 | `scripts/mrbind-pybind11/` | Modified pybind11 fork | ✅ YES |
| thirdparty scripts | `scripts/thirdparty/` | OpenVDB, blosc, TBB builds | ✅ YES |
| build_thirdparty.sh | `scripts/` | Main orchestration script | ✅ YES |

### What To Write (Minimal New Code)

| File | Lines | Purpose |
|------|-------|---------|
| `CMakeLists.txt` (root) | ~200 | Standalone build configuration |
| `CMakePresets.json` | ~100 | Build presets for native/WASM |
| `source/MRWasm/MRWasm.cpp` | ~300 | Embind wrappers for MRMeshC |
| `packages/meshlib-threejs/` | ~1000 | TypeScript wrappers for three.js |

**Summary:**
- **Lines of copied code: ~50,000+**
- **Lines of new code: ~1,600**
- **New code percentage: ~3%**

---

## Dependencies

### Required (All Platforms)
- Boost (headers + filesystem, iostreams)
- Eigen3
- TBB (Intel Threading Building Blocks)
- fmt
- spdlog
- jsoncpp
- libzip
- tl-expected

### For Voxels (MRVoxels)
- OpenVDB 10
- blosc

### For Python
- pybind11
- NumPy

### For WebAssembly
- Emscripten SDK

---

## Success Criteria

The implementation is complete when:

1. **Native Build**: `cmake --build` produces working libraries
2. **Python**: `pip install` works, all operations functional
3. **WASM**: Browser loads module, mesh offset works
4. **Feature Parity**: Results match main MeshLib exactly

---

*Document Version: 3.0*
*See `phases/` folder for detailed implementation instructions.*
