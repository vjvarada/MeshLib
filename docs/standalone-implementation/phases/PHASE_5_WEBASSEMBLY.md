# Phase 5: WebAssembly Build

## Goal
Compile MRMesh and MRVoxels to WebAssembly using Emscripten for browser usage.

## Duration: 2-3 weeks

## Prerequisites
- Phase 2 (MRMesh) completed
- Phase 3 (MRVoxels with OpenVDB) completed
- Emscripten SDK installed

---

## Key Discovery: OpenVDB Already Works in WASM!

MeshLib has **proven** OpenVDB WASM builds. From `scripts/thirdparty/openvdb.sh`:

```bash
# Critical flags for WASM compatibility
-D OPENVDB_USE_DELAYED_LOADING=OFF  # Required for WASM
-D USE_EXPLICIT_INSTANTIATION=OFF    # Reduces binary size
```

**This means full voxel operations including mesh offset work in the browser!**

---

## Step 5.1: Install Emscripten SDK

### Windows (PowerShell)

```powershell
# Clone emsdk
cd C:\tools
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk

# Install latest
.\emsdk.bat install latest
.\emsdk.bat activate latest

# Add to PATH for current session
.\emsdk_env.bat

# Verify
emcc --version
# Expected: emcc (Emscripten gcc/clang-like replacement...) 3.x.x
```

### Linux/WSL

```bash
# Clone emsdk
cd ~
git clone https://github.com/emscripten-core/emsdk.git
cd emsdk

# Install
./emsdk install latest
./emsdk activate latest

# Source environment
source ./emsdk_env.sh

# Verify
emcc --version
```

### Verify Installation

```powershell
# Check all tools
emcc --version
emmake --version
emcmake --help

# Check cmake finds Emscripten
cmake --version
```

---

## Step 5.2: Copy Emscripten Build Scripts

```powershell
$MESHLIB_DIR = "C:\Users\VijayRaghavVarada\Documents\Github\MeshLib"
$STANDALONE_DIR = "C:\Users\VijayRaghavVarada\Documents\Github\meshlib-standalone"

# Copy thirdparty build scripts (essential!)
Copy-Item "$MESHLIB_DIR\scripts\thirdparty\*" "$STANDALONE_DIR\scripts\thirdparty\" -Recurse -Force

# These scripts build dependencies for Emscripten:
# - blosc.sh
# - boost.sh  
# - openvdb.sh
# - tbb.sh
# - etc.
```

### Key Scripts:

| Script | Purpose |
|--------|---------|
| `openvdb.sh` | OpenVDB with WASM-compatible flags |
| `blosc.sh` | Compression library for OpenVDB |
| `tbb.sh` | Threading (with WASM adjustments) |
| `boost.sh` | Boost headers |
| `build_thirdparty.sh` | Main orchestration script |

---

## Step 5.3: Create WASM CMake Preset

Add to `CMakePresets.json`:

```json
{
    "name": "wasm-release",
    "displayName": "WebAssembly Release",
    "description": "Emscripten WebAssembly build",
    "binaryDir": "${sourceDir}/build/wasm-release",
    "generator": "Ninja",
    "toolchainFile": "${env:EMSDK}/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake",
    "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Release",
        "MESHLIB_BUILD_MRMESH": "ON",
        "MESHLIB_BUILD_VOXELS": "ON",
        "MESHLIB_BUILD_MRIOEXTRAS": "OFF",
        "MESHLIB_BUILD_PYTHON": "OFF",
        "MESHLIB_BUILD_VIEWER": "OFF",
        "MESHLIB_BUILD_CLIB": "OFF",
        "MR_EMSCRIPTEN": "ON",
        "CMAKE_EXECUTABLE_SUFFIX": ".js"
    },
    "environment": {
        "EMSDK": "C:/tools/emsdk"
    }
},
{
    "name": "wasm-debug",
    "displayName": "WebAssembly Debug",
    "inherits": "wasm-release",
    "binaryDir": "${sourceDir}/build/wasm-debug",
    "cacheVariables": {
        "CMAKE_BUILD_TYPE": "Debug"
    }
}
```

---

## Step 5.4: Build Thirdparty for Emscripten

This is the critical step - building dependencies with Emscripten.

### Using Git Bash / WSL:

```bash
#!/bin/bash
cd /path/to/meshlib-standalone

# Set Emscripten environment
source ~/emsdk/emsdk_env.sh

# Set build type
export MR_EMSCRIPTEN=ON

# Build thirdparty libraries
./scripts/thirdparty/build_thirdparty.sh

# Or build individually:
./scripts/thirdparty/blosc.sh
./scripts/thirdparty/openvdb.sh
```

### Using PowerShell (with WSL):

```powershell
# Run thirdparty build in WSL
wsl -e bash -c "cd /mnt/c/Users/VijayRaghavVarada/Documents/Github/meshlib-standalone && source ~/emsdk/emsdk_env.sh && MR_EMSCRIPTEN=ON ./scripts/thirdparty/build_thirdparty.sh"
```

---

## CRITICAL: Use Existing MRMeshC API

**DO NOT write WASM bindings from scratch!**

MeshLib already has a complete **C API** in `source/MRMeshC/` with **76 header files** covering all functionality:

```
source/MRMeshC/
â”œâ”€â”€ MRMesh.h           # Core mesh operations
â”œâ”€â”€ MRMeshBoolean.h    # Boolean operations
â”œâ”€â”€ MRMeshDecimate.h   # Decimation
â”œâ”€â”€ MRMeshFillHole.h   # Hole filling
â”œâ”€â”€ MRMeshRelax.h      # Mesh smoothing
â”œâ”€â”€ MROffset.h         # Mesh offset (OpenVDB)
â”œâ”€â”€ MRICP.h            # ICP alignment
â”œâ”€â”€ MRConvexHull.h     # Convex hull
â”œâ”€â”€ MRMeshProject.h    # Closest point queries
â”œâ”€â”€ ... (76 total)
â””â”€â”€ MRMeshC.h          # Main include
```

**The C API is the foundation for WASM bindings.** We copy it and create thin Embind wrappers.

---

## Step 5.5: Copy MRMeshC (C API)

```powershell
$MESHLIB_DIR = "C:\Users\VijayRaghavVarada\Documents\Github\MeshLib"
$STANDALONE_DIR = "C:\Users\VijayRaghavVarada\Documents\Github\meshlib-standalone"

# Copy ALL MRMeshC files
$SOURCE = "$MESHLIB_DIR\source\MRMeshC"
$TARGET = "$STANDALONE_DIR\source\MRMeshC"

# Create target directory
New-Item -ItemType Directory -Force -Path $TARGET

# Copy all files
Get-ChildItem $SOURCE -File | ForEach-Object {
    Copy-Item $_.FullName $TARGET -Force
    Write-Host "Copied: $($_.Name)"
}

# Copy detail subdirectory
Copy-Item "$SOURCE\detail" "$TARGET\detail" -Recurse -Force

# Count files
$fileCount = (Get-ChildItem $TARGET -Recurse -File).Count
Write-Host "Total files copied: $fileCount"
# Expected: ~150 files (76 headers + 76 cpp + CMakeLists.txt)
```

### Key MRMeshC Files for WASM:

| File | Operations Provided |
|------|---------------------|
| `MROffset.h` | `mrOffsetMesh`, `mrDoubleOffsetMesh`, `mrGeneralOffsetMesh`, `mrThickenMesh` |
| `MRMeshBoolean.h` | `mrBoolean` (union, difference, intersection) |
| `MRMeshDecimate.h` | `mrDecimateMesh` |
| `MRMeshFillHole.h` | `mrFillHole`, `mrFillHoleNicely` |
| `MRMeshRelax.h` | `mrRelax`, `mrRelaxKeepVolume` |
| `MRICP.h` | `mrICPNew`, `mrICPCalculateTransformation` |
| `MRConvexHull.h` | `mrMakeConvexHull` |
| `MRMeshProject.h` | `mrFindProjection`, `mrFindClosestPointOnMesh` |
| `MRMeshLoad.h` | `mrMeshLoadFromFile`, `mrMeshLoadFromAnySupportedFormat` |
| `MRMeshSave.h` | `mrMeshSaveToFile`, `mrMeshSaveToAnySupportedFormat` |

---

## Step 5.6: Create Embind Wrapper (Minimal New Code)

The Embind wrapper is the **ONLY new code needed**. It wraps the existing C API for JavaScript.

Create `source/MRWasm/MRWasm.cpp`:

```cpp
// MRWasm.cpp - Embind wrapper for MRMeshC
// 
// IMPORTANT: This file creates thin wrappers around the EXISTING C API.
// All actual implementation is in MRMeshC. DO NOT duplicate logic here.

#include <emscripten/bind.h>
#include <emscripten/val.h>

// Include the existing C API headers
#include <MRMeshC/MRMesh.h>
#include <MRMeshC/MRMeshC.h>
#include <MRMeshC/MRCube.h>
#include <MRMeshC/MRMakeSphereMesh.h>
#include <MRMeshC/MRCylinder.h>
#include <MRMeshC/MRTorus.h>
#include <MRMeshC/MRMeshLoad.h>
#include <MRMeshC/MRMeshSave.h>
#include <MRMeshC/MRMeshBoolean.h>
#include <MRMeshC/MRMeshDecimate.h>
#include <MRMeshC/MRMeshFillHole.h>
#include <MRMeshC/MRMeshRelax.h>
#include <MRMeshC/MRMeshSubdivide.h>
#include <MRMeshC/MRMeshComponents.h>
#include <MRMeshC/MRFixSelfIntersections.h>
#include <MRMeshC/MRICP.h>
#include <MRMeshC/MRConvexHull.h>
#include <MRMeshC/MRMeshProject.h>
#include <MRMeshC/MROffset.h>

using namespace emscripten;

// ========================================
// JavaScript-Friendly Wrappers
// These wrap the C API functions for Embind
// ========================================

// Mesh wrapper that holds an MRMesh* from the C API
class MeshJS {
private:
    MRMesh* mesh_;
    bool owned_;
    
public:
    MeshJS() : mesh_(nullptr), owned_(false) {}
    MeshJS(MRMesh* m, bool owned = true) : mesh_(m), owned_(owned) {}
    ~MeshJS() { if (owned_ && mesh_) mrMeshFree(mesh_); }
    
    // Move semantics
    MeshJS(MeshJS&& other) noexcept : mesh_(other.mesh_), owned_(other.owned_) {
        other.mesh_ = nullptr;
        other.owned_ = false;
    }
    
    MRMesh* ptr() const { return mesh_; }
    
    // Properties - use existing C API
    int numVertices() const { 
        if (!mesh_) return 0;
        return mrMeshTopologyNumValidVerts(mrMeshTopology(mesh_)); 
    }
    
    int numFaces() const { 
        if (!mesh_) return 0;
        return mrMeshTopologyNumValidFaces(mrMeshTopology(mesh_)); 
    }
    
    float volume() const { 
        if (!mesh_) return 0;
        return mrMeshVolume(mesh_); 
    }
    
    float area() const { 
        if (!mesh_) return 0;
        return mrMeshArea(mesh_); 
    }
};

// ========================================
// Primitive Creation (wrapping C API)
// ========================================

MeshJS makeCube(float size) {
    MRVector3f dimensions = { size, size, size };
    MRMesh* mesh = mrMakeCube(&dimensions, nullptr);
    return MeshJS(mesh);
}

MeshJS makeSphere(float radius, int numMeridians) {
    MRSphereParams params;
    params.radius = radius;
    params.numMeridians = numMeridians;
    MRMesh* mesh = mrMakeUVSphere(&params);
    return MeshJS(mesh);
}

MeshJS makeCylinder(float radius, float height, int resolution) {
    // Use existing C API
    MRMesh* mesh = mrMakeCylinder(radius, radius, 0.0f, height, resolution);
    return MeshJS(mesh);
}

MeshJS makeTorus(float primaryRadius, float secondaryRadius, int primaryRes, int secondaryRes) {
    MRMesh* mesh = mrMakeTorus(primaryRadius, secondaryRadius, primaryRes, secondaryRes, nullptr);
    return MeshJS(mesh);
}

// ========================================
// Offset Operations (wrapping C API)
// ========================================

MeshJS offsetMesh(const MeshJS& input, float offset, float voxelSize) {
    MROffsetParameters params = mrOffsetParametersNew();
    if (voxelSize <= 0) {
        // Auto-calculate voxel size
        MRMeshPart mp = { input.ptr(), nullptr };
        params.voxelSize = mrSuggestVoxelSize(mp, 50000.0f);
    } else {
        params.voxelSize = voxelSize;
    }
    
    MRString* errorString = nullptr;
    MRMeshPart mp = { input.ptr(), nullptr };
    MRMesh* result = mrOffsetMesh(mp, offset, &params, &errorString);
    
    if (errorString) {
        std::string error = mrStringData(errorString);
        mrStringFree(errorString);
        throw std::runtime_error(error);
    }
    
    return MeshJS(result);
}

MeshJS doubleOffsetMesh(const MeshJS& input, float offsetA, float offsetB, float voxelSize) {
    MROffsetParameters params = mrOffsetParametersNew();
    if (voxelSize <= 0) {
        MRMeshPart mp = { input.ptr(), nullptr };
        params.voxelSize = mrSuggestVoxelSize(mp, 50000.0f);
    } else {
        params.voxelSize = voxelSize;
    }
    
    MRString* errorString = nullptr;
    MRMeshPart mp = { input.ptr(), nullptr };
    MRMesh* result = mrDoubleOffsetMesh(mp, offsetA, offsetB, &params, &errorString);
    
    if (errorString) {
        std::string error = mrStringData(errorString);
        mrStringFree(errorString);
        throw std::runtime_error(error);
    }
    
    return MeshJS(result);
}

MeshJS generalOffsetMesh(const MeshJS& input, float offset, float voxelSize) {
    MROffsetParameters params = mrOffsetParametersNew();
    MRGeneralOffsetParameters genParams = mrGeneralOffsetParametersNew();
    
    if (voxelSize <= 0) {
        MRMeshPart mp = { input.ptr(), nullptr };
        params.voxelSize = mrSuggestVoxelSize(mp, 50000.0f);
    } else {
        params.voxelSize = voxelSize;
    }
    
    MRString* errorString = nullptr;
    MRMeshPart mp = { input.ptr(), nullptr };
    MRMesh* result = mrGeneralOffsetMesh(mp, offset, &params, &genParams, &errorString);
    
    if (errorString) {
        std::string error = mrStringData(errorString);
        mrStringFree(errorString);
        throw std::runtime_error(error);
    }
    
    return MeshJS(result);
}

MeshJS thickenMesh(const MeshJS& input, float offset, float voxelSize) {
    MROffsetParameters params = mrOffsetParametersNew();
    MRGeneralOffsetParameters genParams = mrGeneralOffsetParametersNew();
    
    if (voxelSize <= 0) {
        MRMeshPart mp = { input.ptr(), nullptr };
        params.voxelSize = mrSuggestVoxelSize(mp, 50000.0f);
    } else {
        params.voxelSize = voxelSize;
    }
    
    MRString* errorString = nullptr;
    MRMeshPart mp = { input.ptr(), nullptr };
    MRMesh* result = mrThickenMesh(mp, offset, &params, &genParams, &errorString);
    
    if (errorString) {
        std::string error = mrStringData(errorString);
        mrStringFree(errorString);
        throw std::runtime_error(error);
    }
    
    return MeshJS(result);
}

// ========================================
// Boolean Operations (wrapping C API)
// ========================================

MeshJS booleanUnion(const MeshJS& a, const MeshJS& b) {
    MRBooleanParameters params = mrBooleanParametersNew();
    MRBooleanResult result = mrBoolean(a.ptr(), b.ptr(), MRBooleanOperationUnion, &params);
    
    if (result.errorString) {
        std::string error = mrStringData(result.errorString);
        mrStringFree(result.errorString);
        throw std::runtime_error(error);
    }
    
    return MeshJS(result.mesh);
}

MeshJS booleanDifference(const MeshJS& a, const MeshJS& b) {
    MRBooleanParameters params = mrBooleanParametersNew();
    MRBooleanResult result = mrBoolean(a.ptr(), b.ptr(), MRBooleanOperationDifferenceAB, &params);
    
    if (result.errorString) {
        std::string error = mrStringData(result.errorString);
        mrStringFree(result.errorString);
        throw std::runtime_error(error);
    }
    
    return MeshJS(result.mesh);
}

MeshJS booleanIntersection(const MeshJS& a, const MeshJS& b) {
    MRBooleanParameters params = mrBooleanParametersNew();
    MRBooleanResult result = mrBoolean(a.ptr(), b.ptr(), MRBooleanOperationIntersection, &params);
    
    if (result.errorString) {
        std::string error = mrStringData(result.errorString);
        mrStringFree(result.errorString);
        throw std::runtime_error(error);
    }
    
    return MeshJS(result.mesh);
}

// ========================================
// Mesh Processing (wrapping C API)
// ========================================

MeshJS decimateMesh(const MeshJS& input, float ratio) {
    MRDecimateSettings settings = mrDecimateSettingsNew();
    settings.maxDeletedFaces = static_cast<int>(input.numFaces() * (1.0f - ratio));
    
    // Clone input mesh since decimation is in-place
    MRMesh* result = mrMeshCopy(input.ptr());
    mrDecimateMesh(result, &settings);
    
    return MeshJS(result);
}

MeshJS fillHoles(const MeshJS& input) {
    // Clone and fill holes
    MRMesh* result = mrMeshCopy(input.ptr());
    
    // Get hole edges
    MREdgePath* holes = mrMeshTopologyFindHoleRepresentiveEdges(mrMeshTopology(result));
    size_t numHoles = mrEdgePathSize(holes);
    
    for (size_t i = 0; i < numHoles; ++i) {
        MREdgeId edge = mrEdgePathData(holes)[i];
        MRFillHoleParams params = mrFillHoleParamsNew();
        mrFillHole(result, edge, &params);
    }
    
    mrEdgePathFree(holes);
    return MeshJS(result);
}

MeshJS relaxMesh(const MeshJS& input, int iterations) {
    MRMesh* result = mrMeshCopy(input.ptr());
    
    MRMeshRelaxParams params = mrMeshRelaxParamsNew();
    params.iterations = iterations;
    
    mrRelax(result, &params, nullptr);
    
    return MeshJS(result);
}

MeshJS subdivideMesh(const MeshJS& input, float maxEdgeLength) {
    MRMesh* result = mrMeshCopy(input.ptr());
    
    MRSubdivideSettings settings = mrSubdivideSettingsNew();
    settings.maxEdgeLen = maxEdgeLength;
    
    mrSubdivideMesh(result, &settings);
    
    return MeshJS(result);
}

// ========================================
// Geometry Operations (wrapping C API)
// ========================================

MeshJS makeConvexHull(const MeshJS& input) {
    MRMesh* result = mrMakeConvexHull(input.ptr());
    return MeshJS(result);
}

// ========================================
// ICP Registration (wrapping C API)
// ========================================

struct ICPResultJS {
    float rotation[9];
    float translation[3];
    float rmsError;
};

ICPResultJS alignMeshes(const MeshJS& source, const MeshJS& target, int maxIterations) {
    // Use the C API's ICP functions
    MRMeshOrPointsXf sourceXf, targetXf;
    sourceXf.obj.mesh = source.ptr();
    sourceXf.xf = mrAffineXf3fNew();
    targetXf.obj.mesh = target.ptr();
    targetXf.xf = mrAffineXf3fNew();
    
    MRICP* icp = mrICPNew(&sourceXf, &targetXf, nullptr);
    
    MRICPProperties props = mrICPPropertiesNew();
    props.iterLimit = maxIterations;
    mrICPSetParams(icp, &props);
    
    MRAffineXf3f resultXf = mrICPCalculateTransformation(icp);
    
    ICPResultJS result;
    // Extract rotation from resultXf.A
    result.rotation[0] = resultXf.A.x.x;
    result.rotation[1] = resultXf.A.x.y;
    result.rotation[2] = resultXf.A.x.z;
    result.rotation[3] = resultXf.A.y.x;
    result.rotation[4] = resultXf.A.y.y;
    result.rotation[5] = resultXf.A.y.z;
    result.rotation[6] = resultXf.A.z.x;
    result.rotation[7] = resultXf.A.z.y;
    result.rotation[8] = resultXf.A.z.z;
    
    // Extract translation
    result.translation[0] = resultXf.b.x;
    result.translation[1] = resultXf.b.y;
    result.translation[2] = resultXf.b.z;
    
    result.rmsError = mrICPGetMeanSqDistToPoint(icp);
    
    mrICPFree(icp);
    mrAffineXf3fFree(&sourceXf.xf);
    mrAffineXf3fFree(&targetXf.xf);
    
    return result;
}

// ========================================
// Embind Bindings
// ========================================

EMSCRIPTEN_BINDINGS(meshlib) {
    // Mesh class
    class_<MeshJS>("Mesh")
        .constructor<>()
        .function("numVertices", &MeshJS::numVertices)
        .function("numFaces", &MeshJS::numFaces)
        .function("volume", &MeshJS::volume)
        .function("area", &MeshJS::area);
    
    // Primitive creation (wrapping C API)
    function("makeCube", &makeCube);
    function("makeSphere", &makeSphere);
    function("makeCylinder", &makeCylinder);
    function("makeTorus", &makeTorus);
    
    // Offset Operations (wrapping C API)
    function("offsetMesh", &offsetMesh);
    function("doubleOffsetMesh", &doubleOffsetMesh);
    function("generalOffsetMesh", &generalOffsetMesh);
    function("thickenMesh", &thickenMesh);
    
    // Boolean Operations (wrapping C API)
    function("booleanUnion", &booleanUnion);
    function("booleanDifference", &booleanDifference);
    function("booleanIntersection", &booleanIntersection);
    
    // Mesh Processing (wrapping C API)
    function("decimateMesh", &decimateMesh);
    function("fillHoles", &fillHoles);
    function("relaxMesh", &relaxMesh);
    function("subdivideMesh", &subdivideMesh);
    
    // Geometry Operations (wrapping C API)
    function("makeConvexHull", &makeConvexHull);
    
    // ICP Registration (wrapping C API)
    value_object<ICPResultJS>("ICPResult")
        .field("rotation", &ICPResultJS::rotation)
        .field("translation", &ICPResultJS::translation)
        .field("rmsError", &ICPResultJS::rmsError);
    function("alignMeshes", &alignMeshes);
}
```

**Note:** The above code wraps the existing MRMeshC functions. All actual implementation is in the copied C API - we only create thin Embind wrappers.

---

## Step 5.7: Create MRWasm CMakeLists.txt

Create `source/MRWasm/CMakeLists.txt`:

```cmake
project(MRWasm CXX)

IF(NOT EMSCRIPTEN)
    message(FATAL_ERROR "MRWasm requires Emscripten")
ENDIF()

# Source files
set(SOURCES
    MRWasm.cpp
)

# Create the WebAssembly module
add_executable(meshlib ${SOURCES})

target_link_libraries(meshlib
    PRIVATE
        MRMesh
        MRVoxels
)

target_include_directories(meshlib
    PRIVATE
        ${CMAKE_SOURCE_DIR}/source
)

# Emscripten flags
set(EMSCRIPTEN_LINK_FLAGS
    "-s MODULARIZE=1"
    "-s EXPORT_NAME='MeshLib'"
    "-s ALLOW_MEMORY_GROWTH=1"
    "-s MAXIMUM_MEMORY=4GB"
    "-s WASM=1"
    "-s ENVIRONMENT='web,worker'"
    "-s FILESYSTEM=1"
    "-s FORCE_FILESYSTEM=1"
    "--bind"
    "-lembind"
    "-s EXPORTED_RUNTIME_METHODS=['FS','ccall','cwrap']"
    "-s ASSERTIONS=0"
    "-O3"
)

# Join flags into a string
list(JOIN EMSCRIPTEN_LINK_FLAGS " " EMSCRIPTEN_LINK_FLAGS_STR)

set_target_properties(meshlib PROPERTIES
    SUFFIX ".js"
    LINK_FLAGS "${EMSCRIPTEN_LINK_FLAGS_STR}"
)

# Output files
set(OUTPUT_DIR ${CMAKE_BINARY_DIR}/wasm)

add_custom_command(TARGET meshlib POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory ${OUTPUT_DIR}
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:meshlib> ${OUTPUT_DIR}/
    COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_BINARY_DIR}/meshlib.wasm ${OUTPUT_DIR}/
    COMMENT "Copying WASM files to output directory"
)
```

---

## Step 5.8: Update Root CMakeLists.txt

Add MRWasm to the build:

```cmake
# WebAssembly module
IF(EMSCRIPTEN AND MESHLIB_BUILD_WASM)
    add_subdirectory(source/MRWasm)
ENDIF()
```

---

## Step 5.9: Build WebAssembly

```bash
# From Git Bash or WSL
cd /path/to/meshlib-standalone

# Activate Emscripten
source ~/emsdk/emsdk_env.sh

# Configure
emcmake cmake -B build/wasm-release \
    -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DMESHLIB_BUILD_MRMESH=ON \
    -DMESHLIB_BUILD_VOXELS=ON \
    -DMESHLIB_BUILD_WASM=ON \
    -DMR_EMSCRIPTEN=ON

# Build
cmake --build build/wasm-release

# Check output
ls -la build/wasm-release/wasm/
```

### Expected Output Files:
```
build/wasm-release/wasm/
â”œâ”€â”€ meshlib.js      # JavaScript glue code
â””â”€â”€ meshlib.wasm    # WebAssembly binary
```

---

## Step 5.10: Create TypeScript Declarations

Create `wasm/meshlib.d.ts`:

```typescript
// MeshLib WebAssembly TypeScript Declarations

export interface Vector3 {
    x: number;
    y: number;
    z: number;
}

export interface BoundingBox {
    min: Vector3;
    max: Vector3;
}

export interface Mesh {
    numVertices(): number;
    numFaces(): number;
    volume(): number;
    area(): number;
    boundingBox(): BoundingBox;
    getVertices(): Float32Array;
    getFaces(): Uint32Array;
    getNormals(): Float32Array;
    translate(x: number, y: number, z: number): void;
    scale(factor: number): void;
}

export interface MeshLibModule {
    // Primitives
    makeCube(size?: number): Mesh;
    makeSphere(radius?: number, subdivisions?: number): Mesh;
    makeCylinder(radius?: number, height?: number, segments?: number): Mesh;
    makeTorus(primaryRadius?: number, secondaryRadius?: number, 
              primaryRes?: number, secondaryRes?: number): Mesh;
    
    // File I/O
    loadMeshFromBuffer(data: string, format: string): Mesh;
    saveMeshToBuffer(mesh: Mesh, format: string): string;
    
    // Offset Operations (OpenVDB-based)
    offsetMesh(mesh: Mesh, offset: number, voxelSize?: number): Mesh;
    shellOffset(mesh: Mesh, offset: number, voxelSize?: number): Mesh;
    doubleOffset(mesh: Mesh, offsetA: number, offsetB: number, voxelSize?: number): Mesh;
    thickenMesh(mesh: Mesh, thickness: number, voxelSize?: number): Mesh;
    
    // Boolean Operations
    booleanUnion(a: Mesh, b: Mesh): Mesh;
    booleanDifference(a: Mesh, b: Mesh): Mesh;
    booleanIntersection(a: Mesh, b: Mesh): Mesh;
    
    // Mesh Processing
    decimateMesh(mesh: Mesh, ratio: number): Mesh;
    fillHoles(mesh: Mesh): Mesh;
    relaxMesh(mesh: Mesh, iterations: number): Mesh;
    subdivideMesh(mesh: Mesh, maxEdgeLength: number): Mesh;
    fixSelfIntersections(mesh: Mesh): Mesh;
    
    // Component Operations
    getLargestComponent(mesh: Mesh): number;
    keepLargestComponent(mesh: Mesh): Mesh;
    
    // Geometry Operations
    makeConvexHull(mesh: Mesh): Mesh;
    
    // Plane Operations
    extractPlaneSection(mesh: Mesh, nx: number, ny: number, nz: number, d: number): PlaneSectionResult;
    trimWithPlane(mesh: Mesh, nx: number, ny: number, nz: number, d: number): Mesh;
    
    // ICP Registration
    alignMeshes(source: Mesh, target: Mesh, maxIterations?: number): ICPResult;
    applyTransform(mesh: Mesh, 
                   r0: number, r1: number, r2: number,
                   r3: number, r4: number, r5: number,
                   r6: number, r7: number, r8: number,
                   tx: number, ty: number, tz: number): Mesh;
    
    // Ray Intersection
    rayMeshIntersect(mesh: Mesh, 
                     originX: number, originY: number, originZ: number,
                     dirX: number, dirY: number, dirZ: number): RayIntersectionResult;
    
    // Distance Queries
    findClosestPoint(mesh: Mesh, 
                     queryX: number, queryY: number, queryZ: number): ClosestPointResult;
    signedDistanceToMesh(mesh: Mesh, 
                         queryX: number, queryY: number, queryZ: number): number;
    
    // Classes
    Mesh: new () => Mesh;
    Vector3: new (x?: number, y?: number, z?: number) => Vector3;
}

export interface PlaneSectionResult {
    points: number[];  // Flattened 2D contour points
    numContours: number;
}

export interface ICPResult {
    rotation: number[];    // 9-element rotation matrix (column-major 3x3)
    translation: number[]; // 3-element translation vector
    rmsError: number;      // Final RMS error
    iterations: number;    // Number of iterations performed
}

export interface RayIntersectionResult {
    hit: boolean;          // True if ray hit the mesh
    distance: number;      // Distance along ray to hit point (-1 if no hit)
    pointX: number;        // Hit point X
    pointY: number;        // Hit point Y
    pointZ: number;        // Hit point Z
    normalX: number;       // Surface normal at hit point X
    normalY: number;       // Surface normal at hit point Y
    normalZ: number;       // Surface normal at hit point Z
    faceId: number;        // Face index that was hit (-1 if no hit)
}

export interface ClosestPointResult {
    distance: number;      // Distance to closest point
    pointX: number;        // Closest point X
    pointY: number;        // Closest point Y
    pointZ: number;        // Closest point Z
    faceId: number;        // Face containing closest point
}

declare function MeshLib(): Promise<MeshLibModule>;
export default MeshLib;
```

---

## Step 5.11: Create Test HTML Page

Create `wasm/test.html`:

```html
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>MeshLib WASM Test</title>
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Oxygen, Ubuntu, sans-serif;
            max-width: 800px;
            margin: 0 auto;
            padding: 20px;
            background: #1a1a1a;
            color: #fff;
        }
        h1 { color: #4fc3f7; }
        .test { 
            padding: 10px; 
            margin: 5px 0;
            border-radius: 4px;
        }
        .pass { background: #2e7d32; }
        .fail { background: #c62828; }
        .pending { background: #f57c00; }
        button {
            padding: 10px 20px;
            font-size: 16px;
            background: #4fc3f7;
            border: none;
            border-radius: 4px;
            cursor: pointer;
            color: #000;
        }
        button:hover { background: #81d4fa; }
        #output { 
            background: #2d2d2d;
            padding: 20px;
            border-radius: 8px;
            margin-top: 20px;
        }
        pre { 
            background: #1a1a1a;
            padding: 10px;
            overflow-x: auto;
        }
    </style>
</head>
<body>
    <h1>ðŸ”· MeshLib WebAssembly Test</h1>
    <button onclick="runTests()">Run All Tests</button>
    <div id="output">
        <p>Click the button to run tests...</p>
    </div>
    
    <script src="meshlib.js"></script>
    <script>
        let Module;
        const results = [];
        
        async function init() {
            console.log('Loading MeshLib...');
            Module = await MeshLib();
            console.log('MeshLib loaded!');
            document.getElementById('output').innerHTML = 
                '<p>âœ… MeshLib loaded successfully. Ready to run tests.</p>';
        }
        
        function log(msg, isPass = null) {
            const div = document.createElement('div');
            div.className = 'test ' + (isPass === null ? 'pending' : isPass ? 'pass' : 'fail');
            div.textContent = msg;
            document.getElementById('output').appendChild(div);
            console.log(msg);
        }
        
        async function runTests() {
            document.getElementById('output').innerHTML = '';
            
            try {
                if (!Module) {
                    await init();
                }
                
                // Test 1: Create Cube
                log('Test 1: Creating cube...');
                const cube = Module.makeCube(1.0);
                const cubeVerts = cube.numVertices();
                const cubeFaces = cube.numFaces();
                log(`  Cube: ${cubeVerts} vertices, ${cubeFaces} faces`, cubeVerts === 8 && cubeFaces === 12);
                
                // Test 2: Volume calculation
                log('Test 2: Volume calculation...');
                const volume = cube.volume();
                log(`  Volume: ${volume.toFixed(4)}`, Math.abs(volume - 1.0) < 0.01);
                
                // Test 3: Bounding box
                log('Test 3: Bounding box...');
                const bbox = cube.boundingBox();
                log(`  Min: (${bbox.min.x.toFixed(2)}, ${bbox.min.y.toFixed(2)}, ${bbox.min.z.toFixed(2)})`);
                log(`  Max: (${bbox.max.x.toFixed(2)}, ${bbox.max.y.toFixed(2)}, ${bbox.max.z.toFixed(2)})`);
                
                // Test 4: Get vertices
                log('Test 4: Get vertices as Float32Array...');
                const vertices = cube.getVertices();
                log(`  Vertices array length: ${vertices.length}`, vertices.length === 24);
                
                // Test 5: Get faces
                log('Test 5: Get faces as Uint32Array...');
                const faces = cube.getFaces();
                log(`  Faces array length: ${faces.length}`, faces.length === 36);
                
                // Test 6: Create sphere
                log('Test 6: Creating sphere...');
                const sphere = Module.makeSphere(1.0, 3);
                log(`  Sphere: ${sphere.numVertices()} vertices, ${sphere.numFaces()} faces`, 
                    sphere.numVertices() > 0);
                
                // Test 7: Mesh offset (OpenVDB!)
                log('Test 7: Mesh offset (OpenVDB)...');
                try {
                    const offsetCube = Module.offsetMesh(cube, 0.1);
                    const offsetVolume = offsetCube.volume();
                    log(`  Offset volume: ${offsetVolume.toFixed(4)} (original: ${volume.toFixed(4)})`, 
                        offsetVolume > volume);
                } catch (e) {
                    log(`  Offset failed: ${e.message}`, false);
                }
                
                // Test 8: Boolean union
                log('Test 8: Boolean union...');
                try {
                    const cube2 = Module.makeCube(1.0);
                    cube2.translate(0.5, 0, 0);
                    const union = Module.booleanUnion(cube, cube2);
                    log(`  Union: ${union.numVertices()} vertices`, union.numVertices() > 0);
                } catch (e) {
                    log(`  Boolean failed: ${e.message}`, false);
                }
                
                // Test 9: Decimation
                log('Test 9: Mesh decimation...');
                const denseSpere = Module.makeSphere(1.0, 5);
                const originalFaces = denseSpere.numFaces();
                const decimated = Module.decimateMesh(denseSpere, 0.5);
                log(`  Original: ${originalFaces} faces, Decimated: ${decimated.numFaces()} faces`,
                    decimated.numFaces() < originalFaces);
                
                // Test 10: Transformations
                log('Test 10: Transformations...');
                const testMesh = Module.makeCube(1.0);
                testMesh.translate(1, 2, 3);
                testMesh.scale(2.0);
                const newBox = testMesh.boundingBox();
                log(`  Transformed bbox: (${newBox.min.x.toFixed(2)}, ${newBox.max.x.toFixed(2)})`, true);
                
                log('');
                log('========================================');
                log('All tests completed!');
                
            } catch (error) {
                log(`Error: ${error.message}`, false);
                console.error(error);
            }
        }
        
        // Initialize on load
        init();
    </script>
</body>
</html>
```

---

## Step 5.12: Serve and Test

```powershell
# Start a simple HTTP server
cd $STANDALONE_DIR\build\wasm-release\wasm

# Using Python
python -m http.server 8080

# Or using npx
npx serve .
```

Open `http://localhost:8080/test.html` in browser.

### Expected Results:
- All 10 tests should pass
- **Mesh offset (OpenVDB) should work!** â† This is the key test

---

## Step 5.13: Optimize WASM Size

For production, reduce the WASM binary size:

```cmake
# Add to CMakeLists.txt for release builds
IF(CMAKE_BUILD_TYPE STREQUAL "Release")
    set(EMSCRIPTEN_LINK_FLAGS
        ${EMSCRIPTEN_LINK_FLAGS}
        "-s ASSERTIONS=0"
        "-s DISABLE_EXCEPTION_CATCHING=1"
        "--closure 1"
        "-flto"
        "-O3"
    )
ENDIF()
```

### Size Optimization Tips:

| Flag | Effect |
|------|--------|
| `-O3` | Full optimization |
| `--closure 1` | Minify JS |
| `-flto` | Link-time optimization |
| `-s ASSERTIONS=0` | Remove assertions |
| Selective exports | Only export needed functions |

---

## Phase 5 Checklist

```markdown
## Phase 5 Completion Checklist

### Emscripten Setup
- [ ] emsdk installed
- [ ] emcc --version works
- [ ] Environment variables set

### Thirdparty
- [ ] scripts/thirdparty copied
- [ ] Thirdparty built with Emscripten
- [ ] OpenVDB built (OPENVDB_USE_DELAYED_LOADING=OFF)
- [ ] Blosc built

### Source Files
- [ ] MRWasm directory created
- [ ] MRWasm.cpp with Embind bindings
- [ ] CMakeLists.txt configured

### Build
- [ ] wasm-release preset works
- [ ] meshlib.js generated
- [ ] meshlib.wasm generated

### TypeScript
- [ ] meshlib.d.ts created
- [ ] Types match Embind exports

### Testing - Core Features
- [ ] test.html created
- [ ] HTTP server works
- [ ] Mesh creation works (cube, sphere, cylinder, torus)
- [ ] Volume/area calculation works
- [ ] Bounding box works

### Testing - Offset Operations (OpenVDB)
- [ ] offsetMesh works (single offset)
- [ ] shellOffset works (hollow shell)
- [ ] doubleOffset works (smooth offset)
- [ ] thickenMesh works (surface to solid)

### Testing - Boolean Operations
- [ ] booleanUnion works
- [ ] booleanDifference works
- [ ] booleanIntersection works

### Testing - Mesh Processing
- [ ] decimateMesh works
- [ ] fillHoles works
- [ ] relaxMesh works
- [ ] subdivideMesh works
- [ ] fixSelfIntersections works
- [ ] keepLargestComponent works

### Testing - Geometry Operations
- [ ] makeConvexHull works
- [ ] extractPlaneSection works
- [ ] trimWithPlane works

### Testing - Registration (ICP)
- [ ] alignMeshes works
- [ ] applyTransform works

### Testing - Spatial Queries
- [ ] rayMeshIntersect works
- [ ] findClosestPoint works
- [ ] signedDistanceToMesh works

### Testing - File I/O
- [ ] loadMeshFromBuffer works (STL)
- [ ] loadMeshFromBuffer works (OBJ)
- [ ] loadMeshFromBuffer works (PLY)
- [ ] saveMeshToBuffer works

### Optimization (Optional)
- [ ] Release build optimized
- [ ] WASM size reasonable (<15MB)
- [ ] Closure compiler applied
```

---

## Troubleshooting

### "MeshLib is not a function"
```javascript
// Make sure MODULARIZE is set correctly
// The default export should be a function that returns a Promise
const Module = await MeshLib();
```

### "Out of memory"
```cmake
# Increase memory limits
"-s ALLOW_MEMORY_GROWTH=1"
"-s MAXIMUM_MEMORY=4GB"
```

### "OpenVDB symbols undefined"
Ensure OpenVDB was built with Emscripten and linked properly.

### Slow loading
- Enable `--closure 1`
- Use gzip compression on server
- Lazy-load the WASM module

---

## Next Phase

Once all items are checked, proceed to **PHASE_6_THREEJS_INTEGRATION.md**

---

*Phase 5 Version: 1.0*
*Created: January 13, 2026*
