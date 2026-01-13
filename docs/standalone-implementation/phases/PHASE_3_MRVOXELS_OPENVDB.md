# Phase 3: MRVoxels with OpenVDB

## Goal
Add full voxel operations including mesh offset, marching cubes, and volume booleans using OpenVDB.

## Duration: 1.5 weeks

## Prerequisites
- Phase 2 completed (MRMesh building and tested)
- OpenVDB installed (via vcpkg or build from source)

---

## Critical Features in MRVoxels

| Feature | Function | Importance |
|---------|----------|------------|
| **Mesh Offset** | `generalOffsetMesh()` | ⭐⭐⭐ Critical |
| **Marching Cubes** | `marchingCubes()` | ⭐⭐⭐ Critical |
| **Mesh → SDF** | `meshToLevelSet()` | ⭐⭐⭐ Critical |
| **Volume Boolean** | `csgUnion/Difference/Intersection` | ⭐⭐ Important |
| **DICOM Loading** | `loadDicomFile()` | ⭐⭐ Important |
| **Grid Smoothing** | OpenVDB filters | ⭐ Nice to have |

**ALL of these work in WASM** because MeshLib already compiles OpenVDB for Emscripten!

---

## Step 3.1: Install OpenVDB Dependencies

### Using vcpkg (Windows)

```powershell
# OpenVDB has many dependencies, vcpkg handles them
vcpkg install openvdb:x64-windows
vcpkg install blosc:x64-windows

# Verify installation
vcpkg list | Select-String "openvdb"
```

### Using MeshLib's Build Scripts (Alternative)

```bash
# From Git Bash or WSL
cd /path/to/meshlib-standalone

# Copy and run the OpenVDB build script
cp /path/to/MeshLib/scripts/thirdparty/openvdb.sh scripts/thirdparty/
./scripts/thirdparty/openvdb.sh
```

---

## Step 3.2: Copy MRVoxels Source Files

```powershell
$MESHLIB_DIR = "C:\Users\VijayRaghavVarada\Documents\Github\MeshLib"
$STANDALONE_DIR = "C:\Users\VijayRaghavVarada\Documents\Github\meshlib-standalone"

$SOURCE = "$MESHLIB_DIR\source\MRVoxels"
$TARGET = "$STANDALONE_DIR\source\MRVoxels"

# Copy all source files
Get-ChildItem $SOURCE -File | ForEach-Object {
    Copy-Item $_.FullName $TARGET -Force
    Write-Host "Copied: $($_.Name)"
}

# Count files
$fileCount = (Get-ChildItem $TARGET -File).Count
Write-Host "Total files copied: $fileCount"
```

### Expected Files (~60):

| File | Purpose |
|------|---------|
| `MROffset.h/cpp` | **Main offset operations** |
| `MRVDBConversions.h/cpp` | Mesh ↔ VDB conversions |
| `MRMarchingCubes.h/cpp` | Marching cubes algorithm |
| `MRFloatGrid.h/cpp` | OpenVDB grid wrapper |
| `MRDicom.h/cpp` | DICOM medical imaging |
| `MRBoolean.h/cpp` | Volume boolean operations |
| `MRVoxelsVolume.h/cpp` | Volume data structures |
| `MROpenVDB.h` | OpenVDB includes |
| `CMakeLists.txt` | Build configuration |
| `config.h` | Feature flags |

---

## Step 3.3: Copy OpenVDB Thirdparty (if building from source)

If using MeshLib's OpenVDB build:

```powershell
# Copy OpenVDB source
$SOURCE_VDB = "$MESHLIB_DIR\thirdparty\openvdb"
$TARGET_VDB = "$STANDALONE_DIR\thirdparty\openvdb"

Copy-Item $SOURCE_VDB $TARGET_VDB -Recurse -Force

# Copy Blosc (required for OpenVDB compression)
$SOURCE_BLOSC = "$MESHLIB_DIR\thirdparty\c-blosc"
$TARGET_BLOSC = "$STANDALONE_DIR\thirdparty\c-blosc"

Copy-Item $SOURCE_BLOSC $TARGET_BLOSC -Recurse -Force

# Copy TBB (if not using vcpkg's TBB)
$SOURCE_TBB = "$MESHLIB_DIR\thirdparty\onetbb"
$TARGET_TBB = "$STANDALONE_DIR\thirdparty\onetbb"

Copy-Item $SOURCE_TBB $TARGET_TBB -Recurse -Force
```

---

## Step 3.4: Copy OpenVDB Build Script

```powershell
$SOURCE_SCRIPT = "$MESHLIB_DIR\scripts\thirdparty\openvdb.sh"
$TARGET_SCRIPT = "$STANDALONE_DIR\scripts\thirdparty\openvdb.sh"

Copy-Item $SOURCE_SCRIPT $TARGET_SCRIPT -Force
```

### Key OpenVDB Build Flags (for WASM compatibility):

From `scripts/thirdparty/openvdb.sh`:
```bash
CMAKE_OPTIONS="${CMAKE_OPTIONS} \
  -D OPENVDB_ENABLE_UNINSTALL=OFF \
  -D OPENVDB_ENABLE_INSTALL=OFF \
  -D OPENVDB_CORE_SHARED=ON \
  -D OPENVDB_CORE_STATIC=OFF \
  -D OPENVDB_BUILD_BINARIES=OFF \
  -D OPENVDB_BUILD_VDB_PRINT=OFF \
  -D OPENVDB_USE_DELAYED_LOADING=OFF \  # CRITICAL for WASM!
  -D USE_EXPLICIT_INSTANTIATION=OFF \   # Reduces binary size
"
```

---

## Step 3.5: Verify MRVoxels CMakeLists.txt

Check `$STANDALONE_DIR\source\MRVoxels\CMakeLists.txt`:

### Required Sections:

```cmake
project(MRVoxels CXX)

# Options
option(MRVOXELS_NO_DICOM "Disable DICOM format support" OFF)
option(MRVOXELS_NO_TIFF "Disable TIFF format support" OFF)

IF(MR_EMSCRIPTEN)
    set(MRVOXELS_NO_TIFF ON)  # TIFF not available in Emscripten
ENDIF()

# Source files
file(GLOB SOURCES "*.cpp")
file(GLOB HEADERS "*.h")

add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS})

# Configure file (optional)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/config_cmake.h.in 
               ${CMAKE_CURRENT_SOURCE_DIR}/config_cmake.h)

# Find OpenVDB - REQUIRED, not optional!
find_package(OpenVDB 10 REQUIRED)

# Link libraries
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        MRMesh              # Depends on MRMesh
        OpenVDB::openvdb    # Full OpenVDB!
)

# Blosc for Emscripten static linking
IF(EMSCRIPTEN)
    target_link_libraries(${PROJECT_NAME} PRIVATE blosc)
ENDIF()

# Optional: DICOM support
IF(NOT MRVOXELS_NO_DICOM)
    find_package(GDCM CONFIG REQUIRED)
    target_link_libraries(${PROJECT_NAME} PRIVATE 
        gdcmIOD gdcmDICT gdcmDSED gdcmMEXD gdcmMSFF)
ENDIF()

# Installation
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}
    LIBRARY DESTINATION "${MR_MAIN_LIB_DIR}"
    ARCHIVE DESTINATION "${MR_MAIN_LIB_DIR}"
    RUNTIME DESTINATION "${MR_BIN_DIR}"
)
```

---

## Step 3.6: Build MRVoxels

```powershell
Set-Location $STANDALONE_DIR

# Configure (should find OpenVDB)
cmake --preset native-debug

# Look for OpenVDB detection
# Expected: "-- Found OpenVDB: ..."

# Build
cmake --build --preset native-debug 2>&1 | Tee-Object -FilePath build_voxels_log.txt

# Check for errors
Select-String -Path build_voxels_log.txt -Pattern "error:" | Select-Object -First 10
```

### Common Build Errors:

| Error | Cause | Fix |
|-------|-------|-----|
| `OpenVDB not found` | Not installed | Install via vcpkg |
| `openvdb/openvdb.h not found` | Include path | Check OpenVDB_DIR |
| `undefined reference to blosc_` | Missing blosc | Link blosc library |
| `TBB version mismatch` | TBB conflict | Use consistent TBB version |

---

## Step 3.7: Create MRVoxels Test

Create `$STANDALONE_DIR\tests\cpp\test_mrvoxels.cpp`:

```cpp
#include <MRMesh/MRMesh.h>
#include <MRMesh/MRCube.h>
#include <MRMesh/MRMeshSave.h>
#include <MRMesh/MRRegionBoundary.h>
#include <MRVoxels/MROffset.h>
#include <MRVoxels/MRMarchingCubes.h>
#include <MRVoxels/MRVDBConversions.h>
#include <MRVoxels/MRFloatGrid.h>
#include <iostream>
#include <cassert>
#include <cmath>

void testMeshOffset() {
    std::cout << "Test: Mesh Offset (OpenVDB)... ";
    
    // Create a cube mesh
    MR::Mesh mesh = MR::makeCube();
    
    // Get original bounding box
    auto originalBox = mesh.computeBoundingBox();
    float originalDiagonal = originalBox.diagonal();
    
    // Configure offset parameters
    MR::GeneralOffsetParameters params;
    params.voxelSize = originalDiagonal * 0.02f;  // 2% of diagonal
    
    // Detect if mesh has holes
    if (!MR::findRightBoundary(mesh.topology).empty()) {
        params.signDetectionMode = MR::SignDetectionMode::HoleWindingRule;
    }
    
    // Perform positive offset (expand mesh)
    float offset = originalDiagonal * 0.1f;  // 10% expansion
    auto result = MR::generalOffsetMesh(mesh, offset, params);
    
    assert(result.has_value());
    
    // Verify the result is larger
    auto resultBox = result->computeBoundingBox();
    float resultDiagonal = resultBox.diagonal();
    
    assert(resultDiagonal > originalDiagonal);
    
    std::cout << "PASSED (original=" << originalDiagonal 
              << ", result=" << resultDiagonal << ")" << std::endl;
    
    // Save for visual inspection
    MR::MeshSave::toAnySupportedFormat(*result, "test_offset_cube.stl");
}

void testNegativeOffset() {
    std::cout << "Test: Negative Offset (Shrink)... ";
    
    MR::Mesh mesh = MR::makeCube();
    auto originalBox = mesh.computeBoundingBox();
    float originalDiagonal = originalBox.diagonal();
    
    MR::GeneralOffsetParameters params;
    params.voxelSize = originalDiagonal * 0.02f;
    
    // Negative offset (shrink mesh)
    float offset = -originalDiagonal * 0.05f;
    auto result = MR::generalOffsetMesh(mesh, offset, params);
    
    assert(result.has_value());
    
    auto resultBox = result->computeBoundingBox();
    assert(resultBox.diagonal() < originalDiagonal);
    
    std::cout << "PASSED" << std::endl;
}

void testMeshToLevelSet() {
    std::cout << "Test: Mesh to Level Set... ";
    
    MR::Mesh mesh = MR::makeCube();
    auto box = mesh.computeBoundingBox();
    
    // Convert mesh to signed distance field
    float voxelSize = box.diagonal() * 0.05f;
    auto grid = MR::meshToLevelSet(mesh, MR::AffineXf3f(), 
                                   MR::Vector3f::diagonal(voxelSize));
    
    assert(grid != nullptr);
    
    std::cout << "PASSED" << std::endl;
}

void testMarchingCubes() {
    std::cout << "Test: Marching Cubes... ";
    
    MR::Mesh mesh = MR::makeCube();
    auto box = mesh.computeBoundingBox();
    float voxelSize = box.diagonal() * 0.05f;
    
    // Convert to grid
    auto grid = MR::meshToLevelSet(mesh, MR::AffineXf3f(), 
                                   MR::Vector3f::diagonal(voxelSize));
    
    // Convert back using marching cubes
    MR::VdbVolume volume = MR::floatGridToVdbVolume(std::move(grid));
    
    MR::VolumeToMeshParams vmParams;
    vmParams.iso = 0.0f;
    
    auto result = MR::marchingCubes(volume, vmParams);
    
    assert(result.has_value());
    assert(result->topology.numValidVerts() > 0);
    
    std::cout << "PASSED (verts=" << result->topology.numValidVerts() << ")" << std::endl;
}

void testShellOffset() {
    std::cout << "Test: Shell Offset (Thickening)... ";
    
    MR::Mesh mesh = MR::makeCube();
    
    MR::ShellOffsetParameters params;
    params.voxelSize = 0.05f;
    
    float offset = 0.1f;
    auto result = MR::shellOffset(mesh, offset, params);
    
    // Shell offset should create a hollow shell
    assert(result.has_value());
    
    std::cout << "PASSED" << std::endl;
}

void testDoubleOffset() {
    std::cout << "Test: Double Offset (Smooth)... ";
    
    MR::Mesh mesh = MR::makeCube();
    
    MR::DoubleOffsetParameters params;
    params.voxelSize = 0.05f;
    
    // Double offset: first expand, then shrink - smooths the mesh
    auto result = MR::doubleOffset(mesh, 0.1f, -0.1f, params);
    
    assert(result.has_value());
    
    std::cout << "PASSED" << std::endl;
}

void testFloatGridOperations() {
    std::cout << "Test: Float Grid Operations... ";
    
    MR::Mesh cube1 = MR::makeCube();
    MR::Mesh cube2 = MR::makeCube();
    
    // Offset cube2
    for (auto& p : cube2.points) {
        p.x += 0.5f;
    }
    
    float voxelSize = 0.05f;
    
    // Convert both to grids
    auto grid1 = MR::meshToLevelSet(cube1, MR::AffineXf3f(), 
                                    MR::Vector3f::diagonal(voxelSize));
    auto grid2 = MR::meshToLevelSet(cube2, MR::AffineXf3f(), 
                                    MR::Vector3f::diagonal(voxelSize));
    
    assert(grid1 != nullptr);
    assert(grid2 != nullptr);
    
    std::cout << "PASSED" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "MRVoxels Tests (OpenVDB)" << std::endl;
    std::cout << "========================================" << std::endl;
    
    try {
        testMeshOffset();
        testNegativeOffset();
        testMeshToLevelSet();
        testMarchingCubes();
        testShellOffset();
        testDoubleOffset();
        testFloatGridOperations();
        
        std::cout << "========================================" << std::endl;
        std::cout << "ALL VOXEL TESTS PASSED!" << std::endl;
        std::cout << "========================================" << std::endl;
        
        // Cleanup test files
        std::remove("test_offset_cube.stl");
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "TEST FAILED: " << e.what() << std::endl;
        return 1;
    }
}
```

### Update tests/CMakeLists.txt:

```cmake
# Add after test_mrmesh_basic

# MRVoxels test
IF(MESHLIB_BUILD_VOXELS)
    add_executable(test_mrvoxels cpp/test_mrvoxels.cpp)
    target_link_libraries(test_mrvoxels PRIVATE MRMesh MRVoxels)
    add_test(NAME MRVoxels COMMAND test_mrvoxels)
ENDIF()
```

---

## Step 3.8: Run Voxel Tests

```powershell
# Build tests
cmake --build --preset native-debug --target test_mrvoxels

# Run
.\build\native-debug\bin\test_mrvoxels.exe
```

### Expected Output:
```
========================================
MRVoxels Tests (OpenVDB)
========================================
Test: Mesh Offset (OpenVDB)... PASSED (original=1.73205, result=1.90526)
Test: Negative Offset (Shrink)... PASSED
Test: Mesh to Level Set... PASSED
Test: Marching Cubes... PASSED (verts=386)
Test: Shell Offset (Thickening)... PASSED
Test: Double Offset (Smooth)... PASSED
Test: Float Grid Operations... PASSED
========================================
ALL VOXEL TESTS PASSED!
========================================
```

---

## Step 3.9: Compare with Main MeshLib

Critical verification: ensure offset produces identical results.

### Test Script (PowerShell):

```powershell
# Create test mesh in both libraries and compare results

# This requires both MeshLib and meshlib-standalone to be built
# Run same offset operation in both and compare output files

# In MeshLib:
# ./MeshLib/build/bin/meshconv offset input.stl output_main.stl --offset 0.1

# In Standalone:
# ./meshlib-standalone/build/bin/test_mrvoxels

# Compare:
# diff output_main.stl output_standalone.stl
```

---

## Step 3.10: Optional - DICOM Support

If you need medical imaging support:

### Install GDCM:
```powershell
vcpkg install gdcm:x64-windows
```

### Verify DICOM test:

```cpp
#include <MRVoxels/MRDicom.h>

void testDicomLoad() {
    // Load a DICOM file (you'll need a test file)
    auto result = MR::loadDicomFile("test.dcm");
    if (result.has_value()) {
        std::cout << "DICOM loaded: " 
                  << result->vol.dims.x << "x" 
                  << result->vol.dims.y << "x" 
                  << result->vol.dims.z << std::endl;
    }
}
```

---

## Phase 3 Checklist

```markdown
## Phase 3 Completion Checklist

### Source Files
- [ ] MRVoxels all ~60 files copied
- [ ] OpenVDB thirdparty available (vcpkg or source)
- [ ] Blosc available
- [ ] Build scripts copied (if building from source)

### Dependencies
- [ ] OpenVDB 10 found by CMake
- [ ] Blosc linked (especially for Emscripten)
- [ ] GDCM found (if DICOM enabled)

### Build
- [ ] cmake --preset native-debug succeeds
- [ ] MRVoxels.dll/so/dylib generated
- [ ] Links against MRMesh
- [ ] Links against OpenVDB

### Tests
- [ ] test_mrvoxels compiles
- [ ] test_mrvoxels runs
- [ ] generalOffsetMesh works
- [ ] Negative offset works
- [ ] meshToLevelSet works
- [ ] marchingCubes works
- [ ] shellOffset works
- [ ] doubleOffset works

### Verification
- [ ] Offset results match main MeshLib
- [ ] Marching cubes produces valid meshes
- [ ] No memory leaks (run with AddressSanitizer)

### DICOM (Optional)
- [ ] GDCM installed
- [ ] loadDicomFile works
- [ ] loadDicomFolder works
```

---

## Troubleshooting

### OpenVDB Not Found
```cmake
# Add hint for OpenVDB location
set(OpenVDB_DIR "/path/to/openvdb/lib/cmake/OpenVDB")
```

### Blosc Link Errors (Emscripten)
```cmake
IF(EMSCRIPTEN)
    target_link_libraries(${PROJECT_NAME} PRIVATE blosc)
ENDIF()
```

### TBB Version Mismatch
Ensure both MRMesh and MRVoxels use the same TBB version.

### Grid Creation Fails
Check that OpenVDB was compiled with the same flags as MRVoxels.

---

## Next Phase

Once all items are checked, proceed to **PHASE_4_PYTHON_BINDINGS.md**

---

*Phase 3 Version: 1.0*
*Created: January 13, 2026*