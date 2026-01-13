# Phase 2: MRMesh Core Library

## Goal
Copy and configure the MRMesh core library to compile standalone with FULL functionality.

## Duration: 2 weeks

## Prerequisites
- Phase 1 completed
- vcpkg installed and configured
- Dependencies installed (Boost, Eigen, TBB, fmt, spdlog, etc.)

---

## Critical Philosophy

**MRMesh is the heart of MeshLib.** It contains:
- All data structures (Mesh, Topology, PointCloud, Polyline)
- All core algorithms (Boolean, Decimation, Hole filling, ICP)
- All spatial indexing (AABB trees)
- All basic I/O (STL, OBJ, PLY, OFF)

**We must copy ALL files.** There are no "optional" files in MRMesh.

---

## Step 2.1: Understand MRMesh Structure

### File Categories (~500+ files)

| Category | Files | Description |
|----------|-------|-------------|
| **Core Data** | MRMesh.*, MRMeshTopology.*, MRPointCloud.*, MRPolyline.* | Primary data structures |
| **Math** | MRVector*.*, MRMatrix*.*, MRAffineXf*.*, MRQuaternion.* | Linear algebra |
| **Algorithms** | MRMeshBoolean.*, MRMeshDecimate.*, MRMeshFillHole.*, MRICP.* | Operations |
| **Spatial** | MRAABBTree*.*, MRDenseBox.*, MRBox.* | Acceleration structures |
| **I/O** | MRMeshLoad.*, MRMeshSave.*, MRPly.*, MRPointsLoad.* | File formats |
| **Distance** | MRMeshDistance.*, MRFastWindingNumber.*, MRSurfaceDistance.* | Distance queries |
| **Utility** | MRBitSet.*, MRBuffer.*, MRExpected.*, MRTimer.* | Support classes |
| **Build** | CMakeLists.txt, config.h, config_cmake.h.in | Build configuration |

---

## Step 2.2: Install Dependencies First

### Using vcpkg (Windows)

```powershell
# Navigate to standalone directory
$STANDALONE_DIR = "C:\Users\VijayRaghavVarada\Documents\Github\meshlib-standalone"
Set-Location $STANDALONE_DIR

# Install dependencies
vcpkg install --triplet x64-windows

# This installs from vcpkg.json:
# - boost-headers, boost-multiprecision
# - eigen3
# - fmt
# - spdlog
# - onetbb (TBB)
# - jsoncpp
# - libzip
# - tl-expected
# - gtest
# - pybind11
```

### Using MeshLib's Prebuilt (Alternative)

If you want to use MeshLib's prebuilt thirdparty:

```powershell
# Point to MeshLib's thirdparty build
$env:MESHLIB_THIRDPARTY_ROOT_DIR = "C:\Users\VijayRaghavVarada\Documents\Github\MeshLib"
```

---

## Step 2.3: Copy MRPch (Precompiled Headers)

```powershell
$MESHLIB_DIR = "C:\Users\VijayRaghavVarada\Documents\Github\MeshLib"
$STANDALONE_DIR = "C:\Users\VijayRaghavVarada\Documents\Github\meshlib-standalone"

# Copy MRPch
$SOURCE = "$MESHLIB_DIR\source\MRPch"
$TARGET = "$STANDALONE_DIR\source\MRPch"

# Copy all files
Get-ChildItem $SOURCE -File | ForEach-Object {
    Copy-Item $_.FullName $TARGET -Force
    Write-Host "Copied: $($_.Name)"
}
```

### Files Expected:
- MRPch.h
- MRPch.cpp
- CMakeLists.txt

### Verify MRPch CMakeLists.txt

The copied CMakeLists.txt should work. If it references paths outside MRPch, they need adjustment.

---

## Step 2.4: Copy ALL MRMesh Source Files

**IMPORTANT:** Copy ALL files, not a subset.

```powershell
$SOURCE = "$MESHLIB_DIR\source\MRMesh"
$TARGET = "$STANDALONE_DIR\source\MRMesh"

# Copy all source files
Get-ChildItem $SOURCE -File -Include "*.cpp","*.h","*.hpp" -Recurse | ForEach-Object {
    Copy-Item $_.FullName $TARGET -Force
}

# Copy build files
Copy-Item "$SOURCE\CMakeLists.txt" $TARGET -Force
Copy-Item "$SOURCE\config.h" $TARGET -Force
Copy-Item "$SOURCE\config_cmake.h.in" $TARGET -Force

# Copy any .cmake files
Get-ChildItem $SOURCE -File -Include "*.cmake","*.cmake.in" | ForEach-Object {
    Copy-Item $_.FullName $TARGET -Force
}

# Count files
$fileCount = (Get-ChildItem $TARGET -File).Count
Write-Host "Total files copied: $fileCount"
```

### Expected File Count: ~500-550 files

---

## Step 2.5: Copy Required Thirdparty Headers

```powershell
# parallel-hashmap (header-only hash maps used throughout MRMesh)
$SOURCE_PH = "$MESHLIB_DIR\thirdparty\parallel-hashmap"
$TARGET_PH = "$STANDALONE_DIR\thirdparty\parallel-hashmap"

if (Test-Path $SOURCE_PH) {
    Copy-Item $SOURCE_PH $TARGET_PH -Recurse -Force
    Write-Host "Copied parallel-hashmap"
}

# expected (tl::expected - error handling)
$SOURCE_EX = "$MESHLIB_DIR\thirdparty\expected"
$TARGET_EX = "$STANDALONE_DIR\thirdparty\expected"

if (Test-Path $SOURCE_EX) {
    Copy-Item $SOURCE_EX $TARGET_EX -Recurse -Force
    Write-Host "Copied expected"
}
```

---

## Step 2.6: Identify and Remove Viewer Dependencies

Some MRMesh files may have optional includes for MRViewer. We need to find and handle these.

### Find files with viewer includes:

```powershell
$TARGET = "$STANDALONE_DIR\source\MRMesh"

# Search for MRViewer includes
Get-ChildItem $TARGET -File -Include "*.cpp","*.h" -Recurse | 
    Select-String -Pattern '#include.*MRViewer' | 
    Select-Object Path, LineNumber, Line | 
    Format-Table -AutoSize
```

### Expected Results:
Most files should NOT have MRViewer includes. If some do, they need conditional compilation.

### Common Files That Might Need Fixes:

| File | Potential Issue | Fix |
|------|-----------------|-----|
| `MRVisualObject.cpp` | Rendering callbacks | Usually OK (abstract) |
| `MRSceneRoot.cpp` | Scene management | May need `#ifndef MESHLIB_NO_VIEWER` |
| `MRObjectMesh.cpp` | Visual properties | Usually OK |

---

## Step 2.7: Modify config.h for Standalone

Edit `$STANDALONE_DIR\source\MRMesh\config.h`:

Add at the top:
```cpp
#pragma once

// ============================================================================
// MeshLib Standalone Configuration
// ============================================================================

// This is a standalone build without GUI components
#ifndef MESHLIB_STANDALONE
#define MESHLIB_STANDALONE 1
#endif

// Disable viewer-dependent code paths
#ifndef MESHLIB_NO_VIEWER
#define MESHLIB_NO_VIEWER 1
#endif

// Continue with original config.h content...
```

---

## Step 2.8: Update MRMesh CMakeLists.txt

The copied CMakeLists.txt should mostly work, but verify these key sections:

### Required find_package calls:
```cmake
# Emscripten's Boost package doesn't contain CMake config files
IF(EMSCRIPTEN)
  IF(POLICY CMP0167)
    cmake_policy(SET CMP0167 OLD)
  ENDIF()
  find_package(Boost 1.73 REQUIRED)
ELSE()
  find_package(Boost 1.73 CONFIG COMPONENTS headers REQUIRED)
ENDIF()

find_package(Eigen3 REQUIRED)
find_package(fmt 8 REQUIRED)
find_package(JsonCpp REQUIRED)
find_package(spdlog 1.9 REQUIRED)
find_package(TBB 2020.1 COMPONENTS tbb REQUIRED)
find_package(tl-expected REQUIRED)
```

### Required target_link_libraries:
```cmake
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Boost::headers
    Eigen3::Eigen
    fmt::fmt
    JsonCpp::JsonCpp
    spdlog::spdlog
    TBB::tbb
    tl::expected
  PRIVATE
    ${CMAKE_DL_LIBS}
)
```

### Required include for parallel-hashmap:
```cmake
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${MESHLIB_THIRDPARTY_DIR}/parallel-hashmap>
)
```

---

## Step 2.9: First Build Attempt

```powershell
Set-Location $STANDALONE_DIR

# Configure
cmake --preset native-debug

# Build
cmake --build --preset native-debug 2>&1 | Tee-Object -FilePath build_log.txt

# Check for errors
Select-String -Path build_log.txt -Pattern "error:" | Select-Object -First 20
```

### Common Build Errors and Fixes

| Error | Cause | Fix |
|-------|-------|-----|
| `MRViewer.h not found` | Viewer include | Add `#ifdef MESHLIB_NO_VIEWER` guard |
| `parallel_hashmap/phmap.h not found` | Missing thirdparty | Verify parallel-hashmap copied |
| `tl::expected not found` | Missing dep | Check vcpkg install |
| `MRMESH_API undefined` | Missing export macro | Check config.h copied |
| `MR_PROJECT_NAME undefined` | Missing define | Add in root CMakeLists.txt |

---

## Step 2.10: Fix Compilation Errors Iteratively

### Process:
1. Build
2. Find first error
3. Fix it
4. Rebuild
5. Repeat until clean

### Example Fix for Viewer Include:

If you find a file like `MRSomeFile.cpp` with:
```cpp
#include <MRViewer/MRSomeViewerHeader.h>
```

Wrap it:
```cpp
#ifndef MESHLIB_NO_VIEWER
#include <MRViewer/MRSomeViewerHeader.h>
#endif
```

And guard any code using it:
```cpp
#ifndef MESHLIB_NO_VIEWER
// Code that uses viewer
#endif
```

---

## Step 2.11: Verify Build Output

After successful build:

```powershell
# Check output files
Get-ChildItem "$STANDALONE_DIR\build\native-debug\bin" -File

# Expected:
# - MRMesh.dll (Windows) or libMRMesh.so (Linux) or libMRMesh.dylib (macOS)
# - MRPch.dll/.so/.dylib (if separate)
```

### File Size Check:
- MRMesh.dll should be several MB (5-20 MB depending on build type)
- If it's too small (<1 MB), something is wrong

---

## Step 2.12: Create Basic Test

Create `$STANDALONE_DIR\tests\cpp\test_mrmesh_basic.cpp`:

```cpp
#include <MRMesh/MRMesh.h>
#include <MRMesh/MRCube.h>
#include <MRMesh/MRMeshBoolean.h>
#include <MRMesh/MRMeshDecimate.h>
#include <MRMesh/MRMeshFillHole.h>
#include <MRMesh/MRICP.h>
#include <MRMesh/MRMeshLoad.h>
#include <MRMesh/MRMeshSave.h>
#include <MRMesh/MRPointCloud.h>
#include <MRMesh/MRAABBTree.h>
#include <iostream>
#include <cassert>

void testMeshCreation() {
    std::cout << "Test: Mesh Creation... ";
    
    // Create cube mesh
    MR::Mesh cube = MR::makeCube();
    
    assert(cube.topology.numValidVerts() == 8);
    assert(cube.topology.numValidFaces() == 12);  // 6 faces * 2 triangles
    
    std::cout << "PASSED" << std::endl;
}

void testBoundingBox() {
    std::cout << "Test: Bounding Box... ";
    
    MR::Mesh cube = MR::makeCube();
    auto box = cube.computeBoundingBox();
    
    assert(box.valid());
    assert(box.diagonal() > 0);
    
    std::cout << "PASSED (diagonal=" << box.diagonal() << ")" << std::endl;
}

void testBoolean() {
    std::cout << "Test: Boolean Union... ";
    
    MR::Mesh cube1 = MR::makeCube();
    MR::Mesh cube2 = MR::makeCube();
    
    // Offset cube2
    for (auto& p : cube2.points) {
        p.x += 0.5f;
    }
    
    auto result = MR::boolean(cube1, cube2, MR::BooleanOperation::Union);
    
    assert(result.valid());
    assert(result.mesh.topology.numValidVerts() > 0);
    
    std::cout << "PASSED (verts=" << result.mesh.topology.numValidVerts() << ")" << std::endl;
}

void testDecimate() {
    std::cout << "Test: Decimation... ";
    
    MR::Mesh mesh = MR::makeCube();
    
    // Subdivide to get more faces
    // (In real test, load a high-poly mesh)
    
    MR::DecimateSettings settings;
    settings.maxError = 0.01f;
    
    auto result = MR::decimateMesh(mesh, settings);
    
    // Should not fail
    std::cout << "PASSED" << std::endl;
}

void testFileIO() {
    std::cout << "Test: File I/O (STL)... ";
    
    MR::Mesh cube = MR::makeCube();
    
    // Save
    auto saveResult = MR::MeshSave::toAnySupportedFormat(cube, "test_cube.stl");
    assert(saveResult.has_value() || saveResult.error().empty() == false);
    
    // Load
    auto loadResult = MR::MeshLoad::fromAnySupportedFormat("test_cube.stl");
    if (loadResult.has_value()) {
        assert(loadResult->topology.numValidVerts() == 8);
        std::cout << "PASSED" << std::endl;
    } else {
        std::cout << "SKIPPED (file I/O)" << std::endl;
    }
    
    // Cleanup
    std::remove("test_cube.stl");
}

void testAABBTree() {
    std::cout << "Test: AABB Tree... ";
    
    MR::Mesh cube = MR::makeCube();
    
    // Build AABB tree
    MR::AABBTree tree(cube);
    
    assert(tree.nodes().size() > 0);
    
    std::cout << "PASSED (nodes=" << tree.nodes().size() << ")" << std::endl;
}

void testPointCloud() {
    std::cout << "Test: Point Cloud... ";
    
    MR::PointCloud pc;
    pc.points.push_back(MR::Vector3f(0, 0, 0));
    pc.points.push_back(MR::Vector3f(1, 0, 0));
    pc.points.push_back(MR::Vector3f(0, 1, 0));
    
    assert(pc.points.size() == 3);
    
    auto box = pc.computeBoundingBox();
    assert(box.valid());
    
    std::cout << "PASSED" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "MRMesh Basic Tests" << std::endl;
    std::cout << "========================================" << std::endl;
    
    try {
        testMeshCreation();
        testBoundingBox();
        testBoolean();
        testDecimate();
        testFileIO();
        testAABBTree();
        testPointCloud();
        
        std::cout << "========================================" << std::endl;
        std::cout << "ALL TESTS PASSED!" << std::endl;
        std::cout << "========================================" << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "TEST FAILED: " << e.what() << std::endl;
        return 1;
    }
}
```

### Update tests/CMakeLists.txt:

```cmake
project(MeshLibTests CXX)

# Basic MRMesh test
add_executable(test_mrmesh_basic cpp/test_mrmesh_basic.cpp)
target_link_libraries(test_mrmesh_basic PRIVATE MRMesh)

# Register with CTest
add_test(NAME MRMeshBasic COMMAND test_mrmesh_basic)
```

---

## Step 2.13: Run Tests

```powershell
# Build tests
cmake --build --preset native-debug --target test_mrmesh_basic

# Run
.\build\native-debug\bin\test_mrmesh_basic.exe
```

### Expected Output:
```
========================================
MRMesh Basic Tests
========================================
Test: Mesh Creation... PASSED
Test: Bounding Box... PASSED (diagonal=1.73205)
Test: Boolean Union... PASSED (verts=14)
Test: Decimation... PASSED
Test: File I/O (STL)... PASSED
Test: AABB Tree... PASSED (nodes=23)
Test: Point Cloud... PASSED
========================================
ALL TESTS PASSED!
========================================
```

---

## Step 2.14: Copy and Verify MRIOExtras (Optional in Phase 2)

If you want extended I/O formats now:

```powershell
$SOURCE = "$MESHLIB_DIR\source\MRIOExtras"
$TARGET = "$STANDALONE_DIR\source\MRIOExtras"

# Copy all files
Get-ChildItem $SOURCE -File | ForEach-Object {
    Copy-Item $_.FullName $TARGET -Force
}
```

**Note:** MRIOExtras has many optional dependencies (GLTF, E57, LAS, STEP, etc.). 
Configure them selectively in the CMakeLists.txt based on your needs.

---

## Phase 2 Checklist

```markdown
## Phase 2 Completion Checklist

### Source Files
- [ ] MRPch copied and builds
- [ ] MRMesh ALL ~500 files copied
- [ ] parallel-hashmap thirdparty copied
- [ ] expected thirdparty copied
- [ ] config.h modified for standalone

### Dependencies
- [ ] vcpkg install completed successfully
- [ ] All find_package() calls succeed
- [ ] All target_link_libraries() resolve

### Build
- [ ] cmake --preset native-debug succeeds
- [ ] cmake --build completes without errors
- [ ] MRMesh.dll/so/dylib generated
- [ ] File size is reasonable (>1 MB)

### Tests
- [ ] test_mrmesh_basic compiles
- [ ] test_mrmesh_basic runs
- [ ] Mesh creation works
- [ ] Boolean operations work
- [ ] Decimation works
- [ ] File I/O works
- [ ] AABB tree works
- [ ] Point cloud works

### Verification Against Main MeshLib
- [ ] Create same mesh in both → identical vertex count
- [ ] Boolean same meshes in both → identical results
- [ ] Save/load cycle preserves data

### Documentation
- [ ] Build errors documented and fixed
- [ ] Any workarounds documented
```

---

## Common Issues and Solutions

### Issue: "MRViewer/MRxxx.h not found"
**Solution:** 
```cpp
#ifndef MESHLIB_NO_VIEWER
#include <MRViewer/MRxxx.h>
#endif
```

### Issue: "parallel_hashmap/phmap.h not found"
**Solution:** Verify thirdparty/parallel-hashmap was copied correctly.

### Issue: Link errors for TBB
**Solution:** Ensure TBB is installed via vcpkg and found by CMake.

### Issue: "MRMESH_API" or "MRMESH_CLASS" undefined
**Solution:** Check that config.h is being included and has the export macros defined.

### Issue: Many undefined symbols
**Solution:** You may have missed copying some .cpp files. Ensure ALL files were copied.

---

## Next Phase

Once all items are checked, proceed to **PHASE_3_MRVOXELS_OPENVDB.md**

---

*Phase 2 Version: 1.0*
*Created: January 13, 2026*