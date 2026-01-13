# Phase 4: Python Bindings

## Goal
Create Python bindings using pybind11 to expose MRMesh and MRVoxels functionality.

## Duration: 1.5 weeks

## Prerequisites
- Phase 2 (MRMesh) completed
- Phase 3 (MRVoxels) completed
- Python 3.9+ installed
- pybind11 available

---

## Understanding MeshLib's Python Binding Architecture

MeshLib uses a custom binding generator called **mrbind** (modified pybind11):

```
scripts/mrbind/         → Custom binding generator
scripts/mrbind-pybind11/ → Modified pybind11 fork
source/MRPython/        → Python module source
source/mrmeshnumpy/     → NumPy integration
```

For the standalone build, we have two options:

| Approach | Complexity | Maintenance |
|----------|-----------|-------------|
| Use mrbind (copy MeshLib's approach) | Medium | Easy (matches MeshLib) |
| Hand-written pybind11 | High | Custom |

**Recommendation:** Use mrbind to maintain feature parity.

---

## Step 4.1: Install Python Development Tools

```powershell
# Install Python (if not already)
winget install Python.Python.3.12

# Verify
python --version
pip --version

# Install build tools
pip install pybind11 numpy wheel build
pip install cmake ninja

# Verify pybind11
python -c "import pybind11; print(pybind11.get_cmake_dir())"
```

---

## Step 4.2: Copy mrbind System

```powershell
$MESHLIB_DIR = "C:\Users\VijayRaghavVarada\Documents\Github\MeshLib"
$STANDALONE_DIR = "C:\Users\VijayRaghavVarada\Documents\Github\meshlib-standalone"

# Copy mrbind scripts
Copy-Item "$MESHLIB_DIR\scripts\mrbind" "$STANDALONE_DIR\scripts\mrbind" -Recurse -Force

# Copy modified pybind11
Copy-Item "$MESHLIB_DIR\scripts\mrbind-pybind11" "$STANDALONE_DIR\scripts\mrbind-pybind11" -Recurse -Force

# List contents
Get-ChildItem "$STANDALONE_DIR\scripts\mrbind" -Recurse | Select-Object FullName
```

---

## Step 4.3: Copy MRPython Source

```powershell
$SOURCE = "$MESHLIB_DIR\source\MRPython"
$TARGET = "$STANDALONE_DIR\source\MRPython"

# Copy all files
Get-ChildItem $SOURCE -File | ForEach-Object {
    Copy-Item $_.FullName $TARGET -Force
    Write-Host "Copied: $($_.Name)"
}
```

### Key MRPython Files:

| File | Purpose |
|------|---------|
| `MRPython.h/cpp` | Main module initialization |
| `MRPythonMesh.cpp` | Mesh class bindings |
| `MRPythonVoxels.cpp` | Voxel operations bindings |
| `MRPythonIO.cpp` | File I/O bindings |
| `MRPythonBoolean.cpp` | Boolean operations |
| `MRPythonDecimate.cpp` | Decimation bindings |
| `MRPythonOffset.cpp` | Offset operations |
| `CMakeLists.txt` | Build configuration |

---

## Step 4.4: Copy mrmeshnumpy

```powershell
$SOURCE = "$MESHLIB_DIR\source\mrmeshnumpy"
$TARGET = "$STANDALONE_DIR\source\mrmeshnumpy"

# Copy all files
Get-ChildItem $SOURCE -File | ForEach-Object {
    Copy-Item $_.FullName $TARGET -Force
    Write-Host "Copied: $($_.Name)"
}
```

### Key mrmeshnumpy Files:

| File | Purpose |
|------|---------|
| `mrmeshnumpy.h/cpp` | NumPy array ↔ MRMesh conversion |
| `CMakeLists.txt` | Build configuration |

---

## Step 4.5: Update Root CMakeLists.txt

Add Python binding options to your root `CMakeLists.txt`:

```cmake
# Python bindings
option(MESHLIB_BUILD_PYTHON "Build Python bindings" OFF)
option(MESHLIB_PYTHON_VERSION "Python version to use" "3.12")

IF(MESHLIB_BUILD_PYTHON)
    # Find Python
    find_package(Python3 ${MESHLIB_PYTHON_VERSION} EXACT 
                 COMPONENTS Interpreter Development NumPy
                 REQUIRED)
    
    # Find or fetch pybind11
    find_package(pybind11 CONFIG)
    IF(NOT pybind11_FOUND)
        message(STATUS "pybind11 not found, fetching...")
        include(FetchContent)
        FetchContent_Declare(
            pybind11
            GIT_REPOSITORY https://github.com/pybind/pybind11
            GIT_TAG v2.11.1
        )
        FetchContent_MakeAvailable(pybind11)
    ENDIF()
    
    # Add Python modules
    add_subdirectory(source/mrmeshnumpy)
    add_subdirectory(source/MRPython)
ENDIF()
```

---

## Step 4.6: Configure MRPython CMakeLists.txt

Verify `source/MRPython/CMakeLists.txt`:

```cmake
project(meshlib CXX)

# This creates the Python module
pybind11_add_module(meshlib 
    MRPython.cpp
    MRPythonMesh.cpp
    MRPythonIO.cpp
    MRPythonBoolean.cpp
    MRPythonDecimate.cpp
    MRPythonOffset.cpp
    # Add more binding files as needed
)

target_link_libraries(meshlib
    PRIVATE
        MRMesh
        MRVoxels
        mrmeshnumpy
        Python3::NumPy
)

target_include_directories(meshlib
    PRIVATE
        ${CMAKE_SOURCE_DIR}/source
        ${Python3_NumPy_INCLUDE_DIRS}
)

# Output to python package directory
set_target_properties(meshlib PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python/meshlib"
)

# Copy Python support files
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
    ${CMAKE_BINARY_DIR}/python/meshlib/__init__.py
    COPYONLY
)
```

---

## Step 4.7: Create Python Package Structure

```powershell
$PYTHON_DIR = "$STANDALONE_DIR\python"

# Create package directories
New-Item -ItemType Directory -Force -Path "$PYTHON_DIR\meshlib"
New-Item -ItemType Directory -Force -Path "$PYTHON_DIR\tests"

# Create __init__.py
@"
# MeshLib Python Bindings
# Auto-generated by CMake

from .meshlib import *

__version__ = "0.1.0"
__all__ = ['Mesh', 'load_mesh', 'save_mesh', 'offset_mesh', 'boolean_union']
"@ | Set-Content "$PYTHON_DIR\meshlib\__init__.py"

# Create pyproject.toml
@"
[build-system]
requires = ["setuptools>=61.0", "wheel", "cmake", "ninja", "pybind11>=2.10"]
build-backend = "setuptools.build_meta"

[project]
name = "meshlib-standalone"
version = "0.1.0"
description = "MeshLib 3D mesh processing library"
readme = "README.md"
requires-python = ">=3.9"
license = {text = "MIT"}
classifiers = [
    "Development Status :: 4 - Beta",
    "Intended Audience :: Developers",
    "License :: OSI Approved :: MIT License",
    "Programming Language :: Python :: 3",
    "Programming Language :: Python :: 3.9",
    "Programming Language :: Python :: 3.10",
    "Programming Language :: Python :: 3.11",
    "Programming Language :: Python :: 3.12",
    "Topic :: Scientific/Engineering",
]
dependencies = [
    "numpy>=1.20",
]

[project.optional-dependencies]
dev = [
    "pytest",
    "pytest-cov",
]

[tool.setuptools]
packages = ["meshlib"]
"@ | Set-Content "$STANDALONE_DIR\pyproject.toml"
```

---

## Step 4.8: Build Python Bindings

```powershell
Set-Location $STANDALONE_DIR

# Configure with Python enabled
cmake --preset native-release -DMESHLIB_BUILD_PYTHON=ON

# Build
cmake --build --preset native-release

# Check for Python module
Get-ChildItem -Path "build\native-release\python\meshlib" -Recurse
```

### Expected Files:
```
build/native-release/python/meshlib/
├── __init__.py
├── meshlib.pyd (Windows) / meshlib.so (Linux/Mac)
└── (additional Python files)
```

---

## Step 4.9: Create Python Test Suite

Create `$STANDALONE_DIR\tests\python\test_meshlib.py`:

```python
"""
MeshLib Python Bindings Tests
Tests all major functionality to ensure parity with main MeshLib.
"""

import pytest
import numpy as np
import sys
import os

# Add build output to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../build/native-release/python'))

import meshlib
from meshlib import Mesh


class TestMeshCreation:
    """Test mesh creation and basic operations."""
    
    def test_create_empty_mesh(self):
        """Create an empty mesh."""
        mesh = Mesh()
        assert mesh is not None
        assert mesh.num_vertices() == 0
        assert mesh.num_faces() == 0
    
    def test_create_cube(self):
        """Create a cube primitive."""
        mesh = meshlib.make_cube()
        assert mesh is not None
        assert mesh.num_vertices() == 8
        assert mesh.num_faces() == 12  # 6 faces * 2 triangles
    
    def test_create_sphere(self):
        """Create a sphere primitive."""
        mesh = meshlib.make_sphere(radius=1.0, subdivisions=3)
        assert mesh is not None
        assert mesh.num_vertices() > 0
    
    def test_create_cylinder(self):
        """Create a cylinder primitive."""
        mesh = meshlib.make_cylinder(radius=1.0, height=2.0, segments=32)
        assert mesh is not None


class TestMeshProperties:
    """Test mesh property calculations."""
    
    def test_bounding_box(self):
        """Calculate bounding box."""
        mesh = meshlib.make_cube()
        bbox = mesh.bounding_box()
        assert bbox is not None
        assert bbox.min is not None
        assert bbox.max is not None
    
    def test_volume(self):
        """Calculate mesh volume."""
        mesh = meshlib.make_cube()
        volume = mesh.volume()
        assert volume > 0
        # Unit cube should have volume ~1.0
        assert 0.9 < volume < 1.1
    
    def test_area(self):
        """Calculate surface area."""
        mesh = meshlib.make_cube()
        area = mesh.area()
        assert area > 0
        # Unit cube should have area ~6.0
        assert 5.9 < area < 6.1
    
    def test_centroid(self):
        """Calculate mesh centroid."""
        mesh = meshlib.make_cube()
        centroid = mesh.centroid()
        assert len(centroid) == 3


class TestMeshIO:
    """Test mesh file I/O."""
    
    def test_save_stl(self, tmp_path):
        """Save mesh to STL file."""
        mesh = meshlib.make_cube()
        filepath = tmp_path / "test.stl"
        result = meshlib.save_mesh(mesh, str(filepath))
        assert result is True
        assert filepath.exists()
    
    def test_load_stl(self, tmp_path):
        """Load mesh from STL file."""
        # First save
        mesh = meshlib.make_cube()
        filepath = tmp_path / "test.stl"
        meshlib.save_mesh(mesh, str(filepath))
        
        # Then load
        loaded = meshlib.load_mesh(str(filepath))
        assert loaded is not None
        assert loaded.num_vertices() == mesh.num_vertices()
    
    def test_save_ply(self, tmp_path):
        """Save mesh to PLY file."""
        mesh = meshlib.make_cube()
        filepath = tmp_path / "test.ply"
        result = meshlib.save_mesh(mesh, str(filepath))
        assert result is True
    
    def test_save_obj(self, tmp_path):
        """Save mesh to OBJ file."""
        mesh = meshlib.make_cube()
        filepath = tmp_path / "test.obj"
        result = meshlib.save_mesh(mesh, str(filepath))
        assert result is True


class TestMeshOffset:
    """Test mesh offset operations (requires OpenVDB)."""
    
    def test_positive_offset(self):
        """Expand mesh with positive offset."""
        mesh = meshlib.make_cube()
        original_volume = mesh.volume()
        
        result = meshlib.offset_mesh(mesh, 0.1)
        assert result is not None
        assert result.volume() > original_volume
    
    def test_negative_offset(self):
        """Shrink mesh with negative offset."""
        mesh = meshlib.make_cube()
        original_volume = mesh.volume()
        
        result = meshlib.offset_mesh(mesh, -0.05)
        assert result is not None
        assert result.volume() < original_volume
    
    def test_shell_offset(self):
        """Create hollow shell."""
        mesh = meshlib.make_cube()
        result = meshlib.shell_offset(mesh, 0.1)
        assert result is not None


class TestBooleanOperations:
    """Test boolean operations."""
    
    def test_union(self):
        """Boolean union of two meshes."""
        mesh1 = meshlib.make_cube()
        mesh2 = meshlib.make_cube()
        
        # Translate mesh2
        mesh2.translate([0.5, 0.0, 0.0])
        
        result = meshlib.boolean_union(mesh1, mesh2)
        assert result is not None
        assert result.volume() > mesh1.volume()
    
    def test_difference(self):
        """Boolean difference of two meshes."""
        mesh1 = meshlib.make_cube()
        mesh2 = meshlib.make_cube()
        mesh2.translate([0.5, 0.0, 0.0])
        
        result = meshlib.boolean_difference(mesh1, mesh2)
        assert result is not None
    
    def test_intersection(self):
        """Boolean intersection of two meshes."""
        mesh1 = meshlib.make_cube()
        mesh2 = meshlib.make_cube()
        mesh2.translate([0.5, 0.0, 0.0])
        
        result = meshlib.boolean_intersection(mesh1, mesh2)
        assert result is not None


class TestDecimation:
    """Test mesh decimation."""
    
    def test_decimate_by_ratio(self):
        """Decimate mesh by face ratio."""
        mesh = meshlib.make_sphere(subdivisions=5)
        original_faces = mesh.num_faces()
        
        result = meshlib.decimate(mesh, ratio=0.5)
        assert result is not None
        assert result.num_faces() < original_faces
    
    def test_decimate_by_error(self):
        """Decimate mesh by maximum error."""
        mesh = meshlib.make_sphere(subdivisions=5)
        original_faces = mesh.num_faces()
        
        result = meshlib.decimate(mesh, max_error=0.01)
        assert result is not None
        assert result.num_faces() <= original_faces


class TestNumPyIntegration:
    """Test NumPy array integration."""
    
    def test_get_vertices_as_numpy(self):
        """Get vertices as NumPy array."""
        mesh = meshlib.make_cube()
        vertices = mesh.vertices_numpy()
        
        assert isinstance(vertices, np.ndarray)
        assert vertices.shape == (8, 3)
        assert vertices.dtype == np.float32
    
    def test_get_faces_as_numpy(self):
        """Get faces as NumPy array."""
        mesh = meshlib.make_cube()
        faces = mesh.faces_numpy()
        
        assert isinstance(faces, np.ndarray)
        assert faces.shape[1] == 3  # Triangle indices
        assert faces.dtype == np.int32
    
    def test_create_mesh_from_numpy(self):
        """Create mesh from NumPy arrays."""
        # Define a tetrahedron
        vertices = np.array([
            [0, 0, 0],
            [1, 0, 0],
            [0.5, 1, 0],
            [0.5, 0.5, 1]
        ], dtype=np.float32)
        
        faces = np.array([
            [0, 1, 2],
            [0, 1, 3],
            [1, 2, 3],
            [0, 2, 3]
        ], dtype=np.int32)
        
        mesh = meshlib.Mesh.from_numpy(vertices, faces)
        assert mesh is not None
        assert mesh.num_vertices() == 4
        assert mesh.num_faces() == 4


class TestMeshRepair:
    """Test mesh repair operations."""
    
    def test_fill_holes(self):
        """Fill holes in mesh."""
        mesh = meshlib.make_cube()
        # Note: cube is already closed, so this tests the function exists
        result = meshlib.fill_holes(mesh)
        assert result is not None
    
    def test_remove_duplicate_vertices(self):
        """Remove duplicate vertices."""
        mesh = meshlib.make_cube()
        result = meshlib.remove_duplicate_vertices(mesh)
        assert result is not None
    
    def test_fix_self_intersections(self):
        """Fix self-intersecting faces."""
        mesh = meshlib.make_cube()
        result = meshlib.fix_self_intersections(mesh)
        assert result is not None


class TestTransformations:
    """Test mesh transformations."""
    
    def test_translate(self):
        """Translate mesh."""
        mesh = meshlib.make_cube()
        original_centroid = mesh.centroid()
        
        mesh.translate([1.0, 2.0, 3.0])
        new_centroid = mesh.centroid()
        
        assert abs(new_centroid[0] - original_centroid[0] - 1.0) < 0.001
    
    def test_scale(self):
        """Scale mesh."""
        mesh = meshlib.make_cube()
        original_volume = mesh.volume()
        
        mesh.scale(2.0)
        new_volume = mesh.volume()
        
        assert abs(new_volume - original_volume * 8) < 0.001
    
    def test_rotate(self):
        """Rotate mesh."""
        mesh = meshlib.make_cube()
        original_bbox = mesh.bounding_box()
        
        # Rotate 45 degrees around Z axis
        mesh.rotate([0, 0, 1], np.pi / 4)
        
        # Bounding box should change
        new_bbox = mesh.bounding_box()
        # After rotation, bbox will be larger


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
```

---

## Step 4.10: Run Python Tests

```powershell
# Install pytest
pip install pytest pytest-cov

# Set Python path
$env:PYTHONPATH = "$STANDALONE_DIR\build\native-release\python"

# Run tests
cd $STANDALONE_DIR
pytest tests/python/test_meshlib.py -v

# Run with coverage
pytest tests/python/test_meshlib.py -v --cov=meshlib
```

---

## Step 4.11: Build Python Wheel

```powershell
# Install build tools
pip install build wheel

# Build wheel
cd $STANDALONE_DIR
python -m build --wheel

# Check wheel
Get-ChildItem dist/
```

### Expected Output:
```
dist/
└── meshlib_standalone-0.1.0-cp312-cp312-win_amd64.whl
```

---

## Step 4.12: Install and Test Wheel

```powershell
# Install the wheel
pip install dist/meshlib_standalone-0.1.0-cp312-cp312-win_amd64.whl

# Test import
python -c "import meshlib; print(meshlib.__version__)"

# Run a quick test
python -c @"
import meshlib
mesh = meshlib.make_cube()
print(f'Cube: {mesh.num_vertices()} vertices, {mesh.num_faces()} faces')
print(f'Volume: {mesh.volume()}')
"@
```

---

## Phase 4 Checklist

```markdown
## Phase 4 Completion Checklist

### Setup
- [ ] Python 3.9+ installed
- [ ] pybind11 installed (pip or vcpkg)
- [ ] NumPy installed
- [ ] mrbind scripts copied
- [ ] mrbind-pybind11 copied

### Source Files
- [ ] MRPython sources copied
- [ ] mrmeshnumpy sources copied
- [ ] __init__.py created
- [ ] pyproject.toml created

### CMake Configuration
- [ ] MESHLIB_BUILD_PYTHON option added
- [ ] Python3 found (Interpreter, Development, NumPy)
- [ ] pybind11 found or fetched
- [ ] MRPython CMakeLists.txt configured

### Build
- [ ] cmake --preset native-release -DMESHLIB_BUILD_PYTHON=ON
- [ ] meshlib.pyd/.so built
- [ ] Python package directory created

### Tests
- [ ] test_meshlib.py created
- [ ] pytest runs successfully
- [ ] Mesh creation works
- [ ] File I/O works
- [ ] Offset works
- [ ] Boolean operations work
- [ ] NumPy integration works

### Wheel
- [ ] pyproject.toml valid
- [ ] python -m build succeeds
- [ ] Wheel file created
- [ ] Wheel installs correctly
- [ ] Installed package imports

### Documentation
- [ ] Python API documentation generated
- [ ] Usage examples written
```

---

## Troubleshooting

### pybind11 Not Found
```powershell
pip install pybind11
# Or use vcpkg
vcpkg install pybind11:x64-windows
```

### NumPy Include Path Missing
```cmake
find_package(Python3 COMPONENTS NumPy REQUIRED)
target_include_directories(meshlib PRIVATE ${Python3_NumPy_INCLUDE_DIRS})
```

### Module Import Error
```python
# Check if the .pyd/.so file is in the right place
import sys
print(sys.path)

# Manually add path
sys.path.insert(0, '/path/to/build/python')
```

### Symbol Not Exported
Ensure all classes/functions have proper PYBIND11_MODULE exports.

---

## Next Phase

Once all items are checked, proceed to **PHASE_5_WEBASSEMBLY.md**

---

*Phase 4 Version: 1.0*
*Created: January 13, 2026*