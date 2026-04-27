# MeshLib Standalone

Industrial-grade mesh processing library for Python and WebAssembly.

## Overview

This is a standalone distribution of [MeshLib](https://github.com/MeshInspector/MeshLib) that provides:

- **Python Package** (`meshlib-*.whl`) - For Python scripts, ML pipelines, automation
- **WebAssembly Module** (`meshlib.wasm` + `meshlib.js`) - For browser apps, three.js integration

**100% feature parity** with the main MeshLib library (excluding GUI components).

## Features

### Core Mesh Operations
- Boolean operations (union, difference, intersection)
- Mesh decimation and subdivision
- Hole filling and mesh repair
- Smoothing and remeshing
- ICP alignment
- File I/O (STL, OBJ, PLY, OFF)

### Extended File Formats (MRIOExtras)
- GLTF/GLB (tinygltf)
- 3MF manufacturing format (tinyxml2)
- E57 point cloud format (libE57Format)
- LAS/LAZ point cloud format (laz-perf)
- OpenCTM compressed mesh format
- JPEG, PNG, TIFF image formats

### Voxel Operations (OpenVDB)
- Mesh offset (expand/shrink meshes)
- Marching cubes
- Mesh to signed distance field
- Volume boolean operations
- DICOM medical imaging support

## Architecture

### Core Libraries
- **MRMesh** (~7 MB) - Core mesh processing library
- **MRVoxels** (~4 MB) - Voxel operations with OpenVDB integration
- **MRIOExtras** - Extended file format support (GLTF, 3MF, E57, LAS, CTM, JPEG, PNG, TIFF)

### Python Bindings
- **mrmeshpy.pyd** - Core mesh operations for Python
- **mrvoxelspy.pyd** - Voxel operations for Python

## Building

### Prerequisites
- CMake 3.18+
- Ninja (recommended)
- vcpkg (for dependencies)
- Python 3.8+ (for Python bindings)
- Emscripten SDK (for WebAssembly)

### Native Build
```bash
cmake --preset native-release
cmake --build --preset native-release
```

### WebAssembly Build
```bash
cmake --preset wasm-release
cmake --build --preset wasm-release
```

## Python Bindings Architecture

### Custom pybind11 Fork (mrbind-pybind11)

This standalone distribution uses a **custom pybind11 fork** from the main MeshLib project,
located in `thirdparty/mrbind-pybind11/`. This fork provides critical features for
cross-version Python compatibility that standard pybind11 does not offer.

#### Why a Custom Fork?

Standard pybind11 compiles bindings against a specific Python version (e.g., Python 3.11).
This means you need separate wheel builds for each Python version (3.8, 3.9, 3.10, 3.11, 3.12, 3.13).

MeshLib's pybind11 fork uses **Python's Stable ABI** (`Py_LIMITED_API`) to create
**single binaries that work across all Python 3.8+ versions**. This dramatically
simplifies distribution and reduces wheel sizes.

#### Key Components

1. **pybind11nonlimitedapi_stubs.dll** (Version-Independent)
   - Stub library providing stable ABI function implementations
   - Links against `python3.lib` (the stable ABI library)
   - Single binary works with all Python versions

2. **pybind11nonlimitedapi_meshlib_standalone_X.Y.dll** (Version-Specific Shims)
   - Small shim libraries for each Python minor version (3.8, 3.9, etc.)
   - Provides version-specific implementations where the stable ABI is insufficient
   - Named with suffix format: `<suffix>_<major>.<minor>`

#### Compiler Definitions

The following definitions are set globally for proper pybind11 integration:

```cmake
# Python stable ABI targeting Python 3.8+
add_compile_definitions(Py_LIMITED_API=0x030800f0)

# pybind11 internals version for ABI compatibility
add_compile_definitions(PYBIND11_INTERNALS_VERSION=5)

# Custom compiler/ABI strings for cross-compiler compatibility
add_compile_definitions(PYBIND11_COMPILER_TYPE="_meshlib")
add_compile_definitions(PYBIND11_BUILD_ABI="_meshlib")

# Shim library naming suffix
add_compile_definitions(PYBIND11_NONLIMITEDAPI_LIB_SUFFIX_FOR_MODULE="meshlib_standalone")
```

#### Build Configuration

Python bindings are configured in `CMakeLists.txt`:

```cmake
set(PYBIND11_NONLIMITEDAPI_PYTHON_MIN_VERSION_HEX 0x030800f0)  # Python 3.8 minimum
set(PYBIND11_NONLIMITEDAPI_SUFFIX "meshlib_standalone")        # Library suffix
option(PYBIND11_NONLIMITEDAPI_BUILD_STUBS ON)                  # Build stubs library
```

### Deviations from Standard pybind11

| Feature | Standard pybind11 | MeshLib's mrbind-pybind11 |
|---------|-------------------|---------------------------|
| Python Version Targeting | Single version per build | All versions 3.8+ |
| ABI Stability | Version-specific | Stable ABI (python3.lib) |
| Distribution | Separate wheels per version | Single wheel for all |
| Required Libraries | python3X.lib | python3.lib + shims |
| Header Location | vcpkg/include/pybind11 | thirdparty/mrbind-pybind11/include |

### File Structure

```
standalone/
├── thirdparty/                         # All bundled dependencies
│   ├── CMakeLists.txt                  # Thirdparty build configuration
│   ├── mrbind-pybind11/                # Custom pybind11 fork (stable ABI)
│   ├── parallel-hashmap/               # Header-only hash maps
│   ├── openvdb/                        # Voxel data structures (v9, v10)
│   ├── c-blosc/                        # Compression for OpenVDB
│   ├── onetbb/                         # Intel Threading Building Blocks
│   ├── eigen/                          # Linear algebra
│   ├── fmt/                            # String formatting
│   ├── spdlog/                         # Logging
│   ├── jsoncpp/                        # JSON parsing
│   ├── expected/                       # std::expected backport
│   ├── googletest/                     # Testing framework
│   ├── tinygltf/                       # GLTF format support
│   ├── tinyxml2/                       # XML parsing
│   ├── libE57Format/                   # E57 point cloud format
│   ├── laz-perf/                       # LAS/LAZ point cloud format
│   ├── libjpeg-turbo/                  # JPEG support
│   ├── libzip/                         # ZIP archive support
│   ├── mbedtls/                        # TLS/SSL
│   └── vcpkg/                          # vcpkg configuration
│       ├── triplets/                   # Custom triplets
│       └── ports/                      # Custom ports
├── source/
│   ├── MRMesh/                         # Core mesh library
│   ├── MRVoxels/                       # Voxel operations (OpenVDB)
│   ├── MRPch/                          # Precompiled headers
│   ├── MRMeshC/                        # C API bindings
│   ├── MRIOExtras/                     # Extra I/O formats
│   └── MRPythonBindings/               # Python binding implementations
│       ├── mrmeshpy.cpp                # Module entry point
│       ├── MRMeshBindings.cpp          # Mesh class bindings
│       ├── MRVectorBindings.cpp        # Vector type bindings
│       ├── MRBoxBindings.cpp           # Box type bindings
│       └── MRMeshOperationsBindings.cpp
├── cmake/Modules/                      # CMake configuration modules
├── vcpkg.json                          # vcpkg dependency manifest
├── CMakeLists.txt                      # Main build configuration
├── CMakePresets.json                   # Build presets (native, wasm)
└── build/bin/                          # Build output
    ├── MRMesh.dll                      # Core mesh library (~7 MB)
    ├── MRVoxels.dll                    # Voxel operations (~4 MB)
    ├── mrmeshpy.pyd                    # Python module
    ├── mrvoxelspy.pyd                  # Voxels Python module
    ├── pybind11nonlimitedapi_stubs.dll # Stable ABI stubs
    └── pybind11nonlimitedapi_meshlib_standalone_3.X.dll
```

### Usage Example

```python
import mrmeshpy as mr

# Create a unit cube
cube = mr.makeCube()
print(f"Volume: {cube.volume()}")  # 1.0
print(f"Area: {cube.area()}")      # 6.0

# Create a torus
torus = mr.makeTorus(primaryRadius=1.0, secondaryRadius=0.2)

# Boolean operations
result = mr.boolean(cube, torus, mr.BooleanOperation.Intersection)
```

## Thirdparty Dependencies

The standalone distribution includes **all thirdparty dependencies** bundled in the `thirdparty/` folder.
This ensures the build is truly self-contained and can be built on any platform without external dependencies.

### Dependency Strategy by Platform

| Platform | Primary Source | Fallback |
|----------|---------------|----------|
| **Windows** | vcpkg | thirdparty/ (build from source) |
| **Linux/macOS** | vcpkg or system packages | thirdparty/ (build from source) |
| **Emscripten/WASM** | thirdparty/ (build from source) | N/A |

### Bundled Libraries

All dependencies are included in `thirdparty/` for self-contained builds:

#### Core Libraries

| Library | Version | Purpose | Source |
|---------|---------|---------|--------|
| **eigen** | 3.4.0 | Linear algebra, matrices, vectors | github.com/libeigen/eigen |
| **fmt** | 10.x | Fast string formatting | github.com/fmtlib/fmt |
| **spdlog** | 1.12.x | Fast C++ logging | github.com/gabime/spdlog |
| **jsoncpp** | 1.9.x | JSON parsing and serialization | github.com/open-source-parsers/jsoncpp |
| **onetbb** | 2021.x | Intel Threading Building Blocks | github.com/oneapi-src/oneTBB |
| **expected** | 1.1.0 | std::expected backport (C++17) | MeshLib fork of TartanLlama/expected |
| **parallel-hashmap** | 1.3.8 | High-performance hash maps | github.com/greg7mdp/parallel-hashmap |

#### Voxel Operations

| Library | Version | Purpose | Source |
|---------|---------|---------|--------|
| **openvdb** | v10.1.0 | Sparse volumetric data structures | github.com/AcademySoftwareFoundation/openvdb |
| **c-blosc** | 1.21.x | Compression for OpenVDB | github.com/Blosc/c-blosc |

#### File Format Support

| Library | Version | Purpose | Source |
|---------|---------|---------|--------|
| **tinygltf** | 2.8.x | GLTF/GLB 3D format | github.com/syoyo/tinygltf |
| **tinyxml2** | 10.x | XML parsing (3MF support) | github.com/leethomason/tinyxml2 |
| **libE57Format** | 3.x | E57 point cloud format | github.com/asmaloney/libE57Format |
| **laz-perf** | 3.x | LAS/LAZ point cloud format | github.com/hobu/laz-perf |
| **libjpeg-turbo** | 3.x | JPEG image I/O | github.com/libjpeg-turbo/libjpeg-turbo |
| **libzip** | 1.10.x | ZIP archive support | github.com/nih-at/libzip |

#### Security & Network

| Library | Version | Purpose | Source |
|---------|---------|---------|--------|
| **mbedtls** | 3.x | TLS/SSL for secure connections | github.com/Mbed-TLS/mbedtls |

#### Python Bindings

| Library | Version | Purpose | Source |
|---------|---------|---------|--------|
| **mrbind-pybind11** | 2.14.0.dev1 | Custom pybind11 fork with stable ABI | MeshLib fork |

#### Testing

| Library | Version | Purpose | Source |
|---------|---------|---------|--------|
| **googletest** | 1.14.x | C++ testing framework | github.com/google/googletest |

### Custom Forks

Two libraries are **MeshLib-specific forks** with important modifications:

#### mrbind-pybind11

Custom fork of pybind11 that enables **Python Stable ABI** support (`Py_LIMITED_API`).
This allows building a single Python wheel that works across all Python 3.8+ versions.

- **Repository:** https://github.com/MeshInspector/mrbind-pybind11
- **Key Feature:** Single binary works with Python 3.8, 3.9, 3.10, 3.11, 3.12, 3.13+

#### expected (tl::expected)

Fork of TartanLlama/expected with additional patches:
- Apple Silicon (ARM64) support
- CMake improvements for subdirectory usage

- **Repository:** https://github.com/Developer-Ecosystem-Engineering/expected

### vcpkg Configuration

The `thirdparty/vcpkg/` folder contains:

```
vcpkg/
├── triplets/                    # Custom vcpkg triplets
│   ├── x64-windows-meshlib.cmake
│   ├── x64-linux-meshlib.cmake
│   └── arm64-linux-meshlib.cmake
├── ports/                       # Custom vcpkg ports
│   ├── clip/                    # Clipboard library
│   ├── laz-perf/               # LAZ point cloud support
│   ├── opencascade-minimal/    # Minimal OpenCASCADE (STEP)
│   └── openctm/                # CTM mesh format
└── downloads/                   # Cached downloads
```

### Building Thirdparty from Source

For Emscripten/WebAssembly builds, dependencies are built from source:

```bash
# The thirdparty CMakeLists.txt handles building all dependencies
cmake -S thirdparty -B thirdparty_build \
    -DCMAKE_TOOLCHAIN_FILE=$EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake \
    -DMR_EMSCRIPTEN=1
cmake --build thirdparty_build
```

## License

See [LICENSE](LICENSE) for details.

The mrbind-pybind11 fork is based on pybind11 and is licensed under the BSD-3-Clause license.
