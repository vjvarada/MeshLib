cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(MeshLibCore
    VERSION 1.0.0
    DESCRIPTION "Standalone 3D Mesh Processing Library"
    LANGUAGES CXX
)

# =============================================================================
# OPTIONS
# =============================================================================
option(MESHLIB_CORE_WASM "Build for WebAssembly" OFF)
option(MESHLIB_CORE_PYTHON "Build Python bindings" OFF)
option(MESHLIB_CORE_VOXELS "Build voxel operations (requires OpenVDB)" OFF)
option(MESHLIB_CORE_PARALLEL "Enable parallel processing with TBB" ON)
option(MESHLIB_CORE_BUILD_TESTS "Build unit tests" ON)
option(MESHLIB_CORE_BUILD_EXAMPLES "Build examples" ON)

# Detect Emscripten
if(EMSCRIPTEN)
    set(MESHLIB_CORE_WASM ON)
    set(MESHLIB_CORE_PARALLEL OFF)
    set(MESHLIB_CORE_VOXELS OFF)
    message(STATUS "Building for WebAssembly (Emscripten detected)")
endif()

# =============================================================================
# C++ STANDARD
# =============================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# =============================================================================
# OUTPUT DIRECTORIES
# =============================================================================
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Include GNUInstallDirs early for install destinations used in subdirectories
include(GNUInstallDirs)

# =============================================================================
# DEPENDENCIES
# =============================================================================

# Find or fetch Eigen
find_package(Eigen3 3.3 QUIET)
if(NOT Eigen3_FOUND)
    message(STATUS "Eigen3 not found, using bundled version")
    # Assume header-only copy in thirdparty/
    set(EIGEN3_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/eigen)
else()
    message(STATUS "Found Eigen3: ${EIGEN3_INCLUDE_DIR}")
endif()

# parallel-hashmap (header-only)
set(PHMAP_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/parallel-hashmap)

# tl-expected (header-only)
find_package(tl-expected QUIET)
if(NOT tl-expected_FOUND)
    set(TL_EXPECTED_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/expected/include)
endif()

# TBB (optional, for parallel processing)
if(MESHLIB_CORE_PARALLEL AND NOT MESHLIB_CORE_WASM)
    find_package(TBB QUIET)
    if(TBB_FOUND)
        message(STATUS "Found TBB: ${TBB_DIR}")
        set(MESHLIB_HAS_TBB ON)
    else()
        message(WARNING "TBB not found, parallel processing disabled")
        set(MESHLIB_CORE_PARALLEL OFF)
    endif()
endif()

# OpenVDB (optional, for voxel operations)
if(MESHLIB_CORE_VOXELS)
    find_package(OpenVDB QUIET)
    if(OpenVDB_FOUND)
        message(STATUS "Found OpenVDB")
        set(MESHLIB_HAS_OPENVDB ON)
    else()
        message(WARNING "OpenVDB not found, voxel operations disabled")
        set(MESHLIB_CORE_VOXELS OFF)
    endif()
endif()

# pybind11 (for Python bindings)
if(MESHLIB_CORE_PYTHON)
    find_package(pybind11 REQUIRED)
    message(STATUS "Found pybind11: ${pybind11_DIR}")
endif()

# =============================================================================
# CONFIGURE HEADER
# =============================================================================
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/include/meshlib/config.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/include/meshlib/config.h
)

# =============================================================================
# CORE LIBRARY
# =============================================================================

set(MESHLIB_CORE_SOURCES
    # Mesh data structures
    src/mesh/Mesh.cpp
    src/mesh/PointCloud.cpp
    src/mesh/Primitives.cpp
    
    # Algorithms
    src/algorithms/Boolean.cpp
    src/algorithms/Decimate.cpp
    src/algorithms/FillHole.cpp
    src/algorithms/ICP.cpp
    src/algorithms/Smooth.cpp
    src/algorithms/Subdivide.cpp
    
    # I/O
    src/io/MeshIO.cpp
)

set(MESHLIB_CORE_HEADERS
    include/meshlib/meshlib.h
    include/meshlib/core/Types.h
    include/meshlib/core/Vector.h
    include/meshlib/core/Matrix.h
    include/meshlib/core/Box.h
    include/meshlib/core/Id.h
    include/meshlib/mesh/Mesh.h
    include/meshlib/mesh/PointCloud.h
    include/meshlib/mesh/Primitives.h
    include/meshlib/algorithms/Boolean.h
    include/meshlib/algorithms/Decimate.h
    include/meshlib/algorithms/FillHole.h
    include/meshlib/algorithms/ICP.h
    include/meshlib/algorithms/Smooth.h
    include/meshlib/algorithms/Subdivide.h
    include/meshlib/io/MeshIO.h
)

# Add voxel sources if enabled
if(MESHLIB_CORE_VOXELS)
    list(APPEND MESHLIB_CORE_SOURCES
        src/voxels/Offset.cpp
        src/voxels/MarchingCubes.cpp
        src/voxels/VDBConversions.cpp
    )
    list(APPEND MESHLIB_CORE_HEADERS
        include/meshlib/voxels/Offset.h
        include/meshlib/voxels/MarchingCubes.h
    )
endif()

# Create the library
if(MESHLIB_CORE_WASM)
    # Static library for WASM
    add_library(meshlib_core STATIC ${MESHLIB_CORE_SOURCES} ${MESHLIB_CORE_HEADERS})
else()
    # Shared library for native builds
    add_library(meshlib_core SHARED ${MESHLIB_CORE_SOURCES} ${MESHLIB_CORE_HEADERS})
endif()

# Include directories
target_include_directories(meshlib_core
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# Third-party includes (use PRIVATE since these are header-only bundled deps)
target_include_directories(meshlib_core SYSTEM
    PRIVATE
        ${EIGEN3_INCLUDE_DIR}
        ${PHMAP_INCLUDE_DIR}
)

if(NOT tl-expected_FOUND)
    target_include_directories(meshlib_core SYSTEM PRIVATE ${TL_EXPECTED_INCLUDE_DIR})
else()
    target_link_libraries(meshlib_core PUBLIC tl::expected)
endif()

# Link TBB if available
if(MESHLIB_CORE_PARALLEL AND TBB_FOUND)
    target_link_libraries(meshlib_core PUBLIC TBB::tbb)
    target_compile_definitions(meshlib_core PUBLIC MESHLIB_HAS_TBB)
endif()

# Link OpenVDB if available
if(MESHLIB_CORE_VOXELS AND OpenVDB_FOUND)
    target_link_libraries(meshlib_core PUBLIC OpenVDB::openvdb)
    target_compile_definitions(meshlib_core PUBLIC MESHLIB_HAS_OPENVDB)
endif()

# Export symbols
if(NOT MESHLIB_CORE_WASM)
    target_compile_definitions(meshlib_core PRIVATE MESHLIB_CORE_EXPORTS)
endif()

# Compiler options
if(MSVC)
    target_compile_options(meshlib_core PRIVATE /W4)
else()
    target_compile_options(meshlib_core PRIVATE -Wall -Wextra -Wpedantic)
endif()

# =============================================================================
# WASM BINDINGS
# =============================================================================
if(MESHLIB_CORE_WASM)
    add_subdirectory(bindings/wasm)
endif()

# =============================================================================
# PYTHON BINDINGS
# =============================================================================
if(MESHLIB_CORE_PYTHON)
    add_subdirectory(bindings/python)
endif()

# =============================================================================
# C BINDINGS (disabled for now - needs API update)
# =============================================================================
# add_subdirectory(bindings/c)

# =============================================================================
# EXAMPLES (disabled for now - needs API update)
# =============================================================================
# if(MESHLIB_CORE_BUILD_EXAMPLES AND NOT MESHLIB_CORE_WASM)
#     add_subdirectory(examples/cpp)
# endif()

# =============================================================================
# TESTS
# =============================================================================
if(MESHLIB_CORE_BUILD_TESTS AND NOT MESHLIB_CORE_WASM)
    enable_testing()
    add_subdirectory(tests)
endif()

# =============================================================================
# INSTALLATION
# =============================================================================

install(TARGETS meshlib_core
    EXPORT meshlib_core-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/meshlib
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/include/meshlib/config.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/meshlib
)

install(EXPORT meshlib_core-targets
    FILE meshlib_core-targets.cmake
    NAMESPACE MeshLibCore::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/meshlib_core
)

# Package config
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/meshlib_core-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/meshlib_core-config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/meshlib_core
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/meshlib_core-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/meshlib_core-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/meshlib_core-config-version.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/meshlib_core
)

# =============================================================================
# SUMMARY
# =============================================================================
message(STATUS "")
message(STATUS "MeshLib Core Configuration:")
message(STATUS "  Version:          ${PROJECT_VERSION}")
message(STATUS "  Build type:       ${CMAKE_BUILD_TYPE}")
message(STATUS "  WebAssembly:      ${MESHLIB_CORE_WASM}")
message(STATUS "  Python bindings:  ${MESHLIB_CORE_PYTHON}")
message(STATUS "  Voxel operations: ${MESHLIB_CORE_VOXELS}")
message(STATUS "  Parallel (TBB):   ${MESHLIB_CORE_PARALLEL}")
message(STATUS "  Tests:            ${MESHLIB_CORE_BUILD_TESTS}")
message(STATUS "  Examples:         ${MESHLIB_CORE_BUILD_EXAMPLES}")
message(STATUS "")
