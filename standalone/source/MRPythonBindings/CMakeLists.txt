cmake_minimum_required(VERSION 3.18)
project(MRPythonBindings CXX)

# ============================================================================
# MRBind Auto-Generated Bindings Integration
# ============================================================================
# When MESHLIB_USE_MRBIND_AUTO is ON, the auto-generated bindings from 
# temp_bindings/bindings_all.cpp will be compiled. This provides automatic
# Python bindings for the entire MRMesh/MRVoxels API - full industrial power!
# NOTE: Currently disabled due to template compilation errors - using enhanced manual bindings instead
option(MESHLIB_USE_MRBIND_AUTO "Use mrbind auto-generated bindings" OFF)

# ============================================================================
# Use MeshLib's pybind11 fork for multi-version Python support
# ============================================================================
#
# MeshLib uses a custom pybind11 fork (mrbind-pybind11) that enables:
# - Single binary compatibility across Python 3.8+
# - Uses Py_LIMITED_API for stable ABI
# - Requires pybind11nonlimitedapi_stubs library (built by parent CMakeLists)
#
# The stubs library is built in the parent CMakeLists.txt before this subdirectory

# Include pybind11 headers from standalone's bundled mrbind-pybind11 fork
set(PYBIND11_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/thirdparty/mrbind-pybind11/include")
set(MRBIND_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../thirdparty/mrbind/include")

# Generated bindings file location
set(GENERATED_BINDINGS "${CMAKE_SOURCE_DIR}/temp_bindings/bindings_all.cpp")

# ============================================================================
# Source files selection based on MESHLIB_USE_MRBIND_AUTO
# ============================================================================
if(MESHLIB_USE_MRBIND_AUTO AND EXISTS "${GENERATED_BINDINGS}")
    message(STATUS "MRPythonBindings: Using AUTO-GENERATED mrbind bindings (full API)")
    message(STATUS "  Generated bindings: ${GENERATED_BINDINGS}")
    
    # Use mrbind auto-generated bindings for full API coverage
    set(MRMESHPY_SOURCES
        ${GENERATED_BINDINGS}
    )
    set(USE_MRBIND_MACROS TRUE)
else()
    if(MESHLIB_USE_MRBIND_AUTO)
        message(WARNING "MRPythonBindings: Auto-generated bindings not found!")
        message(WARNING "  Expected at: ${GENERATED_BINDINGS}")
        message(WARNING "  Run scripts/mrbind/generate_bindings.bat first")
        message(WARNING "  Falling back to manual bindings (limited API)")
    else()
        message(STATUS "MRPythonBindings: Using MANUAL bindings (limited API)")
    endif()
    
    # Use manual bindings (limited coverage)
    set(MRMESHPY_SOURCES
        mrmeshpy.cpp
        MRMeshBindings.cpp
        MRVectorBindings.cpp
        MRBoxBindings.cpp
        MRMeshOperationsBindings.cpp
    )
    set(USE_MRBIND_MACROS FALSE)
endif()

# Create shared library for mrmeshpy
add_library(mrmeshpy SHARED ${MRMESHPY_SOURCES})

# Set up Python module properties
set_target_properties(mrmeshpy PROPERTIES
    PREFIX ""  # Python modules don't have lib prefix
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/meshlib"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/meshlib"
)

# Windows-specific module extension
if(WIN32)
    set_target_properties(mrmeshpy PROPERTIES SUFFIX ".pyd")
elseif(APPLE)
    set_target_properties(mrmeshpy PROPERTIES SUFFIX ".so")
endif()

# Include directories
target_include_directories(mrmeshpy PRIVATE
    ${PYBIND11_INCLUDE_DIR}
    ${MRBIND_INCLUDE_DIR}
    ${Python_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../MRMesh
    ${CMAKE_CURRENT_SOURCE_DIR}/../MRVoxels
    ${CMAKE_CURRENT_SOURCE_DIR}/../MRPch
    ${CMAKE_SOURCE_DIR}/../thirdparty/parallel-hashmap
    ${CMAKE_SOURCE_DIR}/../thirdparty/eigen
)

# Link libraries
target_link_libraries(mrmeshpy PRIVATE
    MRMesh
    MRIOExtras
    pybind11nonlimitedapi_stubs
)

# Common compile definitions
target_compile_definitions(mrmeshpy PRIVATE
    Py_LIMITED_API=0x030800f0
    PYBIND11_NONLIMITEDAPI_LIB_SUFFIX_FOR_MODULE=\"meshlib_standalone\"
)

# Additional definitions for mrbind auto-generated bindings
if(USE_MRBIND_MACROS)
    target_compile_definitions(mrmeshpy PRIVATE
        # Tell mrbind to use pybind11 target header
        MRBIND_HEADER=<mrbind/targets/pybind11.h>
        # Module name must match output filename
        MB_PB11_MODULE_NAME=mrmeshpy
        # Only one compilation unit should define this
        MB_DEFINE_IMPLEMENTATION
        # Pybind internals identifiers (prevent cross-talk)
        PYBIND11_COMPILER_TYPE=\"_meshlib_standalone\"
        PYBIND11_BUILD_ABI=\"_meshlib_standalone\"
        PYBIND11_INTERNALS_VERSION=5
        # Strip top-level MR namespace for cleaner Python API
        MB_PB11_STRIPPED_NAMESPACES=\"MR\"
        # Include original headers
        MB_INCLUDE_ORIGINAL_HEADER=1
    )
    
    # C++23 is required for mrbind-generated code
    target_compile_features(mrmeshpy PRIVATE cxx_std_23)
    
    # MSVC requires standard-conformant preprocessor for mrbind macros
    # But clang-cl doesn't support /Zc:preprocessor  
    if(MSVC AND NOT (CMAKE_CXX_COMPILER_ID STREQUAL "Clang"))
        target_compile_options(mrmeshpy PRIVATE 
            /Zc:preprocessor
            /wd4244  # Disable warning C4244: conversion, possible loss of data
            /wd4267  # Disable warning C4267: conversion from size_t
            /bigobj  # Enable big object files (needed for large binding files)
        )
    endif()
    
    # Clang-specific flags for MS compatibility
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(mrmeshpy PRIVATE
            -fms-compatibility
            -fms-extensions
            -Wno-deprecated-declarations
            -Wno-macro-redefined
            -Wno-unused-command-line-argument
        )
    endif()
endif()

# Don't link to specific Python version on non-Windows (use dynamic lookup)
if(NOT WIN32)
    string(REPLACE "-Wl,-z,defs" "" CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS}")
    if(APPLE)
        target_link_options(mrmeshpy PRIVATE -undefined dynamic_lookup)
    endif()
else()
    # On Windows, link to the Python library directory for python3.lib (stable ABI)
    target_link_directories(mrmeshpy PRIVATE ${Python_LIBRARY_DIRS})
endif()

# Create MRVoxels Python bindings if voxels are enabled
if(MESHLIB_BUILD_VOXELS)
    add_library(mrvoxelspy SHARED
        mrvoxelspy.cpp
        MRVoxelsBindings.cpp
    )
    
    set_target_properties(mrvoxelspy PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/meshlib"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/meshlib"
    )
    
    if(WIN32)
        set_target_properties(mrvoxelspy PROPERTIES SUFFIX ".pyd")
    elseif(APPLE)
        set_target_properties(mrvoxelspy PROPERTIES SUFFIX ".so")
    endif()
    
    target_include_directories(mrvoxelspy PRIVATE
        ${PYBIND11_INCLUDE_DIR}
        ${Python_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/../MRMesh
        ${CMAKE_CURRENT_SOURCE_DIR}/../MRVoxels
        ${CMAKE_CURRENT_SOURCE_DIR}/../MRPch
    )
    
    target_link_libraries(mrvoxelspy PRIVATE
        MRMesh
        MRVoxels
        pybind11nonlimitedapi_stubs
    )
    
    target_compile_definitions(mrvoxelspy PRIVATE
        Py_LIMITED_API=0x030800f0
        PYBIND11_NONLIMITEDAPI_LIB_SUFFIX_FOR_MODULE=\"meshlib_standalone\"
    )
    
    if(NOT WIN32)
        if(APPLE)
            target_link_options(mrvoxelspy PRIVATE -undefined dynamic_lookup)
        endif()
    else()
        target_link_directories(mrvoxelspy PRIVATE ${Python_LIBRARY_DIRS})
    endif()
endif()

# Install targets
install(TARGETS mrmeshpy DESTINATION lib/meshlib)
if(TARGET mrvoxelspy)
    install(TARGETS mrvoxelspy DESTINATION lib/meshlib)
endif()
