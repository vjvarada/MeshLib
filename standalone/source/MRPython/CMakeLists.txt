project(MRPython CXX)

message(STATUS "MRPython: Building auto-generated pybind11 bindings")

# Find Python
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)

# Use MeshLib's mrbind-pybind11 fork
set(PYBIND11_DIR "${CMAKE_SOURCE_DIR}/../thirdparty/mrbind-pybind11")
if(NOT EXISTS "${PYBIND11_DIR}/include/pybind11/pybind11.h")
    # Fallback to system pybind11
    find_package(pybind11 CONFIG REQUIRED)
else()
    # Add mrbind-pybind11 headers
    add_library(pybind11_headers INTERFACE)
    target_include_directories(pybind11_headers INTERFACE "${PYBIND11_DIR}/include")
    target_link_libraries(pybind11_headers INTERFACE Python3::Python)
endif()

# Path to mrbind includes
set(MRBIND_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/../thirdparty/mrbind/include")

# Generated bindings file
set(GENERATED_BINDINGS "${CMAKE_SOURCE_DIR}/temp_bindings/bindings_all.cpp")

if(EXISTS "${GENERATED_BINDINGS}")
    message(STATUS "Using auto-generated bindings from: ${GENERATED_BINDINGS}")
    
    # Create the Python module
    Python3_add_library(mrmeshpy MODULE
        ${GENERATED_BINDINGS}
        MRMeshBindings.cpp  # Manual bindings for topology navigation etc.
    )
    
    target_compile_definitions(mrmeshpy PRIVATE
        # Tell mrbind to use pybind11 target
        MRBIND_HEADER=<mrbind/targets/pybind11.h>
        # Module name
        MB_PB11_MODULE_NAME=mrmeshpy
        # Include implementation (only one .cpp should have this)
        MB_DEFINE_IMPLEMENTATION
        # Python version compatibility
        Py_LIMITED_API=0x030900f0
        # Pybind internals identifiers
        PYBIND11_COMPILER_TYPE="_meshlib"
        PYBIND11_BUILD_ABI="_meshlib"
        PYBIND11_INTERNALS_VERSION=5
        # Strip namespaces for cleaner Python API
        MB_PB11_STRIPPED_NAMESPACES="MR"
    )
    
    target_include_directories(mrmeshpy PRIVATE
        ${MRBIND_INCLUDE_DIR}
        ${PYBIND11_DIR}/include
        ${CMAKE_SOURCE_DIR}/source
        ${CMAKE_SOURCE_DIR}/../thirdparty/parallel-hashmap
        ${CMAKE_SOURCE_DIR}/../thirdparty/eigen
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
    )
    
    target_link_libraries(mrmeshpy PRIVATE
        MRMesh
        MRIOExtras
        Python3::Python
        Python3::NumPy
    )
    
    # C++23 standard (matching mrbind parsing)
    target_compile_features(mrmeshpy PRIVATE cxx_std_23)
    
    # Output to python package directory
    set_target_properties(mrmeshpy PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python/meshlib"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python/meshlib"
    )
    
    # Clang-specific flags for compatibility
    if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        target_compile_options(mrmeshpy PRIVATE
            -fms-compatibility
            -fms-extensions
            -Wno-deprecated-declarations
        )
    endif()
    
else()
    message(WARNING "Generated bindings not found at: ${GENERATED_BINDINGS}")
    message(WARNING "Run scripts/mrbind/generate_bindings.bat first")
    
    # Fallback: build with manual bindings only
    Python3_add_library(mrmeshpy MODULE
        MRMeshBindings.cpp
    )
    
    target_include_directories(mrmeshpy PRIVATE
        ${CMAKE_SOURCE_DIR}/source
        ${Python3_INCLUDE_DIRS}
    )
    
    target_link_libraries(mrmeshpy PRIVATE
        MRMesh
        Python3::Python
    )
    
    set_target_properties(mrmeshpy PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python/meshlib"
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/python/meshlib"
    )
endif()

# Create __init__.py for the package
file(WRITE "${CMAKE_BINARY_DIR}/python/meshlib/__init__.py" 
"# MeshLib Python bindings
from .mrmeshpy import *
")
