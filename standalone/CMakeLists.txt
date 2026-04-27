cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

# ============================================================================
# MeshLib Standalone
# A complete mesh processing library without GUI components
# Full feature parity with MeshLib - Python wheels and WebAssembly
# ============================================================================

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules")

# Include common configuration modules
include(DefaultOptions)
include(ConfigureVcpkg)

# ============================================================================
# Project Definition
# ============================================================================

project(MeshLibStandalone CXX)

set(MESHLIB_PROJECT_NAME "MeshLibStandalone" CACHE STRING "Project name")
add_compile_definitions(MR_PROJECT_NAME=\"${MESHLIB_PROJECT_NAME}\")

# Compiler options
include(CompilerOptions)

# Emscripten configuration
IF(EMSCRIPTEN)
    set(MR_EMSCRIPTEN ON)
    include(ConfigureEmscripten)
ENDIF()

# ============================================================================
# Output Directories
# ============================================================================

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Visual Studio generator workaround
IF(${CMAKE_GENERATOR} MATCHES "^Visual Studio")
    foreach(BUILD_TYPE ${CMAKE_CONFIGURATION_TYPES})
        string(TOUPPER ${BUILD_TYPE} BUILD_TYPE)
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${BUILD_TYPE} ${CMAKE_BINARY_DIR}/bin)
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${BUILD_TYPE} ${CMAKE_BINARY_DIR}/bin)
        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${BUILD_TYPE} ${CMAKE_BINARY_DIR}/bin)
    endforeach()
ENDIF()

# ============================================================================
# Build Options
# ============================================================================

option(MESHLIB_BUILD_VOXELS "Build MRVoxels library (requires OpenVDB)" ON)
option(MESHLIB_BUILD_IOEXTRAS "Build MRIOExtras library" ON)
option(MESHLIB_BUILD_SYMBOLMESH "Build MRSymbolMesh library" OFF)
option(MESHLIB_BUILD_PYTHON "Build Python bindings" ON)
option(MESHLIB_BUILD_TESTS "Build unit tests" ON)
option(MESHLIB_BUILD_C_BINDING "Build C API bindings (MRMeshC)" ON)

# Feature toggles for I/O formats
option(MESHLIB_IO_3MF "Enable 3MF support" ON)
option(MESHLIB_IO_E57 "Enable E57 support" ON)
option(MESHLIB_IO_GLTF "Enable GLTF support" ON)
option(MESHLIB_IO_LAS "Enable LAS/LAZ support" ON)
option(MESHLIB_IO_DICOM "Enable DICOM support" OFF)  # GDCM dependency
option(MESHLIB_IO_PDF "Enable PDF export support" ON)  # libharu
option(MESHLIB_IO_STEP "Enable STEP support" ON)  # Requires OpenCASCADE - now available

# Set MRVoxels options based on feature flags (must be set before add_subdirectory)
IF(NOT MESHLIB_IO_DICOM)
    set(MRVOXELS_NO_DICOM ON CACHE BOOL "Disable DICOM support in MRVoxels" FORCE)
ELSE()
    set(MRVOXELS_NO_DICOM OFF CACHE BOOL "Disable DICOM support in MRVoxels" FORCE)
ENDIF()

# Emscripten overrides - initially disabled, will be enabled in Phase 5.5
IF(EMSCRIPTEN)
    set(MESHLIB_BUILD_PYTHON OFF CACHE BOOL "" FORCE)
    # Note: STEP and PDF WASM support will be added in Phase 5.5
    # For now, disable them until thirdparty is ported
    IF(NOT MESHLIB_WASM_PDF)
        set(MESHLIB_IO_PDF OFF CACHE BOOL "" FORCE)
    ENDIF()
    IF(NOT MESHLIB_WASM_STEP)
        set(MESHLIB_IO_STEP OFF CACHE BOOL "" FORCE)
    ENDIF()
ENDIF()

# ============================================================================
# Thirdparty Configuration
# ============================================================================

set(MESHLIB_THIRDPARTY_DIR "${CMAKE_SOURCE_DIR}/thirdparty" CACHE PATH "Thirdparty source location")
set(MESHLIB_THIRDPARTY_ROOT_DIR "${CMAKE_SOURCE_DIR}" CACHE PATH "Thirdparty build output location")

# Add thirdparty to search paths
IF(EXISTS "${MESHLIB_THIRDPARTY_ROOT_DIR}/include")
    include_directories(${MESHLIB_THIRDPARTY_ROOT_DIR}/include)
ENDIF()
IF(EXISTS "${MESHLIB_THIRDPARTY_ROOT_DIR}/lib")
    link_directories(${MESHLIB_THIRDPARTY_ROOT_DIR}/lib)
ENDIF()

# Parallel hashmap (header-only)
IF(EXISTS "${MESHLIB_THIRDPARTY_DIR}/parallel-hashmap")
    include_directories(${MESHLIB_THIRDPARTY_DIR}/parallel-hashmap)
ENDIF()

# ============================================================================
# Version
# ============================================================================

IF(NOT "$ENV{MR_VERSION}" STREQUAL "")
    set(MESHLIB_VERSION $ENV{MR_VERSION})
ELSE()
    set(MESHLIB_VERSION "1.0.0-standalone")
ENDIF()
message(STATUS "MeshLib Standalone version: ${MESHLIB_VERSION}")

# ============================================================================
# Install Paths
# ============================================================================

include(GNUInstallDirs)
set(MR_BIN_DIR "${CMAKE_INSTALL_BINDIR}")
set(MR_INCLUDE_DIR "${CMAKE_INSTALL_INCLUDEDIR}/MeshLib")
set(MR_MAIN_LIB_DIR "${CMAKE_INSTALL_LIBDIR}/MeshLib")
set(MR_CONFIG_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/MeshLib")
set(MR_PY_LIB_DIR "${CMAKE_INSTALL_LIBDIR}/MeshLib/meshlib")

# ============================================================================
# Source Directory
# ============================================================================

set(PROJECT_SOURCE_DIR ${CMAKE_SOURCE_DIR}/source)
include_directories(${PROJECT_SOURCE_DIR})

# ============================================================================
# Platform Detection
# ============================================================================

include(DetectPlatform)

# ============================================================================
# Python Configuration
# ============================================================================

IF(MESHLIB_BUILD_PYTHON AND NOT EMSCRIPTEN)
    # Use pybind11's FindPython approach for best compatibility
    set(PYBIND11_FINDPYTHON ON)
    
    # On Windows with vcpkg, prefer to use system Python if development files are available
    # vcpkg's Python has limited development support
    IF(WIN32 AND EXISTS "$ENV{LOCALAPPDATA}/Programs/Python")
        # Find the most recent Python version in the default install location
        file(GLOB _PYTHON_VERSIONS "$ENV{LOCALAPPDATA}/Programs/Python/Python3*")
        if(_PYTHON_VERSIONS)
            list(SORT _PYTHON_VERSIONS)
            list(REVERSE _PYTHON_VERSIONS)
            list(GET _PYTHON_VERSIONS 0 _PYTHON_ROOT)
            get_filename_component(_PYTHON_VERSION_DIR "${_PYTHON_ROOT}" NAME)
            message(STATUS "Found system Python installation: ${_PYTHON_ROOT}")
            
            # Extract version number from directory name (e.g., "Python313" -> "313")
            string(REGEX REPLACE "Python" "" _PYTHON_VER_NUM "${_PYTHON_VERSION_DIR}")
            
            # Set hints for FindPython
            set(Python_ROOT_DIR "${_PYTHON_ROOT}")
            set(Python_EXECUTABLE "${_PYTHON_ROOT}/python.exe")
            set(Python_INCLUDE_DIR "${_PYTHON_ROOT}/include")
            set(Python_LIBRARY "${_PYTHON_ROOT}/libs/python${_PYTHON_VER_NUM}.lib")
        endif()
    ENDIF()
    
    # Try to find Python - first try the specific version if requested
    IF(DEFINED MESHLIB_PYTHON_VERSION)
        find_package(Python ${MESHLIB_PYTHON_VERSION} EXACT COMPONENTS Interpreter Development.Module)
    ELSE()
        # Find any Python version with development module support
        find_package(Python COMPONENTS Interpreter Development.Module)
    ENDIF()
    
    if(Python_FOUND)
        message(STATUS "Python found: ${Python_VERSION} at ${Python_EXECUTABLE}")
    else()
        message(WARNING "Python with Development.Module not found - Python bindings may not build")
    endif()
ENDIF()

# ============================================================================
# Thirdparty Libraries (built from source)
# ============================================================================

# Build OpenCTM if CTM support is enabled and we have it
IF(MESHLIB_BUILD_IOEXTRAS AND EXISTS "${MESHLIB_THIRDPARTY_DIR}/OpenCTM-git")
    add_subdirectory(${MESHLIB_THIRDPARTY_DIR}/OpenCTM-git ${CMAKE_BINARY_DIR}/OpenCTM)
    # Suppress warnings for third-party OpenCTM library
    IF(TARGET OpenCTM)
        IF(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
            target_compile_options(OpenCTM PRIVATE 
                -Wno-misleading-indentation
                -Wno-unused-but-set-variable
                -Wno-self-assign
            )
        ENDIF()
    ENDIF()
ENDIF()

# Build laz-perf if LAS support is enabled
IF(MESHLIB_BUILD_IOEXTRAS AND MESHLIB_IO_LAS AND EXISTS "${MESHLIB_THIRDPARTY_DIR}/laz-perf")
    set(WITH_TESTS OFF CACHE BOOL "" FORCE)
    set(WITH_TOOLS OFF CACHE BOOL "" FORCE)
    set(WITH_EMBIND OFF CACHE BOOL "" FORCE)
    add_subdirectory(${MESHLIB_THIRDPARTY_DIR}/laz-perf ${CMAKE_BINARY_DIR}/laz-perf)
    # Disable treating warnings as errors for laz-perf (third-party code)
    IF(TARGET lazperf)
        target_compile_options(lazperf PRIVATE /WX-)
    ENDIF()
    IF(TARGET lazperf_s)
        target_compile_options(lazperf_s PRIVATE /WX-)
    ENDIF()
ENDIF()

# Build OpenVDB if voxels are enabled and not found via vcpkg
IF(MESHLIB_BUILD_VOXELS)
    find_package(OpenVDB 10 QUIET)
    IF(NOT OpenVDB_FOUND AND EXISTS "${MESHLIB_THIRDPARTY_DIR}/openvdb/v10")
        message(STATUS "Building bundled OpenVDB v10 from thirdparty...")
        set(OPENVDB_ENABLE_UNINSTALL OFF CACHE BOOL "" FORCE)
        set(OPENVDB_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
        set(OPENVDB_CORE_SHARED ON CACHE BOOL "" FORCE)
        set(OPENVDB_CORE_STATIC OFF CACHE BOOL "" FORCE)
        set(OPENVDB_BUILD_BINARIES OFF CACHE BOOL "" FORCE)
        set(OPENVDB_BUILD_VDB_PRINT OFF CACHE BOOL "" FORCE)
        set(USE_EXPLICIT_INSTANTIATION OFF CACHE BOOL "" FORCE)
        add_subdirectory(${MESHLIB_THIRDPARTY_DIR}/openvdb/v10/openvdb ${CMAKE_BINARY_DIR}/openvdb)
        
        # Mark OpenVDB as found and create necessary variables for FindOpenVDB
        set(OpenVDB_FOUND TRUE CACHE BOOL "" FORCE)
        set(OpenVDB_VERSION "10.1.0" CACHE STRING "" FORCE)
        set(OpenVDB_INCLUDE_DIR "${MESHLIB_THIRDPARTY_DIR}/openvdb/v10/openvdb/openvdb" CACHE PATH "" FORCE)
        if(NOT TARGET OpenVDB::openvdb)
            add_library(OpenVDB::openvdb ALIAS openvdb)
        endif()
    ENDIF()
ENDIF()

# ============================================================================
# Subdirectories
# ============================================================================

# Set GTest requirement for MRMesh
IF(NOT MESHLIB_BUILD_TESTS)
    set(MRMESH_NO_GTEST ON CACHE BOOL "Disable GTest in MRMesh" FORCE)
ENDIF()

# Always build precompiled headers and core mesh library
add_subdirectory(source/MRPch)
add_subdirectory(source/MRMesh)

# Optional: C API bindings
IF(MESHLIB_BUILD_C_BINDING)
    add_subdirectory(source/MRMeshC)
ENDIF()

# Optional: Voxel operations (OpenVDB)
IF(MESHLIB_BUILD_VOXELS)
    add_subdirectory(source/MRVoxels)
ENDIF()

# Optional: Extra I/O formats
IF(MESHLIB_BUILD_IOEXTRAS)
    # Pass format options to MRIOExtras
    IF(NOT MESHLIB_IO_3MF)
        set(MRIOEXTRAS_NO_3MF ON CACHE BOOL "" FORCE)
    ENDIF()
    IF(NOT MESHLIB_IO_E57)
        set(MRIOEXTRAS_NO_E57 ON CACHE BOOL "" FORCE)
    ENDIF()
    IF(NOT MESHLIB_IO_GLTF)
        set(MRIOEXTRAS_NO_GLTF ON CACHE BOOL "" FORCE)
    ENDIF()
    IF(NOT MESHLIB_IO_LAS)
        set(MRIOEXTRAS_NO_LAS ON CACHE BOOL "" FORCE)
    ENDIF()
    IF(NOT MESHLIB_IO_STEP)
        set(MRIOEXTRAS_NO_STEP ON CACHE BOOL "" FORCE)
    ENDIF()
    IF(NOT MESHLIB_IO_PDF)
        set(MRIOEXTRAS_NO_PDF ON CACHE BOOL "" FORCE)
    ENDIF()
    add_subdirectory(source/MRIOExtras)
ENDIF()

# Optional: Symbol to mesh conversion
IF(MESHLIB_BUILD_SYMBOLMESH)
    add_subdirectory(source/MRSymbolMesh)
ENDIF()

# Optional: Python bindings
IF(MESHLIB_BUILD_PYTHON AND NOT EMSCRIPTEN)
    # Configure pybind11 non-limited API settings (for multi-version Python support)
    # This is how MeshLib achieves single-binary compatibility across Python versions
    set(PYBIND11_NONLIMITEDAPI_PYTHON_MIN_VERSION_HEX 0x030800f0 CACHE STRING "") # Python 3.8
    set(PYBIND11_NONLIMITEDAPI_SUFFIX "meshlib_standalone" CACHE STRING "")
    set(PYBIND11_NONLIMITEDAPI_LIBRARY_OUTPUT_DIR "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/meshlib" CACHE PATH "")
    set(PYBIND11_NONLIMITEDAPI_INSTALL_LIBDIR_STUBS "${MR_MAIN_LIB_DIR}" CACHE PATH "")
    set(PYBIND11_NONLIMITEDAPI_INSTALL_LIBDIR_SHIMS "${MR_PY_LIB_DIR}" CACHE PATH "")
    option(PYBIND11_NONLIMITEDAPI_BUILD_STUBS "" ON)
    # Disable exports to avoid CMake errors with build-directory paths
    option(PYBIND11_NONLIMITEDAPI_INSTALL_EXPORTS "" OFF)
    
    # Build the pybind11 non-limited API stubs and shims from standalone's bundled fork
    add_subdirectory(${CMAKE_SOURCE_DIR}/thirdparty/mrbind-pybind11/source/non_limited_api ./pybind11_nonlimitedapi)
    
    # Now build our Python bindings
    add_subdirectory(source/MRPythonBindings)
ENDIF()

# Optional: Tests
IF(MESHLIB_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
ENDIF()

# ============================================================================
# Install Configuration
# ============================================================================

include(CMakePackageConfigHelpers)

configure_package_config_file(
    ${CMAKE_SOURCE_DIR}/cmake/meshlib-standalone-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/meshlib-standalone-config.cmake
    INSTALL_DESTINATION ${MR_CONFIG_DIR}
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/meshlib-standalone-config-version.cmake
    VERSION ${MESHLIB_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/meshlib-standalone-config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/meshlib-standalone-config-version.cmake
    DESTINATION ${MR_CONFIG_DIR}
)

# ============================================================================
# Summary
# ============================================================================

message(STATUS "")
message(STATUS "=== MeshLib Standalone Configuration ===")
message(STATUS "Version:        ${MESHLIB_VERSION}")
message(STATUS "Build type:     ${CMAKE_BUILD_TYPE}")
message(STATUS "Voxels:         ${MESHLIB_BUILD_VOXELS}")
message(STATUS "I/O Extras:     ${MESHLIB_BUILD_IOEXTRAS}")
message(STATUS "C Binding:      ${MESHLIB_BUILD_C_BINDING}")
message(STATUS "Python:         ${MESHLIB_BUILD_PYTHON}")
message(STATUS "Tests:          ${MESHLIB_BUILD_TESTS}")
message(STATUS "Emscripten:     ${MR_EMSCRIPTEN}")
message(STATUS "=========================================")
message(STATUS "")
