# Python bindings for MeshLib Core

# Find pybind11
find_package(pybind11 QUIET)
if(NOT pybind11_FOUND)
    # Try to use bundled pybind11
    if(EXISTS ${CMAKE_SOURCE_DIR}/thirdparty/pybind11)
        add_subdirectory(${CMAKE_SOURCE_DIR}/thirdparty/pybind11 pybind11 EXCLUDE_FROM_ALL)
    else()
        message(FATAL_ERROR "pybind11 not found. Please install it or provide it in thirdparty/")
    endif()
endif()

pybind11_add_module(meshlib_python MODULE
    bindings.cpp
)

target_link_libraries(meshlib_python PRIVATE meshlib_core)

# Set output name
set_target_properties(meshlib_python PROPERTIES
    OUTPUT_NAME "meshlib_core"
    PREFIX ""
)

# Platform-specific suffix
if(WIN32)
    set_target_properties(meshlib_python PROPERTIES SUFFIX ".pyd")
endif()

# Install Python module
install(TARGETS meshlib_python
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}/python
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}/python
)

# Install __init__.py
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
    ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/meshlib_core/__init__.py
    COPYONLY
)

install(FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/__init__.py
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/python/meshlib_core
)

