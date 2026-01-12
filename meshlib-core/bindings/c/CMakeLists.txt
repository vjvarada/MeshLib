# C bindings for MeshLib Core

add_library(meshlib_c SHARED
    meshlib_c.cpp
)

target_link_libraries(meshlib_c PRIVATE meshlib_core)

target_include_directories(meshlib_c PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

# Install
install(TARGETS meshlib_c
    EXPORT meshlib_c-targets
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(FILES meshlib_c.h
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT meshlib_c-targets
    FILE meshlib_c-targets.cmake
    NAMESPACE MeshLibCore::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/meshlib_core
)
