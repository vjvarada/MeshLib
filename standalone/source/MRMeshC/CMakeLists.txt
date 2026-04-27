project(MRMeshC CXX)

# Collect all source files
file(GLOB SOURCES "*.cpp")
file(GLOB HEADERS "*.h")
file(GLOB DETAIL_HEADERS "detail/*.h" "detail/*.hpp")

# Create the C API library
add_library(${PROJECT_NAME} SHARED ${SOURCES} ${HEADERS} ${DETAIL_HEADERS})

# Compiler options
if(NOT MSVC)
    target_compile_options(${PROJECT_NAME} PRIVATE "-Wno-strict-aliasing")
endif()

# Include directories
target_include_directories(${PROJECT_NAME} 
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
        $<INSTALL_INTERFACE:include/MRMeshC>
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../MRMesh
        ${CMAKE_CURRENT_SOURCE_DIR}/../MRPch
)

# Link libraries
target_link_libraries(${PROJECT_NAME} 
    PUBLIC
        MRMesh
)

# Link MRVoxels if available
if(TARGET MRVoxels)
    target_link_libraries(${PROJECT_NAME} PRIVATE MRVoxels)
endif()

# Link MRIOExtras if available  
if(TARGET MRIOExtras)
    target_link_libraries(${PROJECT_NAME} PRIVATE MRIOExtras)
endif()

# Export symbols for DLL
if(WIN32)
    target_compile_definitions(${PROJECT_NAME} PRIVATE MRMESHC_EXPORTS)
endif()

# Set output directories
set_target_properties(${PROJECT_NAME} PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
)

# Install
install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES ${HEADERS} DESTINATION include/MRMeshC)
install(FILES ${DETAIL_HEADERS} DESTINATION include/MRMeshC/detail)

# PCH support
if(TARGET MRPch)
    target_precompile_headers(${PROJECT_NAME} REUSE_FROM MRPch)
endif()
