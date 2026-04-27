project(MRWasm CXX)

# Only build for Emscripten
if(NOT EMSCRIPTEN)
    message(STATUS "MRWasm: Skipping (not Emscripten build)")
    return()
endif()

message(STATUS "MRWasm: Building WebAssembly module")

# Collect source files  
file(GLOB SOURCES "*.cpp")
file(GLOB HEADERS "*.h")

# Create WASM module - this wraps MRMeshC for JavaScript
add_executable(meshlib_wasm ${SOURCES})

# Link to the C API
target_link_libraries(meshlib_wasm PRIVATE
    MRMeshC
    MRMesh
)

# Link MRVoxels if available
if(TARGET MRVoxels)
    target_link_libraries(meshlib_wasm PRIVATE MRVoxels)
endif()

# Emscripten-specific settings
set_target_properties(meshlib_wasm PROPERTIES
    SUFFIX ".js"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/wasm"
)

# Emscripten link flags
target_link_options(meshlib_wasm PRIVATE
    # Export all MRMeshC functions
    "-sEXPORTED_FUNCTIONS=['_malloc','_free']"
    "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','FS','HEAPF32','HEAPU8','HEAPU32']"
    # Module settings
    "-sMODULARIZE=1"
    "-sEXPORT_NAME='MeshLib'"
    "-sALLOW_MEMORY_GROWTH=1"
    "-sSTACK_SIZE=5MB"
    # File system support
    "-sFORCE_FILESYSTEM=1"
    # Threading (if multi-threaded build)
    "$<$<BOOL:${MR_EMSCRIPTEN_SINGLE}>:-sENVIRONMENT=web>"
    "$<$<NOT:$<BOOL:${MR_EMSCRIPTEN_SINGLE}>>:-sENVIRONMENT=web,worker>"
    "$<$<NOT:$<BOOL:${MR_EMSCRIPTEN_SINGLE}>>:-sPTHREAD_POOL_SIZE=4>"
)

# Generate TypeScript definitions
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/meshlib.d.ts.in
    ${CMAKE_BINARY_DIR}/wasm/meshlib.d.ts
    @ONLY
)

# Copy WASM files to output
add_custom_command(TARGET meshlib_wasm POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_BINARY_DIR}/wasm/meshlib_wasm.wasm
        ${CMAKE_BINARY_DIR}/wasm/meshlib.wasm
    COMMENT "Copying WASM files"
)
