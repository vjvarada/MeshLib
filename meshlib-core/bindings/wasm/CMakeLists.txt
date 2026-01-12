# WebAssembly bindings for MeshLib Core

add_executable(meshlib_wasm
    bindings.cpp
)

target_link_libraries(meshlib_wasm PRIVATE meshlib_core)

# Emscripten-specific settings
set_target_properties(meshlib_wasm PROPERTIES
    OUTPUT_NAME "meshlib"
    SUFFIX ".js"
)

# Emscripten linker flags
target_link_options(meshlib_wasm PRIVATE
    # Module settings
    "-sEXPORTED_RUNTIME_METHODS=['ccall','cwrap','getValue','setValue','UTF8ToString','stringToUTF8']"
    "-sMODULARIZE=1"
    "-sEXPORT_NAME='MeshLibCore'"
    "-sENVIRONMENT='web,worker,node'"
    
    # Memory settings
    "-sALLOW_MEMORY_GROWTH=1"
    "-sINITIAL_MEMORY=64MB"
    "-sMAXIMUM_MEMORY=2GB"
    "-sSTACK_SIZE=1MB"
    
    # WebGL (if needed for visualization)
    # "-sUSE_WEBGL2=1"
    # "-sFULL_ES3=1"
    
    # Embind for C++ bindings
    "--bind"
    
    # Optimize
    "-O3"
    "-flto"
    
    # Generate TypeScript definitions
    "--emit-tsd=meshlib.d.ts"
)

# Copy additional files to output
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/meshlib.d.ts
    ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/meshlib.d.ts
    COPYONLY
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/three_integration.ts
    ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/three_integration.ts
    COPYONLY
)

# Install WASM output
install(FILES
    ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/meshlib.js
    ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/meshlib.wasm
    ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/meshlib.d.ts
    ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/three_integration.ts
    DESTINATION share/meshlib_core/wasm
)
