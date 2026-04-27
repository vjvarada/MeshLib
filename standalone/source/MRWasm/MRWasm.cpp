/**
 * MeshLib WebAssembly Entry Point
 * 
 * This file provides the JavaScript-friendly interface to MRMeshC.
 * All MRMeshC functions are automatically exported to WASM via Emscripten.
 */

#include "MRMeshC/MRMeshC.h"

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <emscripten/bind.h>

// Module initialization
EMSCRIPTEN_KEEPALIVE
extern "C" int meshlib_init() {
    return 1;  // Success
}

// Version info
EMSCRIPTEN_KEEPALIVE
extern "C" const char* meshlib_version() {
    return "MeshLib Standalone WASM 1.0";
}

// Memory management helpers
EMSCRIPTEN_KEEPALIVE
extern "C" void* meshlib_alloc(size_t size) {
    return malloc(size);
}

EMSCRIPTEN_KEEPALIVE  
extern "C" void meshlib_free(void* ptr) {
    free(ptr);
}

#endif // __EMSCRIPTEN__
