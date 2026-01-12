#pragma once

// Auto-generated configuration header

#define MESHLIB_CORE_VERSION_MAJOR 1
#define MESHLIB_CORE_VERSION_MINOR 0
#define MESHLIB_CORE_VERSION_PATCH 0
#define MESHLIB_CORE_VERSION "1.0.0"

// Feature flags (set by CMake)
/* #undef MESHLIB_HAS_TBB */
/* #undef MESHLIB_HAS_OPENVDB */
/* #undef MESHLIB_CORE_WASM */

// Platform detection
#if defined(__EMSCRIPTEN__)
    #define MESHLIB_PLATFORM_WASM
#elif defined(_WIN32)
    #define MESHLIB_PLATFORM_WINDOWS
#elif defined(__APPLE__)
    #define MESHLIB_PLATFORM_MACOS
#elif defined(__linux__)
    #define MESHLIB_PLATFORM_LINUX
#endif

// Export/Import macros
#if defined(MESHLIB_PLATFORM_WASM)
    #define MESHLIB_API
#elif defined(MESHLIB_PLATFORM_WINDOWS)
    #if defined(MESHLIB_CORE_EXPORTS)
        #define MESHLIB_API __declspec(dllexport)
    #else
        #define MESHLIB_API __declspec(dllimport)
    #endif
#else
    #define MESHLIB_API __attribute__((visibility("default")))
#endif

// Class visibility for vtables (needed on non-Windows)
#if defined(MESHLIB_PLATFORM_WINDOWS)
    #define MESHLIB_CLASS
#else
    #define MESHLIB_CLASS __attribute__((visibility("default")))
#endif

// Inline hints
#if defined(_MSC_VER)
    #define MESHLIB_FORCEINLINE __forceinline
#else
    #define MESHLIB_FORCEINLINE inline __attribute__((always_inline))
#endif

// Threading support
#ifdef MESHLIB_HAS_TBB
    #define MESHLIB_PARALLEL_ENABLED 1
#else
    #define MESHLIB_PARALLEL_ENABLED 0
#endif

// Namespace
#define MESHLIB_NAMESPACE_BEGIN namespace meshlib {
#define MESHLIB_NAMESPACE_END }
