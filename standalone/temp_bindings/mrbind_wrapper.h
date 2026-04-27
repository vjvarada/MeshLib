// Wrapper header that ensures complete type definitions before pybind11
#pragma once

// CRITICAL: Fix offsetof BEFORE any system headers
#include <stddef.h>
#undef offsetof
#define offsetof(s,m) __builtin_offsetof(s,m)

// Include all MeshLib headers FIRST - this ensures complete type definitions
// are available when pybind11 headers are processed
#include "C:/Users/VijayRaghavVarada/Documents/Github/MeshLib/standalone/temp_bindings/all_headers.h"

// Now include the original mrbind target header
#include <mrbind/targets/pybind11.h>
