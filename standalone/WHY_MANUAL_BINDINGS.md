# Why Standalone Uses Manual Python Bindings Instead of mrbind

## TL;DR - The Short Answer

**We CAN use automatic binding generation (mrbind) in standalone, but we deliberately chose NOT to for simplicity and practicality.**

The standalone uses **manual pybind11 bindings** instead of the **automatic mrbind** code generation that main MeshLib uses. **This is a strategic design decision, not a technical limitation.**

---

## What is mrbind?

**mrbind** is MeshLib's custom tool that automatically generates Python and C# bindings from C++ headers.

**How it works:**
1. Parser uses libclang to analyze C++ headers
2. Generates intermediate representation (JSON or macros)
3. Code generator creates pybind11 binding code
4. Result: Complete Python bindings without manual coding

Main MeshLib location: `thirdparty/mrbind/` (parser) + `scripts/mrbind/` (build scripts)

**Build process:**
- Run mrbind parser on C++ headers
- Generate pybind11 binding code automatically  
- Compile generated code into Python modules
- **Result**: ALL C++ APIs exposed to Python automatically

---

## Why Standalone Doesn't Use mrbind

### 1. Simplified Build System

**mrbind requires:**
- libclang 18+ libraries
- Matching Clang compiler version
- LLVM development libraries
- Complex Makefile + CMake orchestration
- Multi-stage build (parse, generate, compile)

**Manual bindings require:**
- Any standard C++ compiler (MSVC/GCC/Clang)
- pybind11 headers (already bundled)
- Simple CMake-only build
- Single-stage build

### 2. Reduced Dependencies

| Main MeshLib (mrbind) | Standalone (manual) |
|----------------------|---------------------|
| Python 3.x | Python 3.x |
| pybind11 fork | pybind11 fork (bundled) |
| libclang 18-21 | - |
| LLVM libraries | - |
| Clang compiler | Any C++ compiler |
| CMake + Make | CMake only |

### 3. Build Time

| Approach | Time |
|----------|------|
| With mrbind | Parse (5min) + Generate (1min) + Compile (5min) = **10-20 min** |
| Manual | Compile only = **2-5 min** |

### 4. Debugging & Control

**Manual bindings:**
- See exactly what's being bound
- Easy to step through in debugger
- Add custom type conversions easily
- Full control over Python API design
- Clear, readable binding code

**Generated bindings:**
- Opaque generated code
- Harder to debug
- Custom behavior needs header annotations
- Must re-run parser for changes

### 5. Project Philosophy

**Standalone goals:**
- Simplicity - easy to build anywhere
- Minimal dependencies - standard tools only
- Clear code - anyone can understand
- Focused API - only what's needed
- Easy maintenance - no special tools

**Main MeshLib goals:**
- Complete API coverage - everything exposed
- Multi-language - Python, C#, etc.
- Consistency - guaranteed C++/Python parity
- Large scale - thousands of functions

---

## Could Standalone Use mrbind? YES!

It's **technically feasible** and would take only a few hours to set up. Here's how:

### Quick Setup Guide

```powershell
# 1. Install MSYS2 CLANG64 environment
pacman -S mingw-w64-clang-x86_64-clang
pacman -S mingw-w64-clang-x86_64-cmake
pacman -S mingw-w64-clang-x86_64-llvm

# 2. Build mrbind
cd thirdparty/mrbind
cmake -B build -G "MSYS Makefiles"
cmake --build build -j8

# 3. Configure for standalone
# Create scripts/mrbind/standalone_flags.txt
# Point to standalone headers
# Run: make -f scripts/mrbind/generate.mk

# 4. Modify CMakeLists.txt to use generated code
```

**Result:** Automatic API parity with main MeshLib, zero manual binding maintenance.

---

## Why We Still Choose Manual

Despite feasibility, we intentionally use manual bindings:

### 1. Proportionality

- **Main MeshLib**: ~500 classes, ~5000 functions  Automation **essential**
- **Standalone**: ~20 classes, ~100 functions  Manual is **manageable**

### 2. User Experience

Building standalone:
- **With mrbind**: "Install MSYS2, install Clang 21, install LLVM, run make, then cmake..."
- **Without**: "Just cmake and build" 

### 3. Maintenance Reality

- Adding one function manually: **2 minutes**
- Setting up mrbind: **4+ hours initial + complexity forever**

For standalone's small API, manual wins.

### 4. Educational Value

Manual bindings serve as:
- **Documentation** of C++ API usage
- **Learning resource** for pybind11 patterns
- **Reference** for users creating their own bindings
- **Template** showing best practices

---

## Current Strategy: Hybrid Approach

1. **Manual bindings** (current) - fast, simple, clear
2. **Add as needed** - document in API_PARITY.md, add when users request
3. **Future option** - can switch to mrbind if API grows large

Each missing function takes ~2 minutes to add manually. We add them when users encounter missing APIs.

---

## How to Add Missing APIs (Manual Approach)

See [API_PARITY.md](API_PARITY.md) for detailed guide.

**Quick example:**

```cpp
// 1. Find C++ function (e.g., MRMeshFixer.h)
Expected<FaceBitSet> findDegenerateFaces(const Mesh&, float);

// 2. Add binding (MRMeshOperationsBindings.cpp)
#include "MRMesh/MRMeshFixer.h"

m.def("findDegenerateFaces", [](const Mesh& mesh, float ratio) {
    auto result = findDegenerateFaces(mesh, ratio);
    if (!result.has_value())
        throw std::runtime_error(result.error());
    return result.value();
}, py::arg("mesh"), py::arg("criticalAspectRatio") = FLT_MAX);

// 3. Rebuild
cmake --build standalone/build --config Release --target mrmeshpy
```

Time: 2 minutes per function.

---

## Decision Matrix

| Criterion | Manual | mrbind |
|-----------|--------|--------|
| Initial setup |  10 min |  4+ hours |
| Dependencies |  Minimal |  Heavy |
| Build time |  Fast (2-5min) |  Slow (10-20min) |
| Debugging |  Easy |  Harder |
| Completeness |  Partial |  100% |
| Maintenance |  Manual |  Auto |
| Code clarity |  Clear |  Generated |
| Customization |  Full control |  Via annotations |
| Portability |  Any compiler |  Needs Clang |
| Documentation |  High value |  Low value |

**Score: Manual 8-2 for standalone's use case**

---

## Conclusion

**We don't use mrbind because we deliberately chose not to, not because it's impossible.**

**Decision based on:**
1. **Simplicity** - easier for users to build
2. **Pragmatism** - manual sufficient for small API
3. **Accessibility** - no special tools needed
4. **Clarity** - readable code anyone can modify

**This is architecture philosophy, not technical limitation.**

If standalone API grows significantly or multi-language support becomes important, we can switch to mrbind in a few hours. The infrastructure exists in the main MeshLib repository.

For now, manual bindings provide the **best balance** of simplicity, clarity, and maintainability for standalone's focused scope.

---

## Related Documentation

- [API_PARITY.md](API_PARITY.md) - How to add missing APIs manually
- [../thirdparty/mrbind/README.md](../thirdparty/mrbind/README.md) - Full mrbind docs
- [pybind11 docs](https://pybind11.readthedocs.io/) - Manual binding reference
