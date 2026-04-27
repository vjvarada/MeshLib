@echo off
setlocal enabledelayedexpansion

echo ========================================
echo Generating Python Bindings for Standalone  
echo ========================================

REM Set paths
set ROOT_DIR=%~dp0..\..\..
set STANDALONE_DIR=%~dp0..\..
set MRBIND_EXE=%ROOT_DIR%\thirdparty\mrbind\build\mrbind.exe
set SOURCE_DIR=%STANDALONE_DIR%\source
set TEMP_DIR=%STANDALONE_DIR%\temp_bindings
set OUTPUT_JSON=%TEMP_DIR%\bindings_all.cpp

echo DEBUG: MRBIND_EXE = %MRBIND_EXE%
if not exist "%MRBIND_EXE%" (
    echo ERROR: mrbind.exe not found at %MRBIND_EXE%
    exit /b 1
)

REM Ensure temp directory exists
if not exist "%TEMP_DIR%" mkdir "%TEMP_DIR%"

REM Set paths for includes
set MSVC_VER=14.39.33519
set SDK_VER=10.0.22621.0

REM Compiler flags for parsing (based on main MeshLib's generate.mk)
set CLANG_FLAGS=-x c++ -std=c++23
set CLANG_FLAGS=%CLANG_FLAGS% -fparse-all-comments
set CLANG_FLAGS=%CLANG_FLAGS% -fms-compatibility
set CLANG_FLAGS=%CLANG_FLAGS% -fms-extensions
set CLANG_FLAGS=%CLANG_FLAGS% -frelaxed-template-template-args
set CLANG_FLAGS=%CLANG_FLAGS% -Wno-nonportable-include-path
set CLANG_FLAGS=%CLANG_FLAGS% -Wno-enum-constexpr-conversion
set CLANG_FLAGS=%CLANG_FLAGS% -Wno-deprecated-enum-enum-conversion
set CLANG_FLAGS=%CLANG_FLAGS% -Wno-deprecated-declarations
set CLANG_FLAGS=%CLANG_FLAGS% -Wno-implicitly-unsigned-literal
set CLANG_FLAGS=%CLANG_FLAGS% -ferror-limit=0
REM Force use of Clang's builtin offsetof to avoid constexpr errors
set CLANG_FLAGS=%CLANG_FLAGS% -Doffsetof(s,m)=__builtin_offsetof(s,m)
set CLANG_FLAGS=%CLANG_FLAGS% -DMR_PARSING_FOR_PB11_BINDINGS
set CLANG_FLAGS=%CLANG_FLAGS% -DMR_PARSING_FOR_ANY_BINDINGS
set CLANG_FLAGS=%CLANG_FLAGS% -DMB_FUNC=
set CLANG_FLAGS=%CLANG_FLAGS% -DMB_CLASS=
set CLANG_FLAGS=%CLANG_FLAGS% -DMB_ENUM=
set CLANG_FLAGS=%CLANG_FLAGS% -DNOMINMAX
set CLANG_FLAGS=%CLANG_FLAGS% -DMESHLIB_NO_VIEWER
set CLANG_FLAGS=%CLANG_FLAGS% -D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH
set CLANG_FLAGS=%CLANG_FLAGS% -D_SILENCE_CXX17_CODECVT_HEADER_DEPRECATION_WARNING
set CLANG_FLAGS=%CLANG_FLAGS% -D_SILENCE_CXX20_OLD_SHARED_PTR_ATOMIC_SUPPORT_DEPRECATION_WARNING
set CLANG_FLAGS=%CLANG_FLAGS% -D_SILENCE_CXX23_ALIGNED_STORAGE_DEPRECATION_WARNING
set CLANG_FLAGS=%CLANG_FLAGS% -I%STANDALONE_DIR%\source
set CLANG_FLAGS=%CLANG_FLAGS% -isystem %STANDALONE_DIR%\thirdparty\parallel-hashmap
set CLANG_FLAGS=%CLANG_FLAGS% -isystem %STANDALONE_DIR%\build\vcpkg_installed\x64-windows\include
set CLANG_FLAGS=%CLANG_FLAGS% -isystem %STANDALONE_DIR%\build\vcpkg_installed\x64-windows\include\eigen3
set CLANG_FLAGS=%CLANG_FLAGS% -isystem %STANDALONE_DIR%\thirdparty\expected\include
set CLANG_FLAGS=%CLANG_FLAGS% -isystem %STANDALONE_DIR%\thirdparty\mrbind-pybind11\include
set CLANG_FLAGS=%CLANG_FLAGS% -I%ROOT_DIR%\thirdparty\mrbind\include
set CLANG_FLAGS=%CLANG_FLAGS% -I"C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Tools\MSVC\%MSVC_VER%\include"
set CLANG_FLAGS=%CLANG_FLAGS% -I"C:\Program Files (x86)\Windows Kits\10\Include\%SDK_VER%\ucrt"
set CLANG_FLAGS=%CLANG_FLAGS% -I"C:\Program Files (x86)\Windows Kits\10\Include\%SDK_VER%\um"
set CLANG_FLAGS=%CLANG_FLAGS% -I"C:\Program Files (x86)\Windows Kits\10\Include\%SDK_VER%\shared"
set CLANG_FLAGS=%CLANG_FLAGS% --target=x86_64-pc-windows-msvc

REM Set resource directory for constexpr-friendly offsetof (critical for libclang)
for /f "delims=" %%i in ('C:\msys64_meshlib_mrbind\clang64\bin\clang++.exe -print-resource-dir') do set RESOURCE_DIR=%%i
set CLANG_FLAGS=%CLANG_FLAGS% -resource-dir=%RESOURCE_DIR%

REM Create a combined header file that includes all MRMesh headers
set COMBINED_HEADER=%TEMP_DIR%\all_headers.h
echo // Combined header for mrbind - includes all MRMesh headers > %COMBINED_HEADER%
echo #pragma once >> %COMBINED_HEADER%
echo // CRITICAL: Fix offsetof BEFORE any system headers >> %COMBINED_HEADER%
echo #include ^<stddef.h^> >> %COMBINED_HEADER%
echo #undef offsetof >> %COMBINED_HEADER%
echo #define offsetof(s,m) __builtin_offsetof(s,m) >> %COMBINED_HEADER%
echo #include ^<iostream^> >> %COMBINED_HEADER%
echo. >> %COMBINED_HEADER%

REM Include ALL headers from MRMesh (like main MeshLib does)
REM Exclude test and viewer-specific headers
echo Scanning for headers in %SOURCE_DIR%\MRMesh...
for %%H in ("%SOURCE_DIR%\MRMesh\MR*.h") do (
    REM Skip test, viewer, and CUDA headers
    echo %%~nxH | findstr /I "GTest Viewer ImGui GL CUDA" >nul
    if errorlevel 1 (
        echo #include "MRMesh/%%~nxH" >> %COMBINED_HEADER%
    )
)

echo. >> %COMBINED_HEADER%
REM Skip voxels for now - they require OpenVDB which is not installed
REM echo // Include voxels headers too >> %COMBINED_HEADER%
REM for %%%%H in ("%SOURCE_DIR%\MRVoxels\MR*.h") do (
REM     echo #include "MRVoxels/%%%%~nxH" >> %COMBINED_HEADER%
REM )

echo.
echo Combined header created at: %COMBINED_HEADER%
echo.
echo Running mrbind parser...
echo.

REM Set PATH to include MSYS2 DLLs
set PATH=C:\msys64_meshlib_mrbind\clang64\bin;%PATH%

REM Mrbind flags (based on main MeshLib's mrbind_flags.txt)
set MRBIND_FLAGS=--format=macros
set MRBIND_FLAGS=%MRBIND_FLAGS% --combine-types=cv,ref,ptr,smart_ptr
set MRBIND_FLAGS=%MRBIND_FLAGS% --buggy-substitute-default-template-args
set MRBIND_FLAGS=%MRBIND_FLAGS% --canonicalize-64-to-fixed-size-typedefs
set MRBIND_FLAGS=%MRBIND_FLAGS% --canonicalize-size_t-to-uint64_t
set MRBIND_FLAGS=%MRBIND_FLAGS% --implicit-enum-underlying-type-is-always-int
set MRBIND_FLAGS=%MRBIND_FLAGS% --copy-inherited-members
set MRBIND_FLAGS=%MRBIND_FLAGS% --ignore ::
set MRBIND_FLAGS=%MRBIND_FLAGS% --allow MR
set MRBIND_FLAGS=%MRBIND_FLAGS% --ignore MR::detail
set MRBIND_FLAGS=%MRBIND_FLAGS% --ignore MR::Signal
set MRBIND_FLAGS=%MRBIND_FLAGS% --ignore MR::UniquePtr
set MRBIND_FLAGS=%MRBIND_FLAGS% --ignore MR::Config
set MRBIND_FLAGS=%MRBIND_FLAGS% --ignore "/.*operator<=>.*/"
REM Skip free operators for BitSet types that generate bad bindings  
REM These operators use TypedBitSet template which doesn't work with MB_FUNC macro
set MRBIND_FLAGS=%MRBIND_FLAGS% --ignore "/MR::operator&.*BitSet.*/"
set MRBIND_FLAGS=%MRBIND_FLAGS% --ignore "/MR::operator\|.*BitSet.*/"
set MRBIND_FLAGS=%MRBIND_FLAGS% --ignore "/MR::operator\^.*BitSet.*/"
set MRBIND_FLAGS=%MRBIND_FLAGS% --ignore "/MR::operator-.*BitSet.*/"
set MRBIND_FLAGS=%MRBIND_FLAGS% --skip-mentions-of "/std::integral_constant<.*>/"
set MRBIND_FLAGS=%MRBIND_FLAGS% --skip-mentions-of "/boost::dynamic_bitset<.*>/"
set MRBIND_FLAGS=%MRBIND_FLAGS% --skip-mentions-of "/Eigen::Matrix<.*>/"
set MRBIND_FLAGS=%MRBIND_FLAGS% --skip-mentions-of "/MR::Signal<.*>/"
set MRBIND_FLAGS=%MRBIND_FLAGS% --skip-mentions-of "/std::initializer_list<.*>/"
set MRBIND_FLAGS=%MRBIND_FLAGS% --skip-mentions-of Json::Value
set MRBIND_FLAGS=%MRBIND_FLAGS% --skip-mentions-of "/(std|tl)::expected<Json::Value, .*>/"

REM Run mrbind with proper flags
"%MRBIND_EXE%" %MRBIND_FLAGS% -o %OUTPUT_JSON% %COMBINED_HEADER% -- %CLANG_FLAGS%

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ERROR: mrbind failed with error code %ERRORLEVEL%
    exit /b %ERRORLEVEL%
)

echo.
echo ========================================
echo Success! Bindings generated at:
echo %OUTPUT_JSON%
echo ========================================
echo.
echo Next steps:
echo 1. Review the generated JSON
echo 2. Create pybind11 wrapper code
echo 3. Update CMakeLists.txt to compile bindings
echo.

endlocal
