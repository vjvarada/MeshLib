@echo off
setlocal enabledelayedexpansion

echo ========================================
echo Compiling MRBind-Generated Python Bindings
echo ========================================

set ROOT_DIR=%~dp0..\..\..
set STANDALONE_DIR=%~dp0..\..
set TEMP_DIR=%STANDALONE_DIR%\temp_bindings
set BUILD_DIR=%STANDALONE_DIR%\build\bin
set MRBIND_PYBIND11_DIR=%STANDALONE_DIR%\thirdparty\mrbind-pybind11
set VCPKG_DIR=%STANDALONE_DIR%\build\vcpkg_installed\x64-windows

set SOURCE_FILE=%TEMP_DIR%\bindings_all.cpp

if not exist "%SOURCE_FILE%" (
    echo ERROR: bindings_all.cpp not found. Run generate_bindings.bat first.
    exit /b 1
)

if not exist "%BUILD_DIR%" (
    echo ERROR: MeshLib build directory not found at %BUILD_DIR%
    exit /b 1
)

set MSVC_VER=14.39.33519
set SDK_VER=10.0.22621.0
set MSVC_DIR=C:\Program Files\Microsoft Visual Studio\2022\Professional\VC\Tools\MSVC\%MSVC_VER%
set SDK_DIR=C:\Program Files (x86)\Windows Kits\10\Include\%SDK_VER%

REM Get Python paths
for /f "delims=" %%i in ('python -c "import sysconfig; print(sysconfig.get_path('include'))"') do set PYTHON_INCLUDE=%%i
for /f "delims=" %%i in ('python -c "import sysconfig; print(sysconfig.get_config_var('LIBDIR') or sysconfig.get_path('stdlib').replace('Lib','libs'))"') do set PYTHON_LIB_DIR=%%i
for /f "delims=" %%i in ('python -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"') do set PYTHON_VER=%%i
for /f "delims=" %%i in ('python -c "import sys; print(f'{sys.version_info.major}{sys.version_info.minor}')"') do set PYTHON_VER_SHORT=%%i

echo Python Version: %PYTHON_VER%
echo Python Include: %PYTHON_INCLUDE%
echo Python Lib Dir: %PYTHON_LIB_DIR%

REM Clang resource directory for internal headers
for /f "delims=" %%i in ('C:\msys64_meshlib_mrbind\clang64\bin\clang++.exe -print-resource-dir') do set RESOURCE_DIR=%%i
echo Clang Resource Dir: %RESOURCE_DIR%

set CXX=C:\msys64_meshlib_mrbind\clang64\bin\clang++.exe

REM Compiler flags - cross-compile to MSVC ABI
set CFLAGS=-std=c++20 -O2 -DNDEBUG
set CFLAGS=%CFLAGS% -fms-compatibility -fms-extensions
set CFLAGS=%CFLAGS% --target=x86_64-pc-windows-msvc
set CFLAGS=%CFLAGS% -resource-dir=%RESOURCE_DIR%
REM Suppress warnings to speed up compilation
set CFLAGS=%CFLAGS% -w
set CFLAGS=%CFLAGS% -D_SILENCE_CXX20_IS_ALWAYS_EQUAL_DEPRECATION_WARNING

REM Python/Pybind flags
set CFLAGS=%CFLAGS% -DPy_LIMITED_API=0x030800f0
set CFLAGS=%CFLAGS% -DPYBIND11_INTERNALS_VERSION=5
set CFLAGS=%CFLAGS% "-DPYBIND11_NONLIMITEDAPI_LIB_SUFFIX_FOR_MODULE=\"meshlib_standalone\""

REM MRBind flags
set CFLAGS=%CFLAGS% -DMB_PB11_MODULE_NAME=mrmeshpy_auto
set CFLAGS=%CFLAGS% "-DMRBIND_HEADER=\"%TEMP_DIR%/mrbind_wrapper.h\""
set CFLAGS=%CFLAGS% -DMB_DEFINE_IMPLEMENTATION
set CFLAGS=%CFLAGS% -DMB_INCLUDE_ORIGINAL_HEADER=0

REM Windows defines
set CFLAGS=%CFLAGS% -D_DLL -D_MT -DNOMINMAX
set CFLAGS=%CFLAGS% -D_SILENCE_ALL_CXX23_DEPRECATION_WARNINGS
set CFLAGS=%CFLAGS% -D_ALLOW_COMPILER_AND_STL_VERSION_MISMATCH
set CFLAGS=%CFLAGS% -DPYBIND11_EXPORT_EXCEPTION=
set CFLAGS=%CFLAGS% -Xclang --dependent-lib=msvcrt

REM Include directories
set INCLUDES=-I"%PYTHON_INCLUDE%"
set INCLUDES=%INCLUDES% -I"%TEMP_DIR%"
set INCLUDES=%INCLUDES% -I"%MRBIND_PYBIND11_DIR%\include"
set INCLUDES=%INCLUDES% -I"%ROOT_DIR%\thirdparty\mrbind\include"
set INCLUDES=%INCLUDES% -I"%STANDALONE_DIR%\source"
set INCLUDES=%INCLUDES% -I"%VCPKG_DIR%\include"
set INCLUDES=%INCLUDES% -I"%VCPKG_DIR%\include\eigen3"
set INCLUDES=%INCLUDES% -isystem "%STANDALONE_DIR%\thirdparty\parallel-hashmap"
set INCLUDES=%INCLUDES% -isystem "%STANDALONE_DIR%\thirdparty\expected\include"
set INCLUDES=%INCLUDES% -I"%MSVC_DIR%\include"
set INCLUDES=%INCLUDES% -I"%SDK_DIR%\ucrt"
set INCLUDES=%INCLUDES% -I"%SDK_DIR%\um"
set INCLUDES=%INCLUDES% -I"%SDK_DIR%\shared"

REM Linker flags
set LDFLAGS=--target=x86_64-pc-windows-msvc
set LDFLAGS=%LDFLAGS% -rtlib=platform
set LDFLAGS=%LDFLAGS% -Wl,-noimplib
set LDFLAGS=%LDFLAGS% -L"%PYTHON_LIB_DIR%" -lpython%PYTHON_VER_SHORT%
set LDFLAGS=%LDFLAGS% -L"%BUILD_DIR%" -lMRMesh -lMRIOExtras
set LDFLAGS=%LDFLAGS% -L"%BUILD_DIR%" -lpybind11nonlimitedapi_stubs

set OUTPUT_FILE=%BUILD_DIR%\mrmeshpy_auto.pyd

echo.
echo Compiling %SOURCE_FILE%...
echo This may take a while (the file is 7+ MB)...
echo.

%CXX% %CFLAGS% %INCLUDES% -shared %SOURCE_FILE% -o %OUTPUT_FILE% %LDFLAGS%

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo ERROR: Compilation failed with error code %ERRORLEVEL%
    exit /b %ERRORLEVEL%
)

echo.
echo ========================================
echo Success! Auto-generated module created:
echo %OUTPUT_FILE%
echo ========================================
echo.
echo Test with:
echo   cd %BUILD_DIR%
echo   python -c "import mrmeshpy_auto; print(dir(mrmeshpy_auto)[:10])"
echo.

endlocal
