@echo off
setlocal enabledelayedexpansion

REM Setup VS environment
call "C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64

REM Set variables
set MSYS2_DIR=C:\msys64_meshlib_mrbind
set MRBIND_SRC=%~dp0thirdparty\mrbind

echo Building MRBind in MSYS2 with VS environment...
echo MRBIND_SRC=%MRBIND_SRC%
echo VCToolsInstallDir=%VCToolsInstallDir%

REM Clean old build
if exist "%MRBIND_SRC%\build" rmdir /S /Q "%MRBIND_SRC%\build"

REM Build in MSYS2
call %MSYS2_DIR%\msys2_shell.cmd -no-start -defterm -full-path -here -clang64 -where "%MRBIND_SRC%" -c "pwd && cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo 2>&1 && cmake --build build -j4 2>&1 && ls -la build/mrbind*"

echo Build complete.
endlocal
