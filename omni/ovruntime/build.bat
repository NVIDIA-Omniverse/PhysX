@echo off
REM SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
REM SPDX-License-Identifier: LicenseRef-NvidiaProprietary

setlocal enabledelayedexpansion

set BUILD_DIR=_build\windows-x86_64
set COMPILER_DIR=_compiler\vc17win64
set INSTALL_DIR=_install\ovruntime
if defined OVRUNTIME_BUILD_BASE set "BUILD_DIR=%OVRUNTIME_BUILD_BASE%"
if defined OVRUNTIME_BUILD_DIR set "BUILD_DIR=%OVRUNTIME_BUILD_DIR%"
if defined OVRUNTIME_COMPILER_DIR set "COMPILER_DIR=%OVRUNTIME_COMPILER_DIR%"
if defined OVRUNTIME_INSTALL_DIR set "INSTALL_DIR=%OVRUNTIME_INSTALL_DIR%"
REM BUILD_TARGET from env var (may be overridden by CLI via parse_build_args)
set "PYTHON_DIR_ARG="
if defined OVRUNTIME_PYTHON_DIR set "PYTHON_DIR_ARG=-DPYTHON_DIR=%OVRUNTIME_PYTHON_DIR%"
set "OUTPUT_BASE=%CD%\%BUILD_DIR%"
if defined OVRUNTIME_OUTPUT_BASE set "OUTPUT_BASE=%OVRUNTIME_OUTPUT_BASE%"
set "OUTPUT_ARGS=-DPX_OUTPUT_LIB_DIR=%OUTPUT_BASE% -DPX_OUTPUT_BIN_DIR=%OUTPUT_BASE%"
set SKIP_INSTALL=0
if defined OVRUNTIME_SKIP_INSTALL set "SKIP_INSTALL=%OVRUNTIME_SKIP_INSTALL%"
set SKIP_VENV=0
if defined OVRUNTIME_SKIP_VENV set "SKIP_VENV=%OVRUNTIME_SKIP_VENV%"

REM Parse unified build flags
call "%~dp0..\cmake\parse_build_args.bat" %*

REM Process project-specific flags from EXTRA_ARGS
set DO_DEV_PHYSX=0
set DO_DEV_SCHEMA=0
set DO_PER_MODULE=0
set DO_SETUP_VENV=0

set "_REMAINING="
for %%a in (!EXTRA_ARGS!) do (
    if /I "%%a"=="--devphysx"   ( set DO_DEV_PHYSX=1
    ) else if /I "%%a"=="--devschema"  ( set DO_DEV_SCHEMA=1
    ) else if /I "%%a"=="--per-module" ( set DO_PER_MODULE=1
    ) else if /I "%%a"=="--test-venv"  ( set DO_SETUP_VENV=1
    ) else ( set "_REMAINING=!_REMAINING! %%a" )
)
if defined _REMAINING (
    echo Unknown flags:!_REMAINING!
    exit /b 1
)

REM Map unified flags to ovruntime internal variables
set DO_CLEAN_INTERNAL=0
if "!DO_CLEAN!"=="1" set DO_CLEAN_INTERNAL=1
if "!DO_REBUILD!"=="1" set DO_CLEAN_INTERNAL=1

REM Default to release only
if "!BUILD_CONFIG!"=="debug" (
    set DO_DEBUG=1
    set DO_RELEASE=0
) else (
    set DO_DEBUG=0
    set DO_RELEASE=1
)

set DO_GET_DEPS_AND_CONFIGURE=0
if "!DO_GENERATE_ONLY!"=="1" set DO_GET_DEPS_AND_CONFIGURE=1

REM BUILD_TARGET from CLI takes precedence, fall back to env var
if "!BUILD_TARGET!"=="" if defined OVRUNTIME_BUILD_TARGET set "BUILD_TARGET=!OVRUNTIME_BUILD_TARGET!"

REM Record overall start time using epoch seconds to avoid cmd time parsing bugs.
call :capture_epoch START_EPOCH

REM Clean _* folders
if !DO_CLEAN_INTERNAL!==0 goto :skip_clean
echo Cleaning build artifacts...
for /d %%d in (_*) do (
    echo   Removing %%d
    rmdir /s /q "%%d"
)
:skip_clean

REM -c/--clean: clean only, exit
if "!DO_CLEAN!"=="1" if "!DO_REBUILD!"=="0" (
    echo Clean complete.
    exit /b 0
)

REM Pull dependencies with packman
echo Pulling dependencies...
if !DO_DEV_PHYSX!==1 (
    call pull_dependencies.bat --devphysx
) else (
    call pull_dependencies.bat
)
if !ERRORLEVEL! neq 0 (
    echo Failed to pull dependencies!
    exit /b 1
)

if not exist %BUILD_DIR% mkdir %BUILD_DIR%
if not exist %COMPILER_DIR% mkdir %COMPILER_DIR%

REM Skip cmake build when only setting up the venv
if !DO_SETUP_VENV!==1 goto :setup_venv

REM Use packman-provided cmake if available (matching version used by build.sh)
set CMAKE=_build\target-deps\cmake\bin\cmake.exe
if not exist %CMAKE% (
    echo Warning: packman cmake not found at %CMAKE%, falling back to system cmake
    set CMAKE=cmake
)

REM ---- Use packman MSVC from host-deps ----
set MSVC_HOST_DIR=_build\host-deps\msvc
set WINSDK_HOST_DIR=_build\host-deps\winsdk

REM Host-toolchain mode: skip the packman MSVC setup when the package is absent
REM (e.g. removed from host-deps for the open-source release); the VS generator
REM then auto-detects the locally-installed MSVC/WinSDK.
set "CMAKE_PACKMAN_ARGS="
if not exist "%MSVC_HOST_DIR%\VC\Tools\MSVC" (
    echo Using host-installed MSVC/WinSDK toolchain ^(packman MSVC absent^)
    goto :after_packman_msvc
)

REM Find the actual MSVC tools version directory
for /d %%v in ("%MSVC_HOST_DIR%\VC\Tools\MSVC\*") do (
    set MSVC_TOOLS_DIR=%CD%\%%v
    set MSVC_TOOLS_VER=%%~nxv
)
echo   MSVC tools: !MSVC_TOOLS_VER!

REM Discover WinSDK paths (support flat and versioned layouts)
set WINSDK_ROOT=%CD%\%WINSDK_HOST_DIR%
set WINSDK_SDK_VER=
if exist "%WINSDK_ROOT%\Include\ucrt" (
    set WINSDK_INCLUDE_BASE=%WINSDK_ROOT%\Include
    set WINSDK_LIB_BASE=%WINSDK_ROOT%\Lib
    set WINSDK_BIN_DIR=%WINSDK_ROOT%\bin\x64
) else (
    for /d %%v in ("%WINSDK_ROOT%\Include\*") do (
        set WINSDK_SDK_VER=%%~nxv
        set WINSDK_INCLUDE_BASE=%WINSDK_ROOT%\Include\%%~nxv
        set WINSDK_LIB_BASE=%WINSDK_ROOT%\Lib\%%~nxv
        set WINSDK_BIN_DIR=%WINSDK_ROOT%\bin\%%~nxv\x64
    )
)
set MSVC_BIN_DIR=!MSVC_TOOLS_DIR!\bin\Hostx64\x64
set PATH=!MSVC_BIN_DIR!;!WINSDK_BIN_DIR!;!PATH!
set VCToolsInstallDir=!MSVC_TOOLS_DIR!\
set VCINSTALLDIR=%CD%\%MSVC_HOST_DIR%\VC\

set INCLUDE_DIRS=!MSVC_TOOLS_DIR!\include;!WINSDK_INCLUDE_BASE!\ucrt;!WINSDK_INCLUDE_BASE!\um;!WINSDK_INCLUDE_BASE!\shared
set LIB_DIRS=!MSVC_TOOLS_DIR!\lib\x64;!WINSDK_LIB_BASE!\ucrt\x64;!WINSDK_LIB_BASE!\um\x64

REM The packaged NVIDIA.ImportBefore.props ships with stale values that break
REM the VS generator: wrong VCToolsVersion, and a relative VCInstallDir_170
REM that resolves incorrectly from the amd64 MSBuild.  Rather than patching the
REM packman-distributed file (which may be read-only or cached), we drop an
REM override props file that MSBuild imports AFTER the NVIDIA one (alphabetical
REM wildcard import).
set _MSVC_ROOT=%CD%\%MSVC_HOST_DIR%
set _IMPORT_BEFORE_DIR=!_MSVC_ROOT!\MSBuild\Current\Imports\Microsoft.Common.Props\ImportBefore
set _OVERRIDE_PROPS=!_IMPORT_BEFORE_DIR!\ZZ_PackmanFix.ImportBefore.props
if exist "!_IMPORT_BEFORE_DIR!" (
    (
        echo ^<Project xmlns="http://schemas.microsoft.com/developer/msbuild/2003"^>
        echo   ^<PropertyGroup^>
        echo     ^<VCToolsVersion^>!MSVC_TOOLS_VER!^</VCToolsVersion^>
        echo     ^<VCInstallDir_170^>!_MSVC_ROOT!\VC\^</VCInstallDir_170^>
        echo     ^<VCToolsInstallDir_170^>$^(VCInstallDir_170^)Tools\MSVC\$^(VCToolsVersion^)\^</VCToolsInstallDir_170^>
        echo   ^</PropertyGroup^>
        echo ^</Project^>
    ) > "!_OVERRIDE_PROPS!"
    echo   Created !_OVERRIDE_PROPS!
    echo     VCToolsVersion=!MSVC_TOOLS_VER!  VCInstallDir_170=!_MSVC_ROOT!\VC\
)

REM Create Microsoft.VCToolsVersion.{ver}.props — cmake imports this but it doesn't ship with packman MSVC
set _VTV_DIR=!_MSVC_ROOT!\VC\Auxiliary\Build\!MSVC_TOOLS_VER!
if not exist "!_VTV_DIR!" mkdir "!_VTV_DIR!"
(echo ^<?xml version="1.0" encoding="utf-8"?^>
echo ^<Project ToolsVersion="4.0" xmlns="http://schemas.microsoft.com/developer/msbuild/2003"^>
echo   ^<PropertyGroup^>^<VCToolsVersion^>!MSVC_TOOLS_VER!^</VCToolsVersion^>^</PropertyGroup^>
echo ^</Project^>
) > "!_VTV_DIR!\Microsoft.VCToolsVersion.!MSVC_TOOLS_VER!.props"

REM cmake 3.25.1 embeds CMAKE_GENERATOR_INSTANCE literally (incl. ,version=) as a path in vcxproj files.
REM Work around by reading the MSBuild version and creating a junction so that path resolves to msvc/.
for /f "delims=" %%v in ('powershell -NoProfile -Command "$f=Get-Item %CD%\%MSVC_HOST_DIR%\MSBuild\Current\Bin\MSBuild.exe; $f.VersionInfo.FileVersion"') do set VS_BUILD_VERSION=%%v
set _INST_LINK=%CD%\%MSVC_HOST_DIR%,version=!VS_BUILD_VERSION!
if not exist "!_INST_LINK!" mklink /J "!_INST_LINK!" "%CD%\%MSVC_HOST_DIR%"

REM Directory.Build.props: sets include/lib/exe paths for cmake's VCTargetsPath detection project
(
echo ^<Project^>
echo   ^<PropertyGroup^>
echo     ^<VCToolsVersion^>!MSVC_TOOLS_VER!^</VCToolsVersion^>
echo     ^<WindowsSDKDir^>!WINSDK_ROOT!\^</WindowsSDKDir^>
if not "!WINSDK_SDK_VER!"=="" echo     ^<WindowsTargetPlatformVersion^>!WINSDK_SDK_VER!^</WindowsTargetPlatformVersion^>
echo     ^<IncludePath^>!INCLUDE_DIRS!;$^(IncludePath^)^</IncludePath^>
echo     ^<LibraryPath^>!LIB_DIRS!;$^(LibraryPath^)^</LibraryPath^>
echo     ^<ExecutablePath^>!MSVC_BIN_DIR!;!WINSDK_BIN_DIR!;$^(ExecutablePath^)^</ExecutablePath^>
echo   ^</PropertyGroup^>
echo ^</Project^>
) > "%COMPILER_DIR%\Directory.Build.props"

set INCLUDE_DIRS_ESC=!INCLUDE_DIRS:;=\;!
set LIB_DIRS_ESC=!LIB_DIRS:;=\;!
set CMAKE_PACKMAN_ARGS=-DCMAKE_GENERATOR_INSTANCE=%CD%\%MSVC_HOST_DIR%,version=!VS_BUILD_VERSION!
set CMAKE_PACKMAN_ARGS=!CMAKE_PACKMAN_ARGS! -DCMAKE_VS_PLATFORM_TOOLSET_VERSION=!MSVC_TOOLS_VER!
set CMAKE_PACKMAN_ARGS=!CMAKE_PACKMAN_ARGS! -DCMAKE_VS_SDK_INCLUDE_DIRECTORIES=!INCLUDE_DIRS_ESC!
set CMAKE_PACKMAN_ARGS=!CMAKE_PACKMAN_ARGS! -DCMAKE_VS_SDK_LIBRARY_DIRECTORIES=!LIB_DIRS_ESC!

:after_packman_msvc
set "CMAKE_TOOLSET_ARGS="
if exist "%CD%\_build\target-deps\cuda\bin\nvcc.exe" (
    REM VS-generator CUDA detection needs an explicit CUDA toolset, not just CMAKE_CUDA_COMPILER.
    set "CMAKE_TOOLSET_ARGS=-T cuda=%CD%\_build\target-deps\cuda"
)
set "CACHE_FILE=%COMPILER_DIR%\CMakeCache.txt"
if exist "!CACHE_FILE!" (
    for /f "tokens=1,* delims==" %%a in ('findstr /b /c:"CMAKE_GENERATOR_INSTANCE:" "!CACHE_FILE!"') do (
        if not "%%b"=="" set "CACHED_GENERATOR_INSTANCE=%%b"
    )
    for /f "tokens=1,* delims==" %%a in ('findstr /b /c:"CMAKE_GENERATOR_TOOLSET:INTERNAL=" "!CACHE_FILE!"') do (
        if not "%%b"=="" set "CACHED_GENERATOR_TOOLSET=%%b"
    )
)
if defined CACHED_GENERATOR_INSTANCE (
    set CMAKE_PACKMAN_ARGS=-DCMAKE_GENERATOR_INSTANCE=!CACHED_GENERATOR_INSTANCE!
    set CMAKE_PACKMAN_ARGS=!CMAKE_PACKMAN_ARGS! -DCMAKE_VS_PLATFORM_TOOLSET_VERSION=!MSVC_TOOLS_VER!
    set CMAKE_PACKMAN_ARGS=!CMAKE_PACKMAN_ARGS! -DCMAKE_VS_SDK_INCLUDE_DIRECTORIES=!INCLUDE_DIRS_ESC!
    set CMAKE_PACKMAN_ARGS=!CMAKE_PACKMAN_ARGS! -DCMAKE_VS_SDK_LIBRARY_DIRECTORIES=!LIB_DIRS_ESC!
)
if defined CACHED_GENERATOR_TOOLSET (
    set "CMAKE_TOOLSET_ARGS=-T !CACHED_GENERATOR_TOOLSET!"
)

REM Optional: build local schema first (for --devschema)
if !DO_DEV_SCHEMA!==1 (
    echo DevSchema mode: building local physics schema...
    set SCHEMA_BUILD_ARGS=
    if !DO_DEBUG!==1   if !DO_RELEASE!==0 set "SCHEMA_BUILD_ARGS=!SCHEMA_BUILD_ARGS! -d"
    if !DO_RELEASE!==1 if !DO_DEBUG!==0   set "SCHEMA_BUILD_ARGS=!SCHEMA_BUILD_ARGS! -r"
    set "SCHEMA_FETCH_ARGS=!SCHEMA_BUILD_ARGS!"
    if !DO_CLEAN_INTERNAL!==1 set "SCHEMA_FETCH_ARGS=!SCHEMA_FETCH_ARGS! -x"
    pushd ..\schema
    call repo build --fetch-only !SCHEMA_FETCH_ARGS!
    if !ERRORLEVEL! neq 0 ( echo Schema fetch failed! & exit /b 1 )
    call repo usd
    if !ERRORLEVEL! neq 0 ( echo Schema usd gen failed! & exit /b 1 )
    call repo build !SCHEMA_BUILD_ARGS!
    if !ERRORLEVEL! neq 0 ( echo Schema build failed! & exit /b 1 )
    popd
)

REM Optional: build PhysX SDK from source
set CMAKE_DEVPHYSX_ARGS=
if !DO_DEV_PHYSX!==1 (
    echo DevPhysX mode: building PhysX SDK from source...
    set CMAKE_DEVPHYSX_ARGS=-DOVRUNTIME_DEV_PHYSX=ON
)
set CMAKE_DEVSCHEMA_ARGS=
if !DO_DEV_SCHEMA!==1 (
    set CMAKE_DEVSCHEMA_ARGS=-DOVRUNTIME_DEV_SCHEMA=ON
)
set CMAKE_PER_MODULE_ARGS=
if !DO_PER_MODULE!==1 (
    echo Per-module install layout enabled.
    set CMAKE_PER_MODULE_ARGS=-DOVRUNTIME_INSTALL_PER_MODULE=ON
)
REM Configure
%CMAKE% -S . -B %COMPILER_DIR% -G "Visual Studio 17 2022" -A x64 !CMAKE_TOOLSET_ARGS! -DTARGET_BUILD_PLATFORM=windows !CMAKE_PACKMAN_ARGS! !CMAKE_DEVPHYSX_ARGS! !CMAKE_DEVSCHEMA_ARGS! !CMAKE_PER_MODULE_ARGS! !PYTHON_DIR_ARG! !OUTPUT_ARGS!
if !ERRORLEVEL! neq 0 (
    echo CMake configuration failed!
    exit /b 1
)

REM Generate test helper scripts into _build\
if not exist "_build" mkdir "_build"
(
echo @echo off
echo setlocal enableextensions
echo "%%~dp0target-deps\cmake\bin\ctest.exe" --test-dir "%%~dp0..\!COMPILER_DIR!" -C Release -V
) > "_build\test_unit.bat"
REM Only generate test_python.bat when USD has Python support (usd.nopy has no lib\python).
if exist "_build\target-deps\usd\release\lib\python" (
(
echo @echo off
echo setlocal enableextensions
echo pushd "%%~dp0.."
echo "%%~dp0.venv\Scripts\python.exe" -m pytest --junitxml="%%~dp0..\pytest_results.xml"
echo popd
) > "_build\test_python.bat"
) else (
(
echo @echo off
echo echo Skipping Python tests ^(USD built without Python support^)
echo exit /b 0
) > "_build\test_python.bat"
    echo Generated no-op test_python.bat ^(USD built without Python support^)
)

if !DO_GET_DEPS_AND_CONFIGURE!==1 exit /b 0

REM Build Debug
if !DO_DEBUG!==0 goto :skip_debug
call :capture_epoch T0_EPOCH
if not "!BUILD_TARGET!"=="" (
    %CMAKE% --build %COMPILER_DIR% --config Debug --target !BUILD_TARGET! -- /m
) else (
    %CMAKE% --build %COMPILER_DIR% --config Debug -- /m
)
if !ERRORLEVEL! neq 0 (
    echo Debug build failed!
    exit /b 1
)
call :elapsed !T0_EPOCH!
echo Debug build: !ELAPSED_STR!
if "!BUILD_TARGET!"=="" if !SKIP_INSTALL!==0 %CMAKE% --install %COMPILER_DIR% --config Debug --prefix %INSTALL_DIR%\debug
:skip_debug

REM Build Release
if !DO_RELEASE!==0 goto :skip_release
call :capture_epoch T0_EPOCH
if not "!BUILD_TARGET!"=="" (
    %CMAKE% --build %COMPILER_DIR% --config Release --target !BUILD_TARGET! -- /m
) else (
    %CMAKE% --build %COMPILER_DIR% --config Release -- /m
)
if !ERRORLEVEL! neq 0 (
    echo Release build failed!
    exit /b 1
)
call :elapsed !T0_EPOCH!
echo Release build: !ELAPSED_STR!
if "!BUILD_TARGET!"=="" if !SKIP_INSTALL!==0 %CMAKE% --install %COMPILER_DIR% --config Release --prefix %INSTALL_DIR%\release
:skip_release

if not "!BUILD_TARGET!"=="" goto :finish
if !SKIP_VENV!==1 goto :finish

:setup_venv
REM Skip venv setup entirely when USD has no Python support (usd.nopy package).
if not exist "_build\target-deps\usd\release\lib\python" (
    echo Skipping Python test environment setup ^(USD built without Python support^)
    goto :finish
)
REM Setup Python test environment (.venv for VS Code debugging)
REM Uses packman Python to ensure version matches native extensions
set PACKMAN_PYTHON=_build\target-deps\python\python.exe
if not exist _build\.venv (
    echo Creating Python test environment...
    %PACKMAN_PYTHON% -m venv _build\.venv
    _build\.venv\Scripts\python.exe -m pip install --quiet pytest numpy warp-lang debugpy
) else (
    echo Python test environment already exists.
)
if !DO_SETUP_VENV!==1 exit /b 0


REM Install Python package paths into the venv via .pth files
REM This makes "from pxr import Usd, ..." and "import carb" work for VS Code
for /f "usebackq delims=" %%p in (`_build\.venv\Scripts\python.exe -c "import sysconfig; print(sysconfig.get_path(\"purelib\"))"`) do set VENV_SITE=%%p

if exist _build\target-deps\usd\release\lib\python (
    echo %CD%\_build\target-deps\usd\release\lib\python> "%VENV_SITE%\usd.pth"
    echo Installed USD Python path: %VENV_SITE%\usd.pth
)

if exist _build\target-deps\carb_sdk_plugins\_build\windows-x86_64\release\bindings-python (
    echo %CD%\_build\target-deps\carb_sdk_plugins\_build\windows-x86_64\release\bindings-python> "%VENV_SITE%\carb.pth"
    echo Installed Carb Python path: %VENV_SITE%\carb.pth
)

if !DO_DEV_SCHEMA!==1 (
    set PHYSX_SCHEMA_PY_DIR=%CD%\..\schema\_build\windows-x86_64\release\schema\lib\python
) else (
    set PHYSX_SCHEMA_PY_DIR=%CD%\_build\target-deps\usd_ext_physics\release\lib\python
)
if exist "!PHYSX_SCHEMA_PY_DIR!" (
    echo !PHYSX_SCHEMA_PY_DIR!> "%VENV_SITE%\physx_schema.pth"
    echo Installed PhysxSchema Python path: %VENV_SITE%\physx_schema.pth
)

:finish
REM Total elapsed time
call :elapsed %START_EPOCH%
echo.
echo Build completed successfully in !ELAPSED_STR!
exit /b 0

REM ---- Subroutine: capture current time as UTC epoch seconds ----
:capture_epoch
for /f "usebackq delims=" %%t in (`powershell -NoProfile -Command "[DateTimeOffset]::UtcNow.ToUnixTimeSeconds()"`) do set "%~1=%%t"
goto :eof

REM ---- Subroutine: compute elapsed time from start epoch to now ----
:elapsed
call :capture_epoch NOW_EPOCH
set /a "DIFF=NOW_EPOCH-%~1"
if !DIFF! lss 0 set DIFF=0
set /a "EH=DIFF / 3600"
set /a "EM=(DIFF %% 3600) / 60"
set /a "ES=DIFF %% 60"
if !EM! lss 10 set EM=0!EM!
if !ES! lss 10 set ES=0!ES!
if !EH! gtr 0 (
    set "ELAPSED_STR=!EH!h !EM!m !ES!s"
) else if !EM! gtr 0 (
    set "ELAPSED_STR=!EM!m !ES!s"
) else (
    set "ELAPSED_STR=!ES!s"
)
goto :eof
