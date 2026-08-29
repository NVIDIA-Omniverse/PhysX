@echo off
REM SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
REM SPDX-License-Identifier: BSD-3-Clause

setlocal enableextensions enabledelayedexpansion

REM ovphysx Build Script (Windows)
REM
REM Usage examples:
REM   build.bat                           - Release build (default)
REM   build.bat --clean                   - Clean only
REM   build.bat --rebuild                 - Clean + rebuild
REM   build.bat --debug                   - Debug build
REM   build.bat --target mylib            - Build specific target
REM
REM NOTE: This script uses packaged MSVC/WinSDK from packman when present, and
REM falls back to a local Visual Studio installation otherwise (public source
REM drop). The generator is selected via the GENERATOR environment variable
REM (ninja or vs). See build.cmake for details.

REM Clear VS environment variables that might pollute the omni/ build.
set VCINSTALLDIR=
set VCToolsInstallDir=
set VSINSTALLDIR=
set VCToolsVersion=
set VCToolsRedistDir=

REM Parse unified build flags
call "%~dp0cmake\parse_build_args.bat" %*

REM Process project-specific flags from EXTRA_ARGS
set "DO_DEV_PHYSX=0"
set "DO_DEV_SCHEMA=0"
set "DO_BENCHMARKS=0"
set "DEFAULT_RELEASE_RUNTIME_DEPS=1"
set "USER_SET_RELEASE_RUNTIME_DEPS=0"
set "_REMAINING="
set "CMAKE_PASSTHROUGH="
for %%a in (!EXTRA_ARGS!) do (
    if /I "%%a"=="--devphysx" (
        set "DO_DEV_PHYSX=1"
    ) else if /I "%%a"=="--devschema" (
        set "DO_DEV_SCHEMA=1"
    ) else if /I "%%a"=="--benchmarks" (
        set "DO_BENCHMARKS=1"
    ) else (
        set "_TMP_ARG=%%a"
        if /I "!_TMP_ARG:~0,35!"=="-DOVPHYSX_USE_RELEASE_RUNTIME_DEPS=" (
            set "USER_SET_RELEASE_RUNTIME_DEPS=1"
            set "CMAKE_PASSTHROUGH=!CMAKE_PASSTHROUGH! %%a"
        ) else if "!_TMP_ARG:~0,2!"=="-D" (
            set "CMAKE_PASSTHROUGH=!CMAKE_PASSTHROUGH! %%a"
        ) else (
            set "_REMAINING=!_REMAINING! %%a"
        )
    )
)
if defined _REMAINING (
    echo Unknown flags:!_REMAINING!
    echo.
    echo Usage: build.bat [flags]
    call :print_usage
    exit /b 1
)

REM Map unified flags to cmake -D args
set "CMAKE_ARGS="
if "!DO_CLEAN!"=="1" set "CMAKE_ARGS=!CMAKE_ARGS! -DCLEAN_BUILD=ON"
if "!DO_REBUILD!"=="1" set "CMAKE_ARGS=!CMAKE_ARGS! -DCLEAN_BUILD=ON -DCLEAN_ONLY=OFF"
if "!BUILD_CONFIG!"=="debug" set "CMAKE_ARGS=!CMAKE_ARGS! -DBUILD_TYPE=Debug"
if "!DO_GENERATE_ONLY!"=="1" set "CMAKE_ARGS=!CMAKE_ARGS! -DGENERATE_ONLY=ON"
if not "!BUILD_TARGET!"=="" set "CMAKE_ARGS=!CMAKE_ARGS! -DBUILD_TARGET=!BUILD_TARGET!"
if "!DO_DEV_PHYSX!"=="1" set "CMAKE_ARGS=!CMAKE_ARGS! -DDEV_PHYSX=ON"
if "!DO_DEV_SCHEMA!"=="1" set "CMAKE_ARGS=!CMAKE_ARGS! -DDEV_SCHEMA=ON"
if "!DO_BENCHMARKS!"=="1" set "CMAKE_ARGS=!CMAKE_ARGS! -DBENCHMARKS=ON"
if "!USER_SET_RELEASE_RUNTIME_DEPS!"=="0" if "!DEFAULT_RELEASE_RUNTIME_DEPS!"=="1" set "CMAKE_ARGS=!CMAKE_ARGS! -DOVPHYSX_USE_RELEASE_RUNTIME_DEPS=ON"

cmake !CMAKE_ARGS! !CMAKE_PASSTHROUGH! -P "%~dp0scripts\build.cmake"
if errorlevel 1 (
    echo ERROR: Build failed
    exit /b 1
)

echo.
echo Build completed successfully!
exit /b 0

:print_usage
echo Common flags:
echo   -c, --clean        Clean artifacts only (no build)
echo   -x, --rebuild      Clean then build
echo   -d, --debug        Build debug configuration
echo   -r, --release      Build release configuration
echo   -t, --target NAME  Build a specific CMake target
echo   -g, --generate     Configure only (no build)
echo.
echo Project-specific flags:
echo   --devphysx         Build PhysX SDK from source
echo   --devschema        Use locally-built physics schema
echo   --benchmarks       Build the opt-in benchmark suite (tests/benchmarks/)
echo   -DOVPHYSX_USE_RELEASE_RUNTIME_DEPS=ON  Use release runtime deps for Debug builds
echo.
echo Changing --devphysx or --devschema needs a clean rebuild:
echo the flag combination selects a build flavor, and incremental
echo builds across a flavor change are not supported. Add --rebuild
echo whenever the flags differ from the previous build.
goto :eof
