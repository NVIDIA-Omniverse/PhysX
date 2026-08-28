@echo off
REM SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
REM SPDX-License-Identifier: BSD-3-Clause

REM
REM Shared argument parser for build scripts (ovphysx, ovruntime).
REM Call this file to parse unified build flags.
REM
REM Usage: call path\to\parse_build_args.bat %*
REM
REM After calling, the following variables are set:
REM   BUILD_CONFIG       "debug" or "release"; empty means caller defaults to release
REM   DO_CLEAN           1 when -c/--clean (clean only, no build)
REM   DO_REBUILD         1 when -x/--rebuild (clean then build)
REM   DO_GENERATE_ONLY   1 when -g/--generate (configure only)
REM   BUILD_TARGET       CMake target name from -t/--target <name>
REM   EXTRA_ARGS         unrecognized args for project-specific handling

set "BUILD_CONFIG="
set "DO_CLEAN=0"
set "DO_REBUILD=0"
set "DO_GENERATE_ONLY=0"
set "BUILD_TARGET="
set "EXTRA_ARGS="

:parse_loop
if "%~1"=="" goto :parse_done
if /I "%~1"=="-c"          ( set "DO_CLEAN=1"          & goto :next )
if /I "%~1"=="--clean"     ( set "DO_CLEAN=1"          & goto :next )
if /I "%~1"=="-x"          ( set "DO_REBUILD=1"        & goto :next )
if /I "%~1"=="--rebuild"   ( set "DO_REBUILD=1"        & goto :next )
if /I "%~1"=="-d"          ( set "BUILD_CONFIG=debug"   & goto :next )
if /I "%~1"=="--debug"     ( set "BUILD_CONFIG=debug"   & goto :next )
if /I "%~1"=="-r"          ( set "BUILD_CONFIG=release"  & goto :next )
if /I "%~1"=="--release"   ( set "BUILD_CONFIG=release"  & goto :next )
if /I "%~1"=="-g"          ( set "DO_GENERATE_ONLY=1"  & goto :next )
if /I "%~1"=="--generate"  ( set "DO_GENERATE_ONLY=1"  & goto :next )
if /I "%~1"=="-t"          ( if "%~2"=="" ( echo Error: --target requires an argument & exit /b 1 ) else ( set "BUILD_TARGET=%~2" & shift & goto :next ) )
if /I "%~1"=="--target"    ( if "%~2"=="" ( echo Error: --target requires an argument & exit /b 1 ) else ( set "BUILD_TARGET=%~2" & shift & goto :next ) )
if /I "%~1"=="-n"          ( goto :next )
if /I "%~1"=="--no-docker" ( goto :next )
REM Collect unrecognized args
set "EXTRA_ARGS=%EXTRA_ARGS% %~1"
:next
shift
goto :parse_loop
:parse_done

REM Trim leading space from EXTRA_ARGS
if defined EXTRA_ARGS (
    set "EXTRA_ARGS=%EXTRA_ARGS:~1%"
)
goto :eof
