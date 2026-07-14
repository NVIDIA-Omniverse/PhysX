@echo off
REM SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
REM SPDX-License-Identifier: LicenseRef-NvidiaProprietary

set DO_DEV_PHYSX=0
:parse_args
if "%~1"=="" goto :done_args
if /I "%~1"=="--devphysx" set DO_DEV_PHYSX=1
shift
goto :parse_args
:done_args

REM Fetch both release and debug configs so that the CMake build can select the
REM right USD/TBB libraries for each build type (e.g. usd\release\lib vs usd\debug\lib).

echo Pulling Physics ovruntime dependencies with packman...

pushd "%~dp0"

set PACKMAN=..\ovexts\tools\packman\packman.cmd
set PLATFORM=windows-x86_64

REM Pull host deps (msvc) -- skipped when absent (open-source build uses local toolchain)
if exist deps\host-deps.packman.xml call %PACKMAN% pull deps\host-deps.packman.xml -p %PLATFORM%
if %ERRORLEVEL% neq 0 (
    echo Failed to pull host dependencies!
    popd
    exit /b %ERRORLEVEL%
)

REM Pull target deps (PhysX, onnx-mlir, physxdevice, leveldb, snappy, python311)
call %PACKMAN% pull deps\target-deps.packman.xml -p %PLATFORM%
if %ERRORLEVEL% neq 0 (
    echo Failed to pull target dependencies!
    popd
    exit /b %ERRORLEVEL%
)

REM Pull ovruntime_deps package (RTX/fabric plugins, headers)
call %PACKMAN% pull deps\ovruntime-deps.packman.xml -p %PLATFORM% -t platform_target_abi=%PLATFORM%
if %ERRORLEVEL% neq 0 (
    echo Failed to pull ovruntime_deps!
    popd
    exit /b %ERRORLEVEL%
)

REM Pull config-dependent deps for both release and debug
for %%C in (release debug) do (
    REM Import dependencies from ovruntime_deps (Carbonite, USD, Python, CUDA, etc.)
    call %PACKMAN% pull deps\ovruntime-deps-import.packman.xml -p %PLATFORM% -t platform_target_abi=%PLATFORM% -t config=%%C
    if %ERRORLEVEL% neq 0 (
        echo Failed to pull ovruntime_deps imports [%%C]!
        popd
        exit /b %ERRORLEVEL%
    )

    REM Pull kit-kernel for dev headers not yet in ovruntime_deps (omni/timeline, omni/kit/renderer, etc.)
    call %PACKMAN% pull deps\kit-kernel-deps.packman.xml -p %PLATFORM% -t platform_target_abi=%PLATFORM% -t config=%%C
    if %ERRORLEVEL% neq 0 (
        echo Failed to pull kit-kernel [%%C]!
        popd
        exit /b %ERRORLEVEL%
    )

    REM Pull schema deps (physxSchema, physicsSchemaTools headers/libs)
    call %PACKMAN% pull deps\schema-deps.packman.xml -p %PLATFORM% -t platform_target_abi=%PLATFORM% -t config=%%C
    if %ERRORLEVEL% neq 0 (
        echo Failed to pull schema dependencies [%%C]!
        popd
        exit /b %ERRORLEVEL%
    )
)

if %DO_DEV_PHYSX% neq 0 (
    REM Pull PhysX source-build dependencies into the current environment so
    REM PM_SECURELOADLIBRARY_PATH and related variables are available.
    call %PACKMAN% pull ..\..\physx\dependencies.xml --platform vc17win64
    if %ERRORLEVEL% neq 0 (
        echo Failed to pull PhysX source-build dependencies!
        popd
        exit /b %ERRORLEVEL%
    )
)

REM Fetch pip dependencies (Newton USD schemas) using the packman Python.
REM The canonical package list lives in deps\pip_newton.toml.
set PYTHON=_build\target-deps\python\python.exe
if exist "%PYTHON%" (
    "%PYTHON%" tools\pip_fetch.py deps\pip_newton.toml
) else (
    echo Warning: Python not found at %PYTHON% — skipping pip fetch
)

popd

echo.
echo Dependencies pulled successfully!
echo.
