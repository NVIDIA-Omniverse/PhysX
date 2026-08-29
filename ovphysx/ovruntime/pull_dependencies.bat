@echo off
REM SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
REM SPDX-License-Identifier: BSD-3-Clause

REM Usage: pull_dependencies.bat [--config release|debug|all] [--devphysx] [--devschema]

set "DO_DEV_PHYSX=0"
set "DO_DEV_SCHEMA=0"
set "CONFIGS=release"
:parse_args
if "%~1"=="" goto :done_args
if /I "%~1"=="--devphysx" set "DO_DEV_PHYSX=1"
if /I "%~1"=="--devschema" set "DO_DEV_SCHEMA=1"
if /I "%~1"=="--config" (
    if "%~2"=="" (
        echo Error: --config requires an argument: release, debug, or all
        goto :arg_error
    )
    if /I "%~2"=="release" (
        set "CONFIGS=release"
    ) else if /I "%~2"=="debug" (
        set "CONFIGS=debug"
    ) else if /I "%~2"=="all" (
        set "CONFIGS=release debug"
    ) else (
        echo Error: unsupported --config value "%~2"; expected release, debug, or all
        goto :arg_error
    )
    shift
)
shift
goto :parse_args
:arg_error
exit /b 1
:done_args

REM Fetch only the requested config by default. Use --config all for workflows
REM that need both release and debug dependencies in one pull.

echo Pulling Physics ovruntime dependencies with packman...
echo USD mode: namespaced
echo Configs: %CONFIGS%

pushd "%~dp0"

set PACKMAN=tools\packman\packman.cmd
set PLATFORM=windows-x86_64

REM Pull host deps (msvc)
call %PACKMAN% pull deps\host-deps.packman.xml -p %PLATFORM%
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

set "PYTHON=_build\target-deps\python\python.exe"
if not exist "%PYTHON%" set "PYTHON=_build\target-deps\python311\python.exe"
if not exist "%PYTHON%" (
    echo Failed to find packman Python for ovstage dependency fetch!
    popd
    exit /b 1
)

REM ovstage backend (ADR-0002): fetch the released package per platform into
REM _build\target-deps\ovstage (OVSTAGE_DIR). The packman <source> in
REM deps\ovstage-deps.packman.xml is commented out; the fetch lives in
REM ovphysx\scripts\fetch_ovstage_release.py (shipped in the open-source drop).
REM A local <source> in ovstage-deps.packman.xml still uses packman when present.
set "OVSTAGE_SRC="
set "OVSTAGE_FETCH=..\scripts\fetch_ovstage_release.py"
for /f "tokens=2 delims==" %%S in ('findstr /c:"<source path=" deps\ovstage-deps.packman.xml') do (
    set "OVSTAGE_SRC=%%~S"
)
if defined OVSTAGE_SRC (
    if exist "%OVSTAGE_SRC%" (
        echo Linking ovstage source dependency ^(%OVSTAGE_SRC%^)...
        call %PACKMAN% pull deps\ovstage-deps.packman.xml -p %PLATFORM%
    ) else (
        echo Skipping ovstage source link ^(path not present: %OVSTAGE_SRC%^)
    )
) else (
    if exist "%OVSTAGE_FETCH%" (
        echo Fetching ovstage release for %PLATFORM% ...
        "%PYTHON%" "%OVSTAGE_FETCH%" --platform %PLATFORM% --dest "%CD%\_build\target-deps\ovstage"
        if %ERRORLEVEL% neq 0 (
            echo Failed to fetch ovstage release!
            popd
            exit /b %ERRORLEVEL%
        )
    ) else (
        echo ERROR: %OVSTAGE_FETCH% not found. ovstage is required to build ovruntime on this branch.
        popd
        exit /b 1
    )
)

REM Pull ovruntime_deps (always release variant, config-independent).
call %PACKMAN% pull deps\ovruntime-deps.packman.xml -p %PLATFORM% -t platform_target_abi=%PLATFORM%
if %ERRORLEVEL% neq 0 (
    echo Failed to pull ovruntime_deps!
    popd
    exit /b %ERRORLEVEL%
)

REM Pull config-dependent deps
for %%C in (%CONFIGS%) do (
    REM Pull kit-kernel for dev headers not yet in ovruntime_deps (omni/timeline, omni/kit/renderer, etc.).
    REM Namespaced import manifests also read kit_sdk_%%C\dev\all-deps.packman.xml,
    REM so kit-kernel must exist before the config-dependent import runs.
    call %PACKMAN% pull deps\kit-kernel-deps.packman.xml -p %PLATFORM% -t platform_target_abi=%PLATFORM% -t config=%%C
    if %ERRORLEVEL% neq 0 (
        echo Failed to pull kit-kernel [%%C]!
        popd
        exit /b %ERRORLEVEL%
    )

    REM Import the namespaced dependencies for this build config.
    call %PACKMAN% pull deps\ovruntime-deps-import.packman.xml -p %PLATFORM% -t platform_target_abi=%PLATFORM% -t config=%%C
    if %ERRORLEVEL% neq 0 (
        echo Failed to pull config-dependent imports [%%C]!
        popd
        exit /b %ERRORLEVEL%
    )

    REM Pull namespaced schema deps (physxSchema, physicsSchemaTools headers/libs).
    REM Skipped in --devschema mode: CMake will point at the local schema build instead.
    if %DO_DEV_SCHEMA%==0 (
        call %PACKMAN% pull deps\schema-deps.packman.xml -p %PLATFORM% -t platform_target_abi=%PLATFORM% -t config=%%C
        if %ERRORLEVEL% neq 0 (
            echo Failed to pull schema dependencies [%%C]!
            popd
            exit /b %ERRORLEVEL%
        )
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
if exist "%PYTHON%" (
    "%PYTHON%" tools\pip_fetch.py deps\pip_newton.toml
) else (
    echo Warning: Python not found at %PYTHON% -- skipping pip fetch
)

popd

echo.
echo Dependencies pulled successfully!
echo.
