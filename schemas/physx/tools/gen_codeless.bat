@echo off
:: SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
:: SPDX-License-Identifier: Apache-2.0
::
:: Build step for the codeless schemas (Windows). Regenerates everything from schema.usda
:: using only this tooling + native OpenUSD usdGenSchema (NO repo_usd):
::   1. generatedSchema.usda + plugInfo.json per codeless schema via tools/gen_schema_data.py
::      (native usdGenSchema).
::   2. physxSchema/tokens.h + physxSchema/_tokens.py via tools/gen_tokens.py (GatherTokens).
::   3. header-only C++ wrappers + Python API via tools/gen_codeless_api.py.
:: Result: a fully codeless schema with NO compiled schema libraries and no repo_usd dependency.
setlocal enableextensions enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
set "ROOT=%SCRIPT_DIR%.."
set "SRC=%ROOT%\source\physxSchema"

set "USD=%ROOT%\_build\target-deps\usd\release"
if not exist "%USD%\lib\python\pxr" set "USD=%ROOT%\_build\target-deps\usd\debug"
if not exist "%USD%\lib\python\pxr" (
    echo [gen_codeless] ERROR: could not find target-deps USD with pxr bindings; run after deps fetch. 1>&2
    exit /b 1
)

set "USD_PY=%ROOT%\_build\target-deps\python\python.exe"
set "PATH=%USD%\lib;%USD%\bin;%PATH%"

:: usdGenSchema (imported by gen_tokens.py) needs jinja2. Install it locally via pip
:: rather than borrowing it from repo_usd (deprecated). Cached under _build after first run.
set "JINJA_DIR=%ROOT%\_build\codeless_gen_deps"
if not exist "%JINJA_DIR%\jinja2" "%USD_PY%" -m pip install --quiet --disable-pip-version-check --target "%JINJA_DIR%" jinja2
set "PYTHONPATH=%JINJA_DIR%;%USD%\lib\python;%PYTHONPATH%"

:: 1. schema DATA (generatedSchema.usda + plugInfo.json) via native usdGenSchema, for every
::    codeless schema. Replaces repo.sh usd (repo_usd).
for %%D in ("%SRC%" "%ROOT%\source\omniUsdPhysicsDeformableSchema") do (
    echo [gen_codeless] generating schema data ^(native usdGenSchema^): %%~D
    call "%USD_PY%" "%ROOT%\tools\gen_schema_data.py" --schema-dir "%%~D"
    if errorlevel 1 exit /b 1
)

:: 2.+3. header-only tokens (C++) + pure-Python Tokens, then header-only C++ wrappers + Python
::       API, straight from schema.usda -- for every codeless schema. The C++/Python library
::       prefix (e.g. PhysxSchema -> PhysxSchemaTokens / PhysxSchema.<Class>) and the include
::       directory are passed per schema so a single generator serves all codeless schemas.
call :gen_codeless_one "%SRC%" "PhysxSchema" "physxSchema"
if errorlevel 1 exit /b 1
call :gen_codeless_one "%ROOT%\source\omniUsdPhysicsDeformableSchema" "OmniUsdPhysicsDeformableSchema" "omniUsdPhysicsDeformableSchema"
if errorlevel 1 exit /b 1

:: 4. derived per-axis instance tokens for physxJointAxis / physxDrivePerformanceEnvelope, built
::    from the codeless PhysxSchemaTokens *_MultipleApplyTemplate_* tokens (replaces the old
::    hand-authored physicsSchemaTools per-axis constants). Must run after physxSchema tokens.
echo [gen_codeless] generating PhysxAxisInstanceTokens from physxSchema templates
call "%USD_PY%" "%ROOT%\tools\gen_axis_instance_tokens.py" ^
    --tokens-h "%SRC%\tokens.h" ^
    --out-h    "%SRC%\axisInstanceTokens.h"
if errorlevel 1 exit /b 1
goto :eof

:gen_codeless_one
set "DIR=%~1"
set "PREFIX=%~2"
set "LIBDIR=%~3"
echo [gen_codeless] regenerating header-only %PREFIX% tokens from schema.usda
call "%USD_PY%" "%ROOT%\tools\gen_tokens.py" ^
    --schema "%DIR%\schema.usda" ^
    --out-h  "%DIR%\tokens.h" ^
    --out-py "%DIR%\_tokens.py" ^
    --prefix "%PREFIX%" ^
    --lib-dir "%LIBDIR%"
if errorlevel 1 exit /b 1
echo [gen_codeless] generating codeless %PREFIX% API from schema.usda (USD: %USD%)
call "%USD_PY%" "%ROOT%\tools\gen_codeless_api.py" ^
    --schema   "%DIR%\schema.usda" ^
    --tokens-h "%DIR%\tokens.h" ^
    --out-cpp  "%DIR%" ^
    --out-py   "%DIR%\codeless_api.py" ^
    --prefix "%PREFIX%" ^
    --lib-dir "%LIBDIR%"
if errorlevel 1 exit /b 1
goto :eof
