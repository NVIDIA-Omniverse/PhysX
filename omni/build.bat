@echo off

set RUN_SCHEMA_GEN=0

set SCRIPT_DIR=%~dp0
set ARGS=%*

:: Schema gen
:parseargs
if not "%1" == "" (
    if "%1" == "--devschema" (
        set RUN_SCHEMA_GEN=1
    )
    shift
    goto parseargs
)

if "%RUN_SCHEMA_GEN%" == "1" (
      pushd schema
      call "./repo" build --fetch-only %*
      if %errorlevel% neq 0 ( exit /b %errorlevel% )
      call "./repo" usd
      if %errorlevel% neq 0 ( exit /b %errorlevel% )
      popd
   )

:: Main build
call "%SCRIPT_DIR%repo" build %ARGS%
if %errorlevel% neq 0 ( exit /b %errorlevel% )
