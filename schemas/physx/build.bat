@echo off
setlocal enabledelayedexpansion

pushd %~dp0

set "SCRIPT_DIR=%~dp0"
set "PY=%SCRIPT_DIR%_build\target-deps\python\python.exe"
set "PLATFORM=windows-x86_64"
set "PACKMAN=%SCRIPT_DIR%tools\packman\packman.cmd"

set DO_CLEAN=0
for %%a in (%*) do (
    if /I "%%a"=="-x" set DO_CLEAN=1
    if /I "%%a"=="--clean" set DO_CLEAN=1
)
if !DO_CLEAN!==1 (
    echo Cleaning _build\schema and _dist...
    if exist "%SCRIPT_DIR%_build\schema" rmdir /s /q "%SCRIPT_DIR%_build\schema"
    if exist "%SCRIPT_DIR%_dist"          rmdir /s /q "%SCRIPT_DIR%_dist"
)

:: Fetch release deps (schema is codeless; artifacts are config-independent).
call "%PACKMAN%" pull "%SCRIPT_DIR%deps\host-deps.packman.xml" -p %PLATFORM%
if errorlevel 1 exit /b 1
call "%PACKMAN%" pull "%SCRIPT_DIR%deps\kit-kernel-deps.packman.xml" -p %PLATFORM% -t config=release -t platform_target_abi=%PLATFORM%
if errorlevel 1 exit /b 1
call "%PACKMAN%" pull "%SCRIPT_DIR%deps\usd-deps.packman.xml" -p %PLATFORM% -t config=release -t platform_target_abi=%PLATFORM%
if errorlevel 1 exit /b 1

set "CMAKE=%SCRIPT_DIR%_build\host-deps\cmake\bin\cmake.exe"
if not exist "%CMAKE%" (
    echo Warning: packman cmake not found at %CMAKE%, falling back to system cmake
    set "CMAKE=cmake"
)

:: Validate USD compatibility before using it.
call "%PY%" "%SCRIPT_DIR%tools\check_usd_version.py"
if errorlevel 1 exit /b 1

:: Regenerate codeless schema files via native usdGenSchema (no repo_usd, no premake).
call "%SCRIPT_DIR%tools\gen_codeless.bat"
if errorlevel 1 exit /b 1

:: Install schema artifacts and generate the unit database.
"%CMAKE%" -P "%SCRIPT_DIR%tools\install.cmake"
if errorlevel 1 exit /b 1
call "%PY%" "%SCRIPT_DIR%tools\gen_unit_database.py" ^
    "%SCRIPT_DIR%_build\schema\lib\python\PhysicsSchemaTools"
if errorlevel 1 exit /b 1

popd
