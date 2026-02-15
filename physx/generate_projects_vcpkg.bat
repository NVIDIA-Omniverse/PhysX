@echo off
REM Alternative PhysX project generator that uses vcpkg instead of packman
REM Usage: generate_projects_vcpkg.bat <preset-name>
REM Example: generate_projects_vcpkg.bat windows-vc16

setlocal enabledelayedexpansion

set SCRIPT_DIR=%~dp0
set PHYSX_ROOT_DIR=%SCRIPT_DIR%

REM Remove trailing backslash if present
if "%PHYSX_ROOT_DIR:~-1%"=="\" set PHYSX_ROOT_DIR=%PHYSX_ROOT_DIR:~0,-1%

echo =========================================
echo PhysX Project Generator (vcpkg Mode)
echo =========================================
echo.

REM Check for vcpkg installation
echo Checking for vcpkg installation...

REM Check if VCPKG_ROOT environment variable is set
if not defined VCPKG_ROOT (
    echo.
    echo ERROR: vcpkg not found!
    echo.
    echo vcpkg is required for packman-free builds on Windows.
    echo.
    echo Installation instructions:
    echo   1. Open PowerShell as Administrator
    echo   2. cd C:\
    echo   3. git clone https://github.com/Microsoft/vcpkg.git
    echo   4. cd vcpkg
    echo   5. .\bootstrap-vcpkg.bat
    echo   6. Set VCPKG_ROOT environment variable to C:\vcpkg
    echo.
    echo Alternatively, install to a custom location and set VCPKG_ROOT
    echo.
    exit /b 1
)

echo Found vcpkg at: %VCPKG_ROOT%
echo.

REM Check for required packages
echo Checking required vcpkg packages...
set MISSING_PACKAGES=

REM Check for rapidjson
%VCPKG_ROOT%\vcpkg.exe list rapidjson | findstr /C:"rapidjson" >nul 2>&1
if errorlevel 1 (
    set MISSING_PACKAGES=!MISSING_PACKAGES! rapidjson:x64-windows
)

REM Check for freeglut
%VCPKG_ROOT%\vcpkg.exe list freeglut | findstr /C:"freeglut" >nul 2>&1
if errorlevel 1 (
    set MISSING_PACKAGES=!MISSING_PACKAGES! freeglut:x64-windows
)

if not "!MISSING_PACKAGES!"=="" (
    echo.
    echo ERROR: Missing required vcpkg packages!
    echo.
    echo The following packages are required but not installed:
    echo !MISSING_PACKAGES!
    echo.
    echo Please install them using:
    echo   %VCPKG_ROOT%\vcpkg.exe install!MISSING_PACKAGES!
    echo.
    echo Full installation command:
    echo   %VCPKG_ROOT%\vcpkg.exe install rapidjson:x64-windows freeglut:x64-windows
    echo.
    exit /b 1
)

echo All required packages found
echo.

REM Set up environment for vcpkg integration
set CMAKE_TOOLCHAIN_FILE=%VCPKG_ROOT%\scripts\buildsystems\vcpkg.cmake
set VCPKG_TARGET_TRIPLET=x64-windows

echo Using vcpkg packages:
echo   RapidJSON: vcpkg
echo   FreeGLUT: vcpkg
echo.
echo PhysX Root: %PHYSX_ROOT_DIR%
echo CMake Toolchain: %CMAKE_TOOLCHAIN_FILE%
echo =========================================
echo.

REM Run CMake project generator
if "%~1"=="" (
    echo Running project generator...
    python "%PHYSX_ROOT_DIR%\buildtools\cmake_generate_projects.py"
) else (
    echo Generating project for preset: %~1
    python "%PHYSX_ROOT_DIR%\buildtools\cmake_generate_projects.py" "%~1"
)

if errorlevel 1 (
    echo.
    echo ERROR: Project generation failed!
    exit /b 1
)

echo.
echo =========================================
echo Generation complete!
echo Build directories created in: %PHYSX_ROOT_DIR%\compiler\
echo.
echo To build, open the generated Visual Studio solution or use:
echo   cmake --build compiler\^<preset-name^>-^<config^>\ --config Release
echo =========================================
