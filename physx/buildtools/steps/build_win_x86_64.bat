@echo off
SETLOCAL EnableDelayedExpansion

:: Check if at least one argument is provided (preset)
if "%~1"=="" (
    echo You must specify a preset: e.g. vc17win64...
    exit /B 1
)

:: Set the preset based on the argument
set "PRESET=%1"

:: Extract the Visual Studio version from the preset
set "VS_PREFIX=%PRESET:~0,4%"  :: Get the first 4 characters (vc15, vc16, vc17)

:: Determine the correct Visual Studio version based on the preset prefix
if "%VS_PREFIX%" == "vc15" (
    set "VS_VERSION=[15.0,16.0)"
) else if "%VS_PREFIX%" == "vc16" (
    set "VS_VERSION=[16.0,17.0)"
) else if "%VS_PREFIX%" == "vc17" (
    set "VS_VERSION=[17.0,18.0)"
) else (
    echo Unsupported Visual Studio version in preset: %PRESET%
    exit /B 1
)

:: Check if a build configuration is provided, default to 'all' if not
if "%~2" == "" (
    set "BUILD_CONFIG=all"
) else (
    set "BUILD_CONFIG=%2"
)

:: Validate BUILD_CONFIG
if not "%BUILD_CONFIG%"=="debug" if not "%BUILD_CONFIG%"=="release" if not "%BUILD_CONFIG%"=="checked" if not "%BUILD_CONFIG%"=="profile" if not "%BUILD_CONFIG%"=="all" (
    echo Invalid build configuration. Use one of: debug, release, checked, profile, all.
    exit /B 1
)

:: Locate Visual Studio using vswhere
IF EXIST "%ProgramFiles(x86)%\Microsoft Visual Studio\Installer" (
    set "VS_INSTALLER_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer"
    echo VS_INSTALLER_DIR: "!VS_INSTALLER_DIR!"
    
    :: Check if VS_INSTALLER_DIR is already in PATH
    echo !PATH! | findstr /i /c:"!VS_INSTALLER_DIR!" >nul
    if errorlevel 1 (
        set "PATH=!PATH!;!VS_INSTALLER_DIR!"
        echo Updated PATH: !PATH!
    ) else (
        echo VS_INSTALLER_DIR is already in PATH
    )
)

:: Use vswhere to locate the specified Visual Studio installation
for /f "usebackq tokens=*" %%i in (`vswhere -version "%VS_VERSION%" -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
    set "VSINSTALLPATH=%%i"
    echo VSINSTALLPATH: "!VSINSTALLPATH!"
)

:: Check if VSINSTALLPATH is set
if not defined VSINSTALLPATH (
    echo Visual Studio installation not found.
    exit /B 1
)

:: Set COMNTOOLS to point to the correct path
set "COMNTOOLS=!VSINSTALLPATH!\VC\Auxiliary\Build\vcvarsx86_amd64.bat"
echo COMNTOOLS: "!COMNTOOLS!"
call "!COMNTOOLS!"
if errorlevel 1 (
    echo Failed to initialize Visual Studio environment.
    exit /B 1
)

:: Safely handle directory changes
pushd "%~dp0\..\..\compiler"
set "ROOT_PATH=%CD%"
popd

:: Build configurations
if "%BUILD_CONFIG%" == "all" (
    call :BUILD debug
    if errorlevel 1 goto ERROR

    call :BUILD release
    if errorlevel 1 goto ERROR

    call :BUILD checked
    if errorlevel 1 goto ERROR

    call :BUILD profile
    if errorlevel 1 goto ERROR
) else (
    call :BUILD %BUILD_CONFIG%
    if errorlevel 1 goto ERROR
)

:: Success
echo Build completed successfully.
exit /B 0

:ERROR
echo Failure while building *Windows %VS_PREFIX%* targets!
exit /B 1

:: Build subroutine
:BUILD
echo ** Building %PRESET% %1 ... **

:: Check if PRESET ends with -carbonite using string manipulation
setlocal enabledelayedexpansion
set "SUFFIX=-carbonite"
set "PRESET_LENGTH=!PRESET:~-10!"

if "%PRESET_LENGTH%" == "%SUFFIX%" (
    :: Build INSTALL.vcxproj when the preset ends with -carbonite
    msbuild /property:configuration=%1 "%ROOT_PATH%\%PRESET%\INSTALL.vcxproj" /maxcpucount /t:Rebuild /v:m
) else (
    :: Build PhysXSDK.sln when the preset does not end with -carbonite
    msbuild /property:configuration=%1 "%ROOT_PATH%\%PRESET%\PhysXSDK.sln" /maxcpucount /t:Rebuild /v:m
)

if errorlevel 1 (
    echo Build failed for %1.
    exit /B 1
)
echo ** End of %PRESET% %1 **
echo.
goto :EOF



@REM This is for binary distro only (carbonite). Keeping it here in case we need it.
@REM However if we need the licenses and the logic below, it will be probably better to do that in the zipup script not here


:: Copying licenses and files directly, similar to individual scripts
:: Ensure that destination directories exist by creating them before copying

@REM echo Copying LICENSE.md to PhysX...
@REM mkdir "%ROOT_PATH%\..\install\%INSTALL_PATH%\PhysX\PACKAGE-LICENSES" 2>nul
@REM echo f | xcopy /S /Y /I "%ROOT_PATH%\..\documentation\license\PACKAGE-LICENSES\LICENSE.md" "%ROOT_PATH%\..\install\%INSTALL_PATH%\PhysX\PACKAGE-LICENSES\physxsdk-LICENSE.md"
@REM if errorlevel 1 (
@REM     echo Failed to copy LICENSE.md to PhysX.
@REM     goto ERROR
@REM )

@REM echo Copying vhacd-LICENSE.md to VHACD...
@REM mkdir "%ROOT_PATH%\..\install\%INSTALL_PATH%\VHACD\PACKAGE-LICENSES" 2>nul
@REM echo f | xcopy /S /Y /I "%ROOT_PATH%\..\documentation\license\PACKAGE-LICENSES\vhacd-LICENSE.md" "%ROOT_PATH%\..\install\%INSTALL_PATH%\VHACD\PACKAGE-LICENSES\vhacd-LICENSE.md"
@REM if errorlevel 1 (
@REM     echo Failed to copy vhacd-LICENSE.md to VHACD.
@REM     goto ERROR
@REM )

@REM echo Copying physxsdk-PACKAGE-INFO.yaml to PhysX...
@REM mkdir "%ROOT_PATH%\..\install\%INSTALL_PATH%\PhysX" 2>nul
@REM echo f | xcopy /S /Y /I "%ROOT_PATH%\..\documentation\license\physxsdk-PACKAGE-INFO.yaml" "%ROOT_PATH%\..\install\%INSTALL_PATH%\PhysX\PACKAGE-INFO.yaml"
@REM if errorlevel 1 (
@REM     echo Failed to copy physxsdk-PACKAGE-INFO.yaml to PhysX.
@REM     goto ERROR
@REM )

@REM echo Copying vhacd-PACKAGE-INFO.yaml to VHACD...
@REM mkdir "%ROOT_PATH%\..\install\%INSTALL_PATH%\VHACD" 2>nul
@REM echo f | xcopy /S /Y /I "%ROOT_PATH%\..\documentation\license\vhacd-PACKAGE-INFO.yaml" "%ROOT_PATH%\..\install\%INSTALL_PATH%\VHACD\PACKAGE-INFO.yaml"
@REM if errorlevel 1 (
@REM     echo Failed to copy vhacd-PACKAGE-INFO.yaml to VHACD.
@REM     goto ERROR
@REM )

@REM :: Copying VHACD binaries and includes
@REM echo Copying VHACD debug binaries...
@REM mkdir "%ROOT_PATH%\..\install\%INSTALL_PATH%\VHACD\bin\%BINARY_PATH%\debug" 2>nul
@REM xcopy /Y "%ROOT_PATH%\..\bin\%BINARY_PATH%\debug\VHACD*.*" "%ROOT_PATH%\..\install\%INSTALL_PATH%\VHACD\bin\%BINARY_PATH%\debug\"
@REM if errorlevel 1 (
@REM     echo Failed to copy VHACD debug binaries.
@REM     goto ERROR
@REM )

@REM echo Copying VHACD includes...
@REM mkdir "%ROOT_PATH%\..\install\%INSTALL_PATH%\VHACD\include" 2>nul
@REM xcopy /Y "%ROOT_PATH%\..\externals\VHACD\public\*.*" "%ROOT_PATH%\..\install\%INSTALL_PATH%\VHACD\include\"
@REM if errorlevel 1 (
@REM     echo Failed to copy VHACD includes.
@REM     goto ERROR
@REM )