@echo off
SETLOCAL EnableDelayedExpansion

:: Locate Visual Studio using vswhere
IF EXIST "%ProgramFiles(x86)%\Microsoft Visual Studio\Installer" (
    SET "VS_INSTALLER_DIR=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer"
    echo VS_INSTALLER_DIR: "!VS_INSTALLER_DIR!"
    
    :: Check if VS_INSTALLER_DIR is already in PATH
    echo !PATH! | findstr /i /c:"!VS_INSTALLER_DIR!" >nul
    if !errorlevel! neq 0 (
        SET PATH=!PATH!;!VS_INSTALLER_DIR!
        echo Updated PATH: !PATH!
    ) else (
        echo VS_INSTALLER_DIR is already in PATH
    )
)

:: Use vswhere to locate the latest Visual Studio installation
for /f "usebackq tokens=*" %%i in (`vswhere -version "[17.0,18.0)" -latest -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
    SET "VSINSTALLPATH=%%i"
    echo VSINSTALLPATH: "!VSINSTALLPATH!"
)

:: Set COMNTOOLS to point to the correct path
SET "COMNTOOLS=!VSINSTALLPATH!\VC\Auxiliary\Build\vcvarsx86_amd64.bat"
echo COMNTOOLS: "!COMNTOOLS!"
@call "!COMNTOOLS!"

:: When run with no arguments we always perform a rebuild (clean & build)
@if [%1] == [] (
    set TARGET=Rebuild
) else (
    set TARGET=%1
)

@pushd %~dp0\..\..\compiler
@set ROOT_PATH=%CD%
@popd

:: Windows 'all'
@set SOLUTION_PATH=vc17win64
@call :BUILD
@if %ERRORLEVEL% neq 0 goto ERROR

:: Success
@exit /B 0

:ERROR
@echo Failure while building *Windows vc17* targets!
@exit /B 1

:BUILD
@echo | set /p dummyName=** Building %SOLUTION_PATH% debug ... **
msbuild /property:configuration=debug "%ROOT_PATH%\%SOLUTION_PATH%\PhysXSDK.sln" /maxcpucount /t:%TARGET% /v:m
@echo ** End of %SOLUTION_PATH% debug **
@echo.
@if %ERRORLEVEL% neq 0 exit /B 1

@echo | set /p dummyName=** Building %SOLUTION_PATH% profile ... **
msbuild /property:configuration=profile "%ROOT_PATH%\%SOLUTION_PATH%\PhysXSDK.sln" /maxcpucount /t:%TARGET% /v:m
@echo ** End of %SOLUTION_PATH% profile **
@echo.
@if %ERRORLEVEL% neq 0 exit /B 1

@echo | set /p dummyName=** Building %SOLUTION_PATH% checked ... **
msbuild /property:configuration=checked "%ROOT_PATH%\%SOLUTION_PATH%\PhysXSDK.sln" /maxcpucount /t:%TARGET% /v:m
@echo ** End of %SOLUTION_PATH% checked **
@echo.
@if %ERRORLEVEL% neq 0 exit /B 1

@echo | set /p dummyName=** Building %SOLUTION_PATH% release ... **
msbuild /property:configuration=release "%ROOT_PATH%\%SOLUTION_PATH%\PhysXSDK.sln" /maxcpucount /t:%TARGET% /v:m
@echo ** End of %SOLUTION_PATH% release **
@echo.
@if %ERRORLEVEL% neq 0 exit /B 1
@exit /B
