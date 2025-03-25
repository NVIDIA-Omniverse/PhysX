@echo off

call "%~dp0\buildtools\packman\packman" init
set "PYTHONPATH=%PM_MODULE_DIR%;%PYTHONPATH%"

echo Running packman in preparation for premake ...

call "%~dp0buildtools\packman\packman.cmd" pull "%~dp0dependencies.xml" -p windows-x86_64

for /f "tokens=*" %%i in ('"%PM_vswhere_PATH%\VsWhere.exe" -latest -property installationPath') do (
    set InstallDir=%%i
)

for /f "tokens=*" %%i in ('"%PM_vswhere_PATH%\VsWhere.exe" -latest -property installationVersion') do (
    set "VS_VERSION=%%i"
)

:: Check the major version number and map it to the corresponding Visual Studio year version
for /f "tokens=1 delims=." %%v in ("%VS_VERSION%") do (
    set "MAJOR_VERSION=%%v"
)

if "%MAJOR_VERSION%"=="16" (
    set MSVS_VERSION=vs2019
) else if "%MAJOR_VERSION%" GEQ "17" (
    set MSVS_VERSION=vs2022
) else (
    echo Unsupported Visual Studio version: %VS_VERSION%
    goto Error
)

echo Microsoft Visual Studio version: %MSVS_VERSION%
echo Install directory: %InstallDir%

:: Construct the path to cl.exe
setlocal enabledelayedexpansion
if exist "%InstallDir%\VC\Auxiliary\Build\Microsoft.VCToolsVersion.default.txt" (
    pushd "%InstallDir%\VC\Auxiliary\Build\"

    for /f "delims=" %%x in (Microsoft.VCToolsVersion.default.txt) do (
        if not "%%x"=="" (
            set "MSVS_TOOLSET=%InstallDir%\VC\Tools\MSVC\%%x"
        )
    )
    popd
)
endlocal & set "MSVS_TOOLSET=%MSVS_TOOLSET%"

if not exist "%MSVS_TOOLSET%" (
    echo MSVC toolset not found, check Visual Studio installation
    goto Error
)

set CL_PATH="%MSVS_TOOLSET%\bin\HostX64\x64\cl.exe"

:: Check if cl.exe exists and output the path
if exist %CL_PATH% (
    echo cl.exe path: %CL_PATH%
) else (
    echo cl.exe not found: %CL_PATH%
    goto Error
)

:: Construct the path to MSBuild.exe
set "MSBUILD_DIR=%InstallDir%\MSBuild\Current\Bin"
set MSBUILD_PATH="%MSBUILD_DIR%\MSBuild.exe"

:: Check if MSBuild.exe exists and output the path
if exist %MSBUILD_PATH% (
    echo MSBuild.exe path: %MSBUILD_PATH%
) else (
    echo MSBuild.exe not found
    goto Error
)

call %~dp0generate_projects.bat %MSVS_VERSION%
if %errorlevel% neq 0 ( goto Error )

call %MSBUILD_PATH% %~dp0_compiler\%MSVS_VERSION%\nvflow.sln /p:Configuration=debug
if %errorlevel% neq 0 ( goto Error )

call %MSBUILD_PATH% %~dp0_compiler\%MSVS_VERSION%\nvflow.sln /p:Configuration=release
if %errorlevel% neq 0 ( goto Error )

:Success
exit /b 0

:Error
@echo !!! Failure while building nvflow !!!
exit /b %errorlevel%
