:: Reset errorlevel status so we are not inheriting this state from the calling process:
:: @call :CLEAN_EXIT
@echo off
setlocal enabledelayedexpansion

set "PHYSX_ROOT_DIR=%~dp0"

:: Convert backslashes to forward slashes
set "PHYSX_ROOT_DIR=%PHYSX_ROOT_DIR:\=/%"

set PACKMAN_CMD="%PHYSX_ROOT_DIR%buildtools\packman\packman"

:: Initialize Packman (Needed to get PM_PYTHON)
call %PACKMAN_CMD% init
if errorlevel 1 @exit /b %errorlevel%

IF "%1"=="" GOTO ADDITIONAL_PARAMS_MISSING

:: Run packman to ensure dependencies are present and run cmake generation script afterwards
echo Running packman in preparation for cmake ...
set "str1=%1"
if not x%str1:.user=%==x%str1% (
    call %PACKMAN_CMD% pull "%PHYSX_ROOT_DIR%dependencies.xml" --platform %str1:.user=%
) else (
    call %PACKMAN_CMD% pull "%PHYSX_ROOT_DIR%dependencies.xml" --platform %1
)

for /f "usebackq tokens=*" %%i in (`"%PM_vswhere_PATH%\VsWhere.exe  -version [15.0,16.0) -latest -property installationPath"`) do (
	set "Install2017Dir=%%i"
	set "VS150PATH=%%i"
)

for /f "usebackq tokens=*" %%i in (`"%PM_vswhere_PATH%\VsWhere.exe  -version [16.0,17.0) -latest -property installationPath"`) do (
  set "Install2019Dir=%%i"
	set "VS160PATH=%%i"
  @REM Setting VS160COMNTOOLS: This is mainly needed for building for Switch
  @REM Reason: When both MS build tools and Visual Studio are installed together in the same system
  @REM Cmake will use msbuild to generate the project instead of Visual Studio. Which make Cmake fail
  @REM When generating the project for Switch. However, if an environment variable of the form VS##0COMNTOOLS,
  @REM where ## the Visual Studio major version number, is set and points to the Common7/Tools directory within
  @REM one of the VS instances, that instance will be used. see: https://cmake.org/cmake/help/latest/variable/CMAKE_GENERATOR_INSTANCE.html
  set "VS160COMNTOOLS=%%i\Common7\Tools\"
)

for /f "usebackq tokens=*" %%i in (`"%PM_vswhere_PATH%\VsWhere.exe  -version [17.0,18.0) -latest -property installationPath"`) do (
	set "Install2022Dir=%%i"
	set "VS170PATH=%%i"
  set "VS170COMNTOOLS=%%i\Common7\Tools\"
)

if exist "%Install2017Dir%\VC\Auxiliary\Build\Microsoft.VCToolsVersion.default.txt" (
  pushd "%Install2017Dir%\VC\Auxiliary\Build\"
  set /p Version=<Microsoft.VCToolsVersion.default.txt
  for /f "delims=" %%x in (Microsoft.VCToolsVersion.default.txt) do (
	if not "%%x"=="" (
	  rem Example hardcodes x64 as the host and target architecture, but you could parse it from arguments
	  set "VS150CLPATH=%Install2017Dir%\VC\Tools\MSVC\%%x\bin\HostX64\x64\cl.exe"
	)
  )
  popd
)

if exist "%Install2019Dir%\VC\Auxiliary\Build\Microsoft.VCToolsVersion.default.txt" (
  pushd "%Install2019Dir%\VC\Auxiliary\Build\"
  set /p Version=<Microsoft.VCToolsVersion.default.txt
  for /f "delims=" %%x in (Microsoft.VCToolsVersion.default.txt) do (
	if not "%%x"=="" (
	  rem Example hardcodes x64 as the host and target architecture, but you could parse it from arguments
	  set "VS160CLPATH=%Install2019Dir%\VC\Tools\MSVC\%%x\bin\HostX64\x64\cl.exe"
	)
  )
  popd
)

if exist "%Install2022Dir%\VC\Auxiliary\Build\Microsoft.VCToolsVersion.default.txt" (
  pushd "%Install2022Dir%\VC\Auxiliary\Build\"
  set /p Version=<Microsoft.VCToolsVersion.default.txt
  for /f "delims=" %%x in (Microsoft.VCToolsVersion.default.txt) do (
	if not "%%x"=="" (
	  rem Example hardcodes x64 as the host and target architecture, but you could parse it from arguments
	  set "VS170CLPATH=%Install2022Dir%\VC\Tools\MSVC\%%x\bin\HostX64\x64\cl.exe"
	)
  )
  popd
)

:ADDITIONAL_PARAMS_MISSING
call %PM_PYTHON% %PHYSX_ROOT_DIR%buildtools/cmake_generate_projects.py %1
if %ERRORLEVEL% neq 0 (
  set /p DUMMY=Hit ENTER to continue...
  exit /b %errorlevel%
) else (
  goto CLEAN_EXIT
)

:CLEAN_EXIT
@exit /b 0