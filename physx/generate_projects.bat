:: Reset errorlevel status so we are not inheriting this state from the calling process:
@call :CLEAN_EXIT
@echo off

pushd %~dp0
set PHYSX_ROOT_DIR=%CD%
popd
SET PHYSX_ROOT_DIR=%PHYSX_ROOT_DIR:\=/%
SET PM_VSWHERE_PATH=%PHYSX_ROOT_DIR%/../externals/VsWhere
SET PM_CMAKEMODULES_PATH=%PHYSX_ROOT_DIR%/../externals/CMakeModules
SET PM_TARGA_PATH=%PHYSX_ROOT_DIR%/../externals/targa
SET PM_WAVEFRONT_PATH=%PHYSX_ROOT_DIR%/../externals/wavefront
SET PM_rapidjson_PATH=%PHYSX_ROOT_DIR%/../externals/rapidjson
SET PM_PATHS=%PM_CMAKEMODULES_PATH%;%PM_TARGA_PATH%;%PM_WAVEFRONT_PATH%;%PM_rapidjson_PATH%

if exist "%PHYSX_ROOT_DIR%/../externals/cmake/x64/bin/cmake.exe" (
    SET "PM_CMAKE_PATH=%PHYSX_ROOT_DIR%/../externals/cmake/x64"
    GOTO CMAKE_EXTERNAL    
)

where /q cmake
IF ERRORLEVEL 1 (    
	ECHO Cmake is missing, please install cmake version 3.14 and up.
    set /p DUMMY=Hit ENTER to continue...
	exit /b 1
)

:CMAKE_EXTERNAL

:: Use the Python launcher if it exists
py --version 2>NUL
IF ERRORLEVEL 0 (
    set PM_PYTHON=py
)

IF ERRORLEVEL 1 (
    python --version 2>NUL
    IF ERRORLEVEL 1 (
        if "%PM_python_PATH%" == "" (        
            ECHO Python is missing, please install python version 3.5 and up. If Python is installed but not in the PATH, then set the env variable PM_python_PATH pointing to python root directory.
            set /p DUMMY=Hit ENTER to continue...
            exit /b 1
        )
    )
    IF ERRORLEVEL 0 (
        if "%PM_python_PATH%" == "" (    
        set PM_PYTHON=python.exe
        ) else (
            set PM_PYTHON="%PM_python_PATH%\python.exe"
        )
    )
)

IF %1.==. GOTO ADDITIONAL_PARAMS_MISSING

for /f "usebackq tokens=*" %%i in (`"%PM_vswhere_PATH%\VsWhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do (
	set InstallDir=%%i
	set VS150PATH="%%i"		
)	

:ADDITIONAL_PARAMS_MISSING
pushd %~dp0
%PM_PYTHON% "%PHYSX_ROOT_DIR%/buildtools/cmake_generate_projects.py" %1
popd
if %ERRORLEVEL% neq 0 (
    set /p DUMMY=Hit ENTER to continue...
    exit /b %errorlevel%
) else (
    goto CLEAN_EXIT
)

:CLEAN_EXIT
exit /b 0