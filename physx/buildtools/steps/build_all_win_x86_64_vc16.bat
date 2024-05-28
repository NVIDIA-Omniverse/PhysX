:: Setup VS2019 build environment
IF EXIST "%ProgramFiles(x86)%\Microsoft Visual Studio\Installer" SET PATH=%PATH%;%ProgramFiles(x86)%\Microsoft Visual Studio\Installer

for /f "usebackq tokens=*" %%i in (`vswhere  -version "[16.0,17.0)" -latest -property installationPath`) do (
  SET "COMNTOOLS=%%i\Common7\Tools\"
)

@call "%COMNTOOLS%..\..\VC\Auxiliary\Build\vcvarsx86_amd64.bat"

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
@set SOLUTION_PATH=vc16win64
@call :BUILD
@if %ERRORLEVEL% neq 0 goto ERROR

:: Success
@exit /B 0

:ERROR
@echo Failure while building *Windows vc16* targets!
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
