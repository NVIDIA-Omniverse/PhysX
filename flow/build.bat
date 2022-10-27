@echo off

call generate_projects.bat

cd _compiler\vs2017

if defined NVFLOW_MSBUILD2017 (
    echo Using provided NVFLOW_MSBUILD2017
) else (
    echo NVFLOW_MSBUILD2017 not set, attempting default VS2017 install path
    set NVFLOW_MSBUILD2017="%ProgramFiles% (x86)\Microsoft Visual Studio\2017\Professional\MSBuild\15.0\Bin\amd64\MSBuild.exe"
)

call %NVFLOW_MSBUILD2017% nvflow.sln /p:Configuration=debug
call %NVFLOW_MSBUILD2017% nvflow.sln /p:Configuration=release

cd ..\..