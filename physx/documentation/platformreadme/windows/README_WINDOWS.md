# NVIDIA PhysX SDK for Windows

## Location of win32 and win64 Binaries:

* SDK DLLs/LIBs: bin/
* PhysX Device DLLs: bin/

## Required packages to generate projects:

* CMake, minimum version 3.14 (3.14 for vc16win*)
* Python, minimum version 3.5

## Required packages for building and running the Samples:

* Microsoft DirectX SDK June 2010 or later

## PhysX GPU Acceleration:

* Requires CUDA 10.0 compatible display driver and CUDA ARCH 3.0 compatible GPU

## Generating solutions for Visual Studio:

* Solutions are generated through a script in physx root directory: generate_projects.bat
* Script generate_projects.bat expects a preset name as a parameter, if a parameter is not provided it does list the available presets.
* Supported presets for windows platform are: vc12win32, vc12win64, vc14win32, vc14win64, vc15win32, vc15win64, vc16win32, vc16win64.
* Generated solutions are in folder compiler/"preset name".

## Building SDK Source, Snippets and Samples with Visual Studio:

* Visual Studio Solutions are in compiler/"preset name"

## Notes:

* The PhysXDevice/64.dll must be redistributed with GPU-enabled applications, and must live in the PhysXGpu.dll directory.
* The nvcuda.dll and PhysXUpdateLoader/64.dll are loaded and checked for the NVIDIA Corporation digital signature. The signature is expected on all NVIDIA Corporation provided dlls. The application will exit if the signature check failed.
