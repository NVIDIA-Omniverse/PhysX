# NVIDIA PhysX SDK for Windows

## Location of Binaries:

* SDK DLLs/LIBs: bin/
* PhysX Device DLLs: bin/

## Required packages to generate projects:

* CMake, minimum version 3.21
* Python, minimum version 3.5

## PhysX GPU Acceleration:

* Requires CUDA 11.0 compatible display driver and CUDA ARCH 3.0 compatible GPU

## Generating solutions for Visual Studio:

* Solutions are generated through a script in the physx root directory: generate_projects.bat
* The generate_projects.bat script expects a preset name as a parameter; if a parameter is not provided, it lists the available presets.
* The generated solutions are in the folder compiler/"preset name".

## Notes:

* The PhysXDevice/64.dll must be redistributed with GPU-enabled applications, and must live in the same directory as PhysXGpu.dll.
* The nvcuda.dll and PhysXUpdateLoader/64.dll are loaded and checked for the NVIDIA Corporation digital signature. The signature is expected on all NVIDIA Corporation provided dlls. The application will exit if the signature check failed.
