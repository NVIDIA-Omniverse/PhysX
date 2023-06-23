# NVIDIA PhysX SDK for Windows

## Location of Binaries:

* SDK DLLs/LIBs: bin/
* PhysX Device DLLs: bin/

## Required packages to generate projects:

* Windows OS + headers for at least Win7 (`_WIN32_WINNT = 0x0601`)
* CMake, minimum version 3.21
* Python, minimum version 3.5

## PhysX GPU Acceleration:

* Requires CUDA 11.8 compatible display driver. The corresponding driver version can be found [here](https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html#cuda-major-component-versions__table-cuda-toolkit-driver-versions).

## Generating solutions for Visual Studio:

* Solutions are generated through a script in the physx root directory: generate_projects.bat
* The generate_projects.bat script expects a preset name as a parameter; if a parameter is not provided, it lists the available presets.
* The generated solutions are in the folder compiler/"preset name".

## Notes:

* The PhysXDevice/64.dll must be redistributed with GPU-enabled applications, and must live in the same directory as PhysXGpu.dll.
* The nvcuda.dll and PhysXUpdateLoader/64.dll are loaded and checked for the NVIDIA Corporation digital signature. The signature is expected on all NVIDIA Corporation provided dlls. The application will exit if the signature check failed.
