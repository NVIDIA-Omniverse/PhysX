# NVIDIA PhysX SDK for Linux

## Location of Binaries:

* SDK libraries: bin/linux.clang


## Required packages to generate projects:

* CMake, minimum version 3.14
* Python, minimum version 3.5
* curl

Compilers:
  * For linux x86-64 builds we support Ubuntu LTS releases with their respective default compiler versions:
    * Ubuntu 20.04 LTS with gcc 9 or clang 10
    * Ubuntu 22.04 LTS with gcc 11 or clang 14
  * For linux aarch64 builds we support gcc version 9


## Generating Makefiles:

* Makefiles are generated through a script in physx root directory: generate_projects.sh
* Script generate_projects.sh expects a preset name as a parameter, if a parameter is not provided it does list the available presets and you can select one.
* Supported presets for linux platform are: linux, linux-aarch64.
* Generated solutions are in folder compiler/linux-debug, compiler/linux-checked, compiler/linux-profile, compiler/linux-release.


## Building SDK:

* Makefiles are in compiler/linux-debug etc
* Clean solution: make clean
* Build solution: make


## PhysX GPU Acceleration:

* Requires CUDA 11.8 compatible display driver. The corresponding driver version can be found [here](https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html#cuda-major-component-versions__table-cuda-toolkit-driver-versions).


## Required Packages for Building and Running PhysX Snippets:

* freeglut3
* libglu1
* libxdamage-dev
* libxmu6

## How to select the PhysX GPU Device:

* Set the environment variable PHYSX_GPU_DEVICE to the device ordinal on which GPU PhysX should run. Default is 0.
* Example: export PHYSX_GPU_DEVICE="1"

