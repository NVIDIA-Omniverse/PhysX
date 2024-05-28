# NVIDIA PhysX SDK for Linux

## Location of Binaries:

* SDK libraries: bin/linux.clang


## Required packages to generate projects:

* CMake, minimum version 3.14
* Python, minimum version 3.5
* curl
* For linux x86-64: glibc, version 2.31 or higher (Note: GLIBC versions are typically not backwards compatible)

Compilers and C++ Standard:
  * For linux x86-64 builds we support Ubuntu LTS releases with their respective default compiler versions:
    * Ubuntu 20.04 LTS with gcc 9 or clang 10
    * Ubuntu 22.04 LTS with gcc 11 or clang 14 
    * Note: PhysX may fail to compile with clang versions higher than 14 as they have not been tested
  * For linux aarch64 builds we support gcc version 9
  * C++17 compatible


## Generating Makefiles:

* Makefiles are generated through a script in the physx root directory: generate_projects.sh
* The script generate_projects.sh expects a preset name as a parameter, if a parameter is not provided it does list the available presets and you can select one.
* Generated solutions are placed in the folders compiler/linux-debug, compiler/linux-checked, compiler/linux-profile, compiler/linux-release.


## Building SDK:

* Makefiles are in compiler/linux-debug etc
* Clean solution: make clean
* Build solution: make
* Install solution: make install

Note:
Compile errors on unsupported compilers or platforms are frequently caused by additional warnings that are treated as errors by default.
While we cannot offer support in this case we recommend removing all occurences of the `-Werror` flag in the file `physx/source/compiler/cmake/linux/CMakeLists.txt`.

## PhysX GPU Acceleration:

* Running GPU-accelerated simulations requires a CUDA toolkit 11.8 compatible display driver. The corresponding driver version can be found [here](https://docs.nvidia.com/cuda/cuda-toolkit-release-notes/index.html#cuda-major-component-versions__table-cuda-toolkit-driver-versions).
* Pascal, CUDA ARCH 6.0 GPU or higher
* Note that CUDA is not required for building PhysX, it is only a runtime requirement for GPU-accelerated scenes.

## Required Packages for Building and Running PhysX Snippets:

* freeglut3
* libglu1
* libxdamage-dev
* libxmu6

## How to select the PhysX GPU Device:

* Set the environment variable PHYSX_GPU_DEVICE to the device ordinal on which GPU PhysX should run. Default is 0.
* Example: export PHYSX_GPU_DEVICE="1"

