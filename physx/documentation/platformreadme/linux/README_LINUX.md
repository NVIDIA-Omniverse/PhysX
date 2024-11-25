# NVIDIA PhysX SDK for Linux

## Location of Binaries:

* SDK libraries: 
  * For Linux x86-64: bin/linux.x86_64
  * For Linux aarch64: bin/linux.aarch64

## Required packages to generate projects:

* CMake, minimum version 3.14
* Python, minimum version 3.5
* curl
* Clang for linux x86-64
* glibc, version 2.31 or higher (Note: GLIBC versions are typically not backwards compatible)

### Compilers and C++ Standard:
  * We support the following Ubuntu LTS releases and compilers:
    * For linux x86-64 (tested with Clang and GCC):
      * Ubuntu 20.04 LTS: Clang 10, GCC 9.4.0
      * Ubuntu 22.04 LTS: Clang 14, GCC 11.4.0
      * Ubuntu 24.04 LTS: Clang 18, GCC 13.2.0
    * For linux aarch64 (tested with GCC only):
      * Ubuntu 20.04 LTS: GCC 9.4.0
  * Tested with C++11 standard

## Generating Makefiles:

* Makefiles are generated through a script in the physx root directory: generate_projects.sh
* The script generate_projects.sh expects a preset name as a parameter, if a parameter is not provided it does list the available presets and you can select one.
* Generated solutions are placed in the folders compiler/linux-debug, compiler/linux-checked, compiler/linux-profile, compiler/linux-release.

## Building SDK:

* Makefiles are located in compiler/linux-debug, etc.
* Clean solution: `make clean`
* Build solution: `make`
* Install solution: `make install`

Note:
Compile errors on unsupported compilers or platforms are frequently caused by additional warnings that are treated as errors by default. While we cannot offer support in this case, we recommend removing all occurrences of the `-Werror` flag in the file `physx/source/compiler/cmake/linux/CMakeLists.txt`.

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
* Example: `export PHYSX_GPU_DEVICE="1"`