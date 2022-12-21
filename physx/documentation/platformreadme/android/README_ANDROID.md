# NVIDIA PhysX SDK for Android

## Location of Binaries:

* SDK libraries: bin/android.arm64-v8a


## Required packages to generate projects:

* CMake, minimum version 3.14
* Python, minimum version 3.5
* Android SDK, minimum version 29
* Android NDK, minimum version 22


## Generating Makefiles:

* Set the environment variable ANDROID_NDK_HOME to the path to Android NDK.
* Makefiles are generated through a script in physx root directory: generate_projects.bat
* Script generate_projects.bat expects a preset name as a parameter, if a parameter is not provided it does list the available presets and you can select one.
* Supported preset for android platform is: android-arm64-v8a.
* Generated solutions are in folder compiler/android-arm64-v8a-debug, compiler/android-arm64-v8a-checked, compiler/android-arm64-v8a-profile, compiler/android-arm64-v8a-release.


## Building SDK:

* Makefiles are in compiler/android-arm64-v8a-debug, etc.
* Build solution: cmake --build .

## Limitations:

* PhysX Snippets are not supported.
* PhysX Systems that require a CUDA capable GPU are not supported, for example particle system or cloth simulation.
* Omniverse Visual Debugger (OmniPVD) is not supported.
