# NVIDIA PhysX SDK for Mac

## Location of Binaries:

* SDK libraries: bin/mac.x86_64, bin/mac.arm64

## Required packages to generate projects:

* CMake, minimum version 3.20
* Python, minimum version 3.5
* XCode, minimum version 12

### Notes
* Packman script uses 'realpath' command. It can be installed via homebrew by running the command 'brew install coreutils'.
* Python package used by packman is based on x86_x64 architecture and generates the error "dyld[number]: missing symbol called" when run on an arm-based Mac. To use your system's python add the environment variable PM_PYTHON_EXT (for example by adding 'export PM_PYTHON_EXT="python3"' into your .bash_profile file).

## Generating Makefiles:

* Makefiles are generated through a script in physx root directory: generate_projects.sh
* Script generate_projects.bat expects a preset name as a parameter, if a parameter is not provided it does list the available presets and you can select one.
* Supported presets for Mac platform are: mac64, mac-arm64.
* Generated solutions are in folder compiler/mac64, compiler/mac-arm64.

## Building SDK:

* XCode projects are in compiler/mac64, compiler/mac-arm64.
* Build solution: cmake --build . --config (debug|checked|profile|release)

## Limitations:

* PhysX Snippets are not supported.
* PhysX Systems that require a CUDA capable GPU are not supported, for example particle system or cloth simulation.
* Omniverse Visual Debugger (OmniPVD) is not supported.
