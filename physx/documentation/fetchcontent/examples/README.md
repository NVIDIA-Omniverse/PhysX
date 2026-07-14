# PhysX Examples

This directory contains examples showing different ways to integrate and use PhysX in your projects.

## Available Examples

### [fetchcontent/](fetchcontent/)
**Modern CMake FetchContent Integration**

Demonstrates how to use PhysX with CMake's FetchContent feature for zero-configuration setup.

#### Quick Start
```bash
cd fetchcontent/
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel --config Release

# Linux (single-config, e.g. Ninja):
./build/hello-physx-cmake

# Windows: path depends on generator
#   Single-config (e.g. Ninja):  .\build\hello-physx-cmake.exe
#   Multi-config (Visual Studio): .\build\Release\hello-physx-cmake.exe  or  .\build\Debug\hello-physx-cmake.exe
```