# NVIDIA PhysX SDK 5

Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
* Neither the name of NVIDIA CORPORATION nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

## Introduction

Welcome to the NVIDIA PhysX SDK source code repository.

The NVIDIA PhysX SDK is a scalable multi-platform physics solution for CPUs and GPUs.  See [PhysX SDK on developer.nvidia.com](https://developer.nvidia.com/physx-sdk).

The [Release Notes](./CHANGELOG.md) contain updates pertaining to the latest version.

## User Guide and API Documentation

The user guide and API documentation are available on [GitHub Pages](https://nvidia-omniverse.github.io/PhysX/physx/index.html). Please create an [Issue](https://github.com/NVIDIA-Omniverse/PhysX/issues/) if you find a documentation issue.

## Quick Start Instructions

### Linux (Ubuntu 20.04+)

**Prerequisites:**
```bash
sudo apt-get update
sudo apt-get install -y cmake clang build-essential curl \
  libglut-dev libglu1-mesa-dev libopengl-dev rapidjson-dev \
  libx11-dev libxext-dev
```

**Note:** This build configuration uses system packages for OpenGL, GLUT, and RapidJSON instead of packman-managed versions.

**Build (Without Packman - Recommended):**
```bash
cd physx
./generate_projects_no_packman.sh linux-clang-cpu-only
cd compiler/linux-clang-cpu-only-release
make -j$(nproc)
```

**Build (Traditional - Uses Packman):**
```bash
cd physx
./generate_projects.sh linux-clang-cpu-only  # Downloads dependencies via packman
cd compiler/linux-clang-cpu-only-release
make -j$(nproc)
```

Built libraries will be in `bin/linux.x86_64/release/`

**Available presets:** Run `./generate_projects_no_packman.sh` (or `./generate_projects.sh`) to see all options (linux-clang, linux-gcc, linux-clang-cpu-only, etc.)

**Note:** The `generate_projects_no_packman.sh` script uses system packages and tools exclusively. Use this for packman-free builds.

### Windows / macOS

Platform specific environment and build information can be found in [documentation/platformreadme](./documentation/platformreadme).

Run `generate_projects.bat` (Windows) or `generate_projects.sh` (macOS) and follow on-screen prompts to select a platform-specific solution to build. You can then build from the generated solution/make file in the platform- and configuration-specific folders in the `compiler` folder.

### Note

**Packman Dependency:** The PhysX distribution uses the packman package manager to download some binary content from Amazon CloudFront. For Linux builds, this dependency has been minimized - OpenGL, GLUT, and RapidJSON now use system packages instead. Packman is still used for some build tools and the metadata generation feature (which may be removed in the future).

## Acknowledgements

This depot references packages of third party open source software copyright their respective owners.
For copyright details, please refer to the license files included in the packages.

| Software                  | Copyright Holder                                                                    | Package / Source                          |
|---------------------------|-------------------------------------------------------------------------------------|-------------------------------------------|
| CMake                     | Kitware, Inc. and Contributors                                                      | cmake (packman or system)                 |
| LLVM                      | University of Illinois at Urbana-Champaign                                          | clang-physxmetadata (packman)             |
| Visual Studio Locator     | Microsoft Corporation                                                               | VsWhere (packman, Windows only)           |
| Freeglut                  | Pawel W. Olszta                                                                     | system package (Linux)<br>freeglut-windows (Windows) |
| Mesa 3-D graphics library | Brian Paul                                                                          | system package (Linux)                    |
| RapidJSON                 | THL A29 Limited, a Tencent company, and Milo Yip<br>Alexander Chemeris (msinttypes) | system package (Linux)<br>rapidjson (others) |
| OpenGL Ext Wrangler Lib   | Nigel Stewart, Milan Ikits, Marcelo E. Magallon, Lev Povalahev                      | [SDK_ROOT]/snippets/graphics              |
