# NVIDIA PhysX SDK 5

Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.

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

The NVIDIA PhysX SDK is a scalable multi-platform physics solution supporting a wide range of devices, from smartphones to high-end multicore CPUs and GPUs. PhysX is already integrated into some of the most popular game engines, including Unreal Engine, and Unity3D. [PhysX SDK on developer.nvidia.com](https://developer.nvidia.com/physx-sdk).

Please see [Release Notes](./CHANGELOG.md) for updates pertaining to the latest version.

## User Guide and API Documentation

The user guide and API documentation are available on [GitHub Pages](https://nvidia-omniverse.github.io/PhysX/physx/index.html). Please create an [Issue](https://github.com/NVIDIA-Omniverse/PhysX/issues/) if you find a documentation issue.

## Quick Start Instructions

Platform specific environment and build information can be found in [documentation/platformreadme](./documentation/platformreadme).

To begin, clone this repository onto your local drive.  Then change directory to physx/, run ./generate_projects.[bat|sh] and follow on-screen prompts.  This will let you select a platform specific solution to build.  You can then build from the generated solution/make file in the platform- and configuration-specific folders in the ``compiler`` folder.

## Acknowledgements

This depot references packages of third party open source software copyright their respective owners.
For copyright details, please refer to the license files included in the packages.

| Software                  | Copyright Holder                                                                    | Package                          |
|---------------------------|-------------------------------------------------------------------------------------|----------------------------------|
| CMake                     | Kitware, Inc. and Contributors                                                      | cmake                            |
| LLVM                      | University of Illinois at Urbana-Champaign                                          | clang-physxmetadata              |
| Visual Studio Locator     | Microsoft Corporation                                                               | VsWhere                          |
| Freeglut                  | Pawel W. Olszta                                                                     | freeglut-windows<br>opengl-linux |
| Mesa 3-D graphics library | Brian Paul                                                                          | opengl-linux                     |
| RapidJSON                 | THL A29 Limited, a Tencent company, and Milo Yip<br>Alexander Chemeris (msinttypes) | rapidjson                        |
| OpenGL Ext Wrangler Lib   | Nigel Stewart, Milan Ikits, Marcelo E. Magallon, Lev Povalahev                      | [SDK_ROOT]/snippets/graphics     |
