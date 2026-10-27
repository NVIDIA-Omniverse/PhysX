# NVIDIA PhysX SDK 5

Copyright (c) 2008-2026 NVIDIA Corporation & Affiliates. All rights reserved.

PhysX is licensed under the [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0).
The complete license is provided in `LICENSE.md`.

## Introduction

Welcome to the NVIDIA PhysX SDK source code repository.

The NVIDIA PhysX SDK is a scalable multi-platform physics solution for CPUs and GPUs.  See [PhysX SDK on developer.nvidia.com](https://developer.nvidia.com/physx-sdk).

The [Release Notes](./CHANGELOG.md) contain updates pertaining to the latest version.

## User Guide and API Documentation

The user guide and API documentation are available on [GitHub Pages](https://nvidia-omniverse.github.io/PhysX/physx/index.html). Please create an [Issue](https://github.com/NVIDIA-Omniverse/PhysX/issues/) if you find a documentation issue.

## Quick Start Instructions

Platform specific environment and build information can be found in [documentation/platformreadme](./documentation/platformreadme).

To begin, clone this repository onto your local drive.  Then change directory to physx/, run ./generate_projects.[bat|sh] and follow on-screen prompts.  This will let you select a platform specific solution to build.  You can then build from the generated solution/make file in the platform- and configuration-specific folders in the ``compiler`` folder.

Note that the PhysX distribution downloads binary content, such as the PhysX GPU binaries, from Amazon CloudFront on demand, using the packman package manager.

## Acknowledgements

This depot references packages of third party open source software copyright their respective owners.
For copyright details, please refer to the license files included in the packages.

| Software                  | Copyright Holder                                                                    | Package                          |
|---------------------------|-------------------------------------------------------------------------------------|----------------------------------|
| CMake                     | Kitware, Inc. and Contributors                                                      | cmake                            |
| LLVM                      | University of Illinois at Urbana-Champaign                                          | clang-physxmetadata              |
| Visual Studio Locator     | Microsoft Corporation                                                               | VsWhere                          |
| Freeglut                  | Pawel W. Olszta                                                                     | freeglut-windows                 |
| Mesa 3-D graphics library | Brian Paul                                                                          | OpenGL                           |
| RapidJSON                 | THL A29 Limited, a Tencent company, and Milo Yip<br>Alexander Chemeris (msinttypes) | rapidjson                        |
| OpenGL Ext Wrangler Lib   | Nigel Stewart, Milan Ikits, Marcelo E. Magallon, Lev Povalahev                      | [SDK_ROOT]/snippets/graphics     |
