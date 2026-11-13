<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Install

ovphysx comes as a Python package and as a C SDK. Both contain the same native library.
Choose the Python package for Python, or the C SDK for C or C++. The example in
[Your First Simulation](first_simulation.md) comes in both languages.
These guides use ovphysx 0.8.

## Requirements

ovphysx runs on the following platforms:

| Platform | Python Package | C SDK |
| --- | --- | --- |
| Linux x86-64 with glibc 2.35 or newer, such as Ubuntu 22.04 | Yes | Yes |
| Linux AArch64 with glibc 2.35 or newer | Yes | Yes |
| Windows x64 | Yes | Yes |

On x86-64, the CPU must support Advanced Vector Extensions (AVX), a set of vector instructions.
On Linux, `grep -cw avx /proc/cpuinfo` prints a number above zero when it does.

To use the downloaded packages, you need:

| Requirement | Supported Versions |
| --- | --- |
| Python, for the Python package | 3.10 or newer |
| CMake, for the C SDK | 3.16 or newer |
| C compiler, for the C example in [Your First Simulation](first_simulation.md) | C11 support |
| C++ compiler, for the SDK's C++ examples | C++17 support |

The SDK already contains the compiled ovphysx library. Building ovphysx itself from source
requires a C++17 compiler.

The example in [Your First Simulation](first_simulation.md) runs on either CPU or GPU. Some features, including soft bodies and
particles, require an NVIDIA GPU.

For GPU use, the Python package and C SDK have different requirements:

| Installation | GPU | Driver |
| --- | --- | --- |
| Python with NVIDIA Warp 1.19 | NVIDIA Turing or newer | NVIDIA R580 or newer |
| C SDK | NVIDIA Volta or newer | Compatible with CUDA 12.8 |

The Python requirements come from [Warp](https://pypi.org/project/warp-lang/1.19.0/). For a
Python setup with CUDA 12, refer to
[Warp's installation guide](https://nvidia.github.io/warp/installation.html).
For the C SDK, use the driver listed for your platform in the
[CUDA 12.8 release notes](https://docs.nvidia.com/cuda/archive/12.8.0/cuda-toolkit-release-notes/index.html#id6).
Neither installation needs the CUDA Toolkit.

## Install the Python Package

Install ovphysx, its PhysX schemas, and the Warp version used in this guide:

```bash
python -m pip install "ovphysx==0.8.*" physx-usd-schemas "warp-lang==1.19.*"
```

The command installs these packages:

| Package | What It Is |
| --- | --- |
| `ovphysx` | The simulation library. |
| `ovstage` | The scene library that ovphysx reads scenes from. |
| `physx-usd-schemas` | The definitions of the PhysX settings a USD scene can hold. ovphysx needs them for every scene. |
| `warp-lang` | NVIDIA Warp, the array library that holds the results ovphysx returns. |

pip also installs NumPy and a few small dependencies. To check the installation, print the
version:

```bash
python -c "import ovphysx; print(ovphysx.__version__)"
```

## Install the C SDK

Download the archive for your platform from the
[ovphysx 0.8.0 release](https://github.com/NVIDIA-Omniverse/PhysX/releases/tag/ovphysx-0.8.0)
and extract it to any folder.

C programs read the PhysX schemas from a folder that you choose. The SDK includes a
script that downloads them there:

```bash
python /path/to/ovphysx/samples/c_samples/common/fetch_physx_schemas.py \
    --dest ~/physx-schemas
export PHYSX_USD_SCHEMAS_ROOT=~/physx-schemas
```

On Windows, set `$env:PHYSX_USD_SCHEMAS_ROOT` in PowerShell instead of using `export`.

To use the SDK in a CMake project, point `CMAKE_PREFIX_PATH` at the extracted folder:

```bash
cmake -B build -S . -DCMAKE_PREFIX_PATH=/path/to/ovphysx
```

The first time CMake configures a project, it downloads the ovstage package that matches the
SDK. This download needs Python 3.8 or newer.
[Your First Simulation](first_simulation.md) has a complete project.
