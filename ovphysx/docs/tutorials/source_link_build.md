<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# Building Against the ovphysx Source Tree

This tutorial shows how to build your application against the ovphysx source
checkout using CMake's `add_subdirectory()`. This lets you edit ovphysx sources
and have your application automatically recompile the changed files. The compile
and link step does not need an installed SDK; running the app still needs an
installed ovphysx runtime layout for Carbonite and PhysX plugins.

For the installed-SDK approach using `find_package()`, refer to the
[Hello World tutorial](hello_world.md) and the [Quickstart guide](quickstart.md).

## Prerequisites

- A local clone of the [PhysX repository](https://github.com/NVIDIA-Omniverse/PhysX)
- CMake 3.16 or newer on Linux, 4.1 or newer on Windows
- A C/C++ compiler (GCC 11+, MSVC 2022, or Clang 14+). On Linux, prefer a version from the tested matrix in the PhysX SDK [Linux platform readme](https://github.com/NVIDIA-Omniverse/PhysX/blob/main/physx/documentation/platformreadme/linux/README_LINUX.md) rather than the newest available.

## CMakeLists.txt

Your application's `CMakeLists.txt` uses `add_subdirectory()` to include
ovphysx and links against `ovphysx::ovphysx` — the same target name used by the
installed SDK's `find_package()`:

```{literalinclude} ../../tests/c_samples/hello_world_source_link/CMakeLists.txt
:language: cmake
```

Key points:
- `OVPHYSX_SOURCE_DIR` points at the `ovphysx` directory in the repo
- Dependencies are auto-fetched at configure time (`OVPHYSX_FETCH_DEPS=ON` by default)
- `ovphysx_setup_source_link_runtime()` configures RPATH (Linux) or copies DLLs (Windows) so the executable finds `libovphysx` at runtime

## Build and Run

```bash
# Configure (first run fetches dependencies automatically)
cmake -S my_app -B my_app/_build

# Build
cmake --build my_app/_build --parallel 8

# Run (OVPHYSX_LIB anchors runtime config/schema/plugin discovery)
OVPHYSX_LIB=/path/to/ovphysx/_install/lib/libovphysx.so ./my_app/_build/my_app
```

The first configure takes a few minutes to fetch packman dependencies. Subsequent
configures are instant.

### Edit-Rebuild Cycle

After the initial build, editing ovphysx sources triggers recompilation of only
the changed files:

```bash
# Edit ovphysx source.
vim ovphysx/src/ovphysx/ovphysx.cpp

# Rebuild — only changed files recompile
cmake --build my_app/_build --parallel 8
```

## Runtime Plugin Discovery

The ovphysx runtime uses an embedded static Carbonite framework and loads PhysX,
USD, and bootstrap runtime plugins at startup. These plugins are not part of the
`add_subdirectory()` build; they come from the installed SDK's flattened runtime
layout. Build and install the SDK once, then point `OVPHYSX_LIB` at the installed
ovphysx library so the runtime can find the matching `config.toml`, plugins, and
USD schema paths:

```bash
# One-time setup
cd ovphysx
./build.sh
cmake --build _build --target install_sdk

# Then run your app with:
OVPHYSX_LIB=$(pwd)/_install/lib/libovphysx.so ./my_app/_build/my_app
```

On Windows, use `OVPHYSX_LIB=%cd%\_install\bin\ovphysx.dll`.

Pass `-DOVPHYSX_FETCH_DEPS=OFF` on subsequent configures if you do not want the
auto-fetch to re-check dependencies.

## Configuration Reference

| Variable | Default | Description |
|----------|---------|-------------|
| `OVPHYSX_SOURCE_DIR` | *(monorepo-relative)* | Path to the `ovphysx` directory |
| `OVPHYSX_FETCH_DEPS` | `ON` | Auto-fetch packman dependencies at configure time |
| `OVPHYSX_LIB` | *(env var)* | Path to the ovphysx shared library. Python loads this library directly; native/source-link runs use its directory to find `config.toml`, plugins, and USD schema paths. |

## Troubleshooting

If configure, build, or the first run fails, see
[common build failures](../local_development.md#troubleshooting) in the Local
Development guide.

## Result

After this tutorial, you can build your application against the ovphysx source
tree and iterate on ovphysx code without reinstalling the SDK after every edit.
For the complete sample code, refer to `tests/c_samples/hello_world_source_link/`.
