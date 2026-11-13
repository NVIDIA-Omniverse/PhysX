<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Basic Workflow C Sample

This sample exposes the public API sequence for initialization, schema
registration, USD population, write-floor completion, attach, synchronous step,
detach, and cleanup. Unlike `hello_world_c`, it keeps the population and
attachment calls visible in `main.c` instead of wrapping them in sample helpers.
The shared helper only registers the external PhysX schemas.

The SDK ships this directory at `samples/c_samples/basic_workflow_c/`, alongside
`samples/c_samples/common/` and `samples/data/`. Its CMake project uses those
installed paths directly. The scene path is resolved at configure time, so the
executable does not depend on the current working directory.

## Configure and build

Run these commands from the extracted OVPhysX SDK root. Replace the two package
paths with the extracted OVPhysX SDK and matching native OVStage package:

```bash
cmake -S samples/c_samples/basic_workflow_c -B build-basic-workflow -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="/path/to/ovphysx;/path/to/ovstage"
cmake --build build-basic-workflow --config Release
```

For Ninja, add `-G Ninja` to the configure command. On Windows, use Visual Studio
Developer PowerShell configured for x64 with `-Arch amd64 -HostArch amd64` so the
compiler, headers, and libraries are available; see Microsoft's
[developer shell setup](https://learn.microsoft.com/en-us/visualstudio/ide/reference/command-prompt-powershell#target-architecture-and-host-architecture).
Use a fresh build directory when changing generators.

`find_package(ovphysx)` uses a compatible OVStage package from `CMAKE_PREFIX_PATH`.
If none is available, it downloads and caches the pinned package; this requires
Python 3.8+ and network access on the first configure. To use that bootstrap, set
`CMAKE_PREFIX_PATH` to the OVPhysX SDK root alone. For offline setup, supply
OVStage separately and add `-DOVPHYSX_FETCH_OVSTAGE=OFF`.

## Run

Install the external schema provider as described in
[Physics Schemas](../../../docs/physics_schemas.md). Set `PHYSX_USD_SCHEMAS_ROOT`
to the extracted `physx-usd-schemas` wheel directory containing
`physx_usd_schemas/`. Then run from the SDK root:

Linux:

```bash
export PHYSX_USD_SCHEMAS_ROOT="/path/to/physx-schemas"
./build-basic-workflow/basic_workflow_c
```

Windows PowerShell:

```powershell
$env:PHYSX_USD_SCHEMAS_ROOT = "C:\path\to\physx-schemas"
```

- Windows, Visual Studio generator: `.\build-basic-workflow\Release\run_basic_workflow_c.cmd`.
- Windows, Ninja generator: `.\build-basic-workflow\run_basic_workflow_c.cmd`.

On Windows, `ovphysx_copy_runtime_dlls()` copies the OVPhysX DLLs and plugins
beside the executable. `ovphysx_set_ovstage_runtime_env()` generates
`run_basic_workflow_c.cmd`, which adds the resolved OVStage runtime directories
to `PATH` before starting the program. Use this launcher from a shell. Keep
OVStage in its own package so it can find its plugins.

The program exits with a nonzero status when initialization, population,
attachment, stepping, detach, or instance destruction fails.
