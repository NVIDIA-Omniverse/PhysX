<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Quickstart

ovphysx is an SDK for ovstage/USD-based physics simulation, available as a Python wheel and a C/C++ package.
This guide shows how to get started with both. You learn the required SDK layout, CMake configuration, runtime setup, and where to go next.

## Prerequisites

For both Python and C:

- A USD stage to populate. Every ovphysx package ships ready-to-run example
  stages, so you do not need to author one:
  - **Wheel:** `ovphysx/samples/data/` inside the installed Python package
  - **C/C++ SDK:** `<sdk-root>/samples/data/`
  - **Repository checkout:** `tests/data/`

  `simple_physics_scene.usda` (rigid body and ground plane) and
  `links_chain_sample.usda` (articulation) are good starting points. The Python
  example in [Quick Start by Language](#quick-start-by-language) resolves the
  first stage from the installed wheel.
- **x86_64 only:** a CPU with **AVX** (Advanced Vector Extensions). Pre-built ovphysx
  x86_64 binaries require AVX; `ovphysx_initialize()` fails fast when AVX is unavailable.
  On Linux x86_64, check with `grep -qw avx /proc/cpuinfo`. Linux aarch64 wheels are unaffected.
- **GPU simulation (optional):** prebuilt ovphysx wheels and C/C++ SDKs need a
  CUDA-capable NVIDIA driver at runtime, but do not require a CUDA Toolkit
  installation. GPU dynamics requires a Volta (SM 7.0) or newer GPU and a
  driver compatible with CUDA 12.8; use NVIDIA's
  [CUDA 12.8 corresponding-driver table](https://docs.nvidia.com/cuda/archive/12.8.0/cuda-toolkit-release-notes/index.html#id6)
  for x86_64. On Linux aarch64, use the platform's CUDA 12.8-compatible driver.
  The CUDA Toolkit is needed only to build GPU support from source; refer to
  [Local Development](../local_development.md). CPU-only simulation needs
  neither an NVIDIA driver nor the CUDA Toolkit.

## Quick Start by Language

### Python

Install the wheel:

```bash
pip install ovphysx physx-usd-schemas newton-usd-schemas
```

`pip` installs the OVStage version required by ovphysx automatically.

`newton-usd-schemas` is the Newton USD schema the snippet below registers next
to the external PhysX schemas; scenes that author no `newton:*` attributes can leave it out
and drop the `newton_schema_root()` entry.

```python
from pathlib import Path

import ovstage
import ovphysx
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx import PhysX

usd_path = (
    Path(ovphysx.__file__).resolve().parent
    / "samples"
    / "data"
    / "simple_physics_scene.usda"
)
if not usd_path.is_file():
    raise FileNotFoundError(f"ovphysx sample data is missing: {usd_path}")

# Register external PhysX and Newton schemas before the first USD schema read (population or export).
ovstage.population.register_usd_schemas(
    [*[str(p) for p in physx_schema_paths()], str(ovphysx.newton_schema_root())]
)
stage = ovstage.Stage("scene")
ovstage.population.open_usd(
    stage, str(usd_path), ordinal=1, domains=ovstage.PopulationDomain.ALL
)
# attach_ovstage() reads at a sealed ordinal.
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))
physx.step_sync(1.0 / 60.0)
physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
```

`domains` is an OR-combinable bitmask whose ovstage default (`RENDERING`) omits
physics. For arbitrary USD prefer `ALL` (equivalently `PHYSICS | RENDERING`);
`PHYSICS` alone is only safe when the content is known not to put physics under
native scene-graph instances — refer to
[Population domains](../ovstage_integration.md#population-domains).

### C/C++

Download the ovphysx SDK package from the [GitHub Releases](https://github.com/NVIDIA-Omniverse/PhysX/releases) page and extract it to a local path.
You also need CMake 3.16 or newer and a C or C++ compiler.

`find_package(ovphysx)` uses a compatible OVStage already on
`CMAKE_PREFIX_PATH`, or downloads the pinned native archive, verifies its SHA-256,
and caches it per user. Downloading needs Python 3.8+ on `PATH`.

```bash
cmake -B build -S . -DCMAKE_PREFIX_PATH="/path/to/ovphysx"
```

To supply OVStage yourself, use
`-DCMAKE_PREFIX_PATH="/path/to/ovphysx;/path/to/ovstage"`. Keep the two package
roots separate. `-DOVPHYSX_FETCH_OVSTAGE=OFF` disables downloading;
`-DOVPHYSX_OVSTAGE_CACHE_DIR=<dir>` selects the cache location. For manual
provisioning, the SDK records its pinned release in `ovphysx_OVSTAGE_VERSION`
in `lib/cmake/ovphysx/ovphysxConfig.cmake`. CMake accepts a compatible supplied
package as defined by OVStage's package-version metadata.

Install the external schema provider and set `PHYSX_USD_SCHEMAS_ROOT` before
running a native sample; see [Physics Schemas](../physics_schemas.md#installing-the-external-physx-schemas).

**SDK Directory Layout**

```text
ovphysx/
├── SKILLS.md             # Skills index
├── skills/               # Agent skills runbooks
├── include/ovphysx/     # Public headers (ovphysx.h, ovphysx_types.h, and related headers)
├── lib/                 # Shared libraries and CMake package config
│   └── cmake/ovphysx/   # find_package(ovphysx) support
├── scripts/             # fetch_ovstage_release.py, used by the OVStage bootstrap
├── plugins/             # Carbonite and PhysX runtime plugins
├── samples/             # CI-tested C sample source + USD data
├── docs/                # Documentation (HTML + Markdown)
├── LICENSE.txt
└── ovstage-THIRD-PARTY-NOTICES.txt
```

**Build Your First App**

**CMakeLists.txt**

The SDK ships a complete, CI-tested sample. Its CMake project links both the
ovphysx and ovstage companion targets:

```{literalinclude} ../../tests/c_samples/hello_world_c/CMakeLists.txt
:language: cmake
```

#### Source

The sample creates and populates an ovstage instance, attaches it with the
sealed read ordinal, waits for the physics step, and cleans up in lifetime-safe
order. Its `ovphysx_sample_attach_usd_with_ovstage()` helper (from
`samples/c_samples/common/ovstage_sample.h`) first registers with ovstage the
codeless PhysX USD schemas from physx-usd-schemas, passing both provider resource directories to
`ovstage_population_register_usd_schemas()` before it populates the stage;
ovphysx never registers them itself, and the registration must precede the first
population call in the process:

```{literalinclude} ../../tests/c_samples/hello_world_c/main.c
:language: c
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

**Configure and Build**

Replace `/path/to/ovphysx` with the extracted SDK root:

```bash
cmake -B build -S /path/to/ovphysx/samples/c_samples/hello_world_c \
  -DCMAKE_PREFIX_PATH="/path/to/ovphysx"
cmake --build build --config Release
```

## Runtime Libraries for C and C++

### Linux

The CMake config bakes RPATH into your executable automatically, covering both the
OVPhysX libraries and the resolved OVStage runtime directories. No
`LD_LIBRARY_PATH` manipulation is needed. Run the binary directly.

### Windows

Use the `ovphysx_copy_runtime_dlls(<target>)` helper provided by the CMake package
(already shown in the `CMakeLists.txt` example).
This copies the ovphysx DLLs and the `plugins/` tree next to your executable.
Install the schema provider separately and set `PHYSX_USD_SCHEMAS_ROOT` to its
current directory before running a native sample.

OVStage is deliberately *not* copied: its libraries discover their plugin tree
relative to their own location, so they must stay in their own package and be
reached through `PATH` instead. The CMake package knows where the resolved OVStage
is, so you do not have to name the path yourself:

```cmake
ovphysx_copy_runtime_dlls(my_app)
ovphysx_set_ovstage_runtime_env(my_app)
```

That sets the Visual Studio debugger's `PATH` and generates `run_my_app.cmd`
beside the executable. When debugging, also set the working directory for
relative scene paths and provide `PHYSX_USD_SCHEMAS_ROOT` in the debugger
environment. If you would rather
build the environment yourself, `ovphysx_ovstage_runtime_path(<var>)` returns the
directories to prepend.

## Result

You now have a minimal C or C++ application that links against `ovphysx::ovphysx`, consumes an application-owned ovstage, and runs a simulation step.

## Next Steps

- Full C API reference: refer to the [C API Reference](../api.md)
- More tutorials: [Hello World](hello_world.md), [Tensor Bindings (deprecated)](tensor_bindings.md), [Contact Binding](contact_binding.md)
