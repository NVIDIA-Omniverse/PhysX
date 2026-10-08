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
pip install ovphysx
```

```python
from pathlib import Path

import ovstage
import ovphysx
from ovphysx import PhysX

usd_path = (
    Path(ovphysx.__file__).resolve().parent
    / "samples"
    / "data"
    / "simple_physics_scene.usda"
)
if not usd_path.is_file():
    raise FileNotFoundError(f"ovphysx sample data is missing: {usd_path}")

# ovphysx ships its PhysX USD schemas as codeless resources and never registers
# them itself; register them with ovstage before the first population call.
ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])
stage = ovstage.Stage("scene")
ovstage.population.open_usd(
    stage, str(usd_path), ordinal=1, domains=ovstage.PopulationDomain.PHYSICS
)
# attach_ovstage() reads at a sealed ordinal.
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.attach_ovstage(stage, read_ordinal=1)
physx.step_sync(1.0 / 60.0)
physx.detach_ovstage()
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

Repository source builds fetch ovstage automatically from public PyPI. The
manual download described in this section applies only if you use the prebuilt
ovphysx SDK.

**ovstage is not part of the SDK.** ovphysx binds to the application-supplied
ovstage, so the package ships no ovstage headers, library, or runtime. Download
the matching native archive for your platform from the
[ovstage GitHub Releases](https://github.com/NVIDIA-Omniverse/ovstage/releases)
page and extract it to a separate directory beside ovphysx. This release uses
OVStage `0.2.0.377349`, published under the `v0.2.0` release. Do not overlay the
two package trees.

`find_package(ovphysx)` pulls it in through `find_dependency(ovstage)`, so add
both roots to `CMAKE_PREFIX_PATH`. Both roots are required even if you never
call ovstage, because the public ovphysx headers `#include <ovstage/...>`.

**SDK Directory Layout**

```text
ovphysx/
├── SKILLS.md             # Skills index
├── skills/               # Agent skills runbooks
├── include/ovphysx/     # Public headers (ovphysx.h, ovphysx_types.h, and related headers)
├── lib/                 # Shared libraries and CMake package config
│   └── cmake/ovphysx/   # find_package(ovphysx) support
├── plugins/             # Carbonite and PhysX runtime plugins
├── schemas/physx/       # Codeless PhysX USD schemas (the application registers them)
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
codeless PhysX USD schemas that ovphysx ships, passing the root returned by
`ovphysx_get_codeless_schema_root()` to
`ovstage_population_register_usd_schemas()` before it populates the stage;
ovphysx never registers them itself, and the registration must precede the first
population call in the process:

```{literalinclude} ../../tests/c_samples/hello_world_c/main.c
:language: c
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

**Configure and Build**

Point `CMAKE_PREFIX_PATH` at both package roots and build the bundled sample.
Replace `/path/to/ovphysx` with your extracted SDK root (the directory
containing `include/`, `lib/`, and `plugins/`), and `/path/to/ovstage` with the
directory you extracted the ovstage archive into:

```bash
cmake -B build -S /path/to/ovphysx/samples/c_samples/hello_world_c \
  -DCMAKE_PREFIX_PATH="/path/to/ovphysx;/path/to/ovstage"
cmake --build build
```

## Runtime Libraries for C and C++

### Linux

The CMake config embeds the RPATH in your executable automatically.
No `LD_LIBRARY_PATH` manipulation is needed. Run the binary directly.

### Windows

Use the `ovphysx_copy_runtime_dlls(<target>)` helper provided by the CMake package
(already shown in the `CMakeLists.txt` example).
This copies the ovphysx DLLs, the `plugins/` tree, and the codeless schema tree
(`schemas/physx`, so `ovphysx_get_codeless_schema_root()` resolves in that layout)
next to your executable. Keep the
separate ovstage package intact and add its `bin`, `bin/plugins`,
`bin/plugins/omni.client.lib`, and `bin/plugins/omni.usd_resolver` directories
to the process `PATH` before launch.

```powershell
$env:OVSTAGE_ROOT = "C:\path\to\extracted\ovstage"
$env:PATH = "$env:OVSTAGE_ROOT\bin;$env:OVSTAGE_ROOT\bin\plugins;$env:OVSTAGE_ROOT\bin\plugins\omni.client.lib;$env:OVSTAGE_ROOT\bin\plugins\omni.usd_resolver;$env:PATH"
```

## Result

You now have a minimal C or C++ application that links against `ovphysx::ovphysx`, consumes an application-owned ovstage, and runs a simulation step.

## Next Steps

- Full C API reference: refer to the [C API Reference](../api.md)
- More tutorials: [Hello World](hello_world.md), [Tensor Bindings (deprecated)](tensor_bindings.md), [Contact Binding](contact_binding.md)
