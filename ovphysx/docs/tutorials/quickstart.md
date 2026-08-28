<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# Quickstart

ovphysx is an SDK for ovstage/USD-based physics simulation, available as a Python wheel and a C/C++ package.
This guide shows how to get started with both. You learn the required SDK layout, CMake configuration, runtime setup, and where to go next.

## Prerequisites

For both Python and C:

- Prepare a simple USD file such as `scene.usda` for ovstage population.
- **x86_64 only:** a CPU with **AVX** (Advanced Vector Extensions). Pre-built ovphysx
  x86_64 binaries require AVX; `ovphysx_initialize()` fails fast when AVX is unavailable.
  On Linux x86_64, check with `grep -qw avx /proc/cpuinfo`. Linux aarch64 wheels are unaffected.
- Install CUDA Toolkit and a compatible NVIDIA driver (recommended, but can be skipped if only CPU simulation is used).

## Quick Start by Language

### Python

Install the wheel:

```bash
pip install ovphysx
```

```python
import ovstage
import ovphysx
from ovphysx import PhysX

stage = ovstage.Stage("scene")
ovstage.population.open_usd(
    stage, "scene.usda", ordinal=1, domains=ovstage.PopulationDomain.PHYSICS
)
# attach_ovstage() reads at a sealed ordinal.
stage.advance_write_floor(ordinal=1).wait()

physx = PhysX()
physx.attach_ovstage(stage, read_ordinal=1)
physx.step(1.0 / 60.0)
physx.detach_ovstage()
physx.release()
stage.destroy()
```

`domains` is an OR-combinable bitmask whose ovstage default (`RENDERING`) omits
physics. For arbitrary USD prefer `ALL` (equivalently `PHYSICS | RENDERING`);
`PHYSICS` alone is only safe when the content is known not to put physics under
native scene-graph instances — refer to
[Population domains](../ovstage_integration.md#population-domains).

### C/C++

Download the ovphysx SDK package from the [GitHub Releases](https://github.com/NVIDIA-Omniverse/PhysX/releases) page and extract it to a local path.
You also need a C or C++ compiler and CMake — 3.16 or newer on Linux, 4.1 or newer on Windows.

Repository source builds fetch OVStage automatically from public PyPI. The
manual download below is only for users of the prebuilt OVPhysX SDK.

**ovstage is not part of the SDK.** ovphysx binds to the application-supplied
ovstage, so the package ships no ovstage headers, library, or runtime. Download
the matching native archive for your platform from the
[OVStage GitHub Releases](https://github.com/NVIDIA-Omniverse/ovstage/releases)
page and extract it to a separate directory beside ovphysx. This release uses
OVStage `0.1.0.346039`, published under the `v0.1.0` release. Do not overlay the
two package trees.

`find_package(ovphysx)` pulls it in via `find_dependency(ovstage)`, so just add
both roots to `CMAKE_PREFIX_PATH`. This is required even if you never call ovstage because the
public ovphysx headers `#include <ovstage/...>`.

**SDK Directory Layout**

```text
ovphysx/
├── SKILLS.md             # Skills index
├── skills/               # Agent skills runbooks
├── include/ovphysx/     # Public headers (ovphysx.h, ovphysx_types.h, and related headers)
├── lib/                 # Shared libraries and CMake package config
│   └── cmake/ovphysx/   # find_package(ovphysx) support
├── plugins/             # Carbonite, PhysX, and USD runtime plugins
│   └── gpu/             # GPU-only plugins (loaded only when GPU is enabled)
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
order:

```{literalinclude} ../../tests/c_samples/hello_world_c/main.c
:language: c
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

**Configure and Build**

Point `CMAKE_PREFIX_PATH` at the SDK root (the directory containing `include/`,
`lib/`, and `plugins/`) and build the bundled sample:

```bash
cmake -B build -S /path/to/ovphysx/samples/c_samples/hello_world_c \
  -DCMAKE_PREFIX_PATH="/path/to/ovphysx;/path/to/ovstage"
cmake --build build
```

## Runtime Libraries (C/C++)

### Linux

The CMake config bakes RPATH into your executable automatically.
No `LD_LIBRARY_PATH` manipulation is needed. Run the binary directly.

### Windows

Use the `ovphysx_copy_runtime_dlls(<target>)` helper provided by the CMake package
(already shown in the `CMakeLists.txt` example).
This copies the OVPhysX DLLs and plugins next to your executable. Keep the
separate OVStage package intact and add its `bin`, `bin/plugins`,
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
- More tutorials: [Hello World](hello_world.md), [Tensor Bindings](tensor_bindings.md), [Contact Binding](contact_binding.md)
