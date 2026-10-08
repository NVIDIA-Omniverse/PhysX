<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# ovphysx SDK

ovphysx is an SDK for USD-based physics simulation with Warp-array output in
Python and DLTensor-based native interoperability, available as a Python wheel
and a C/C++ package.

- [Latest releases](https://github.com/NVIDIA-Omniverse/PhysX/releases)
- [Getting started](docs/tutorials/quickstart.md)
- [Full documentation](docs/index.md)

## Quick Start (C/C++)

The native OVPhysX package does not include OVStage. Download the matching
native archive from the
[OVStage GitHub Releases](https://github.com/NVIDIA-Omniverse/ovstage/releases)
page and extract it beside OVPhysX, without overlaying the two trees. This
release uses OVStage `0.2.0.377349`, published under the `v0.2.0` release. Then
build and run a bundled sample:

```bash
# Run from the extracted SDK root directory
export OVSTAGE_ROOT=/path/to/extracted/ovstage
cmake -B build -S samples/c_samples/hello_world_c \
  -DCMAKE_PREFIX_PATH="$PWD;$OVSTAGE_ROOT"
cmake --build build
./build/hello_world_c
```

On Windows, set the OVStage runtime paths in the shell that launches the sample:

```powershell
$env:OVSTAGE_ROOT = "C:\path\to\extracted\ovstage"
$env:PATH = "$env:OVSTAGE_ROOT\bin;$env:OVSTAGE_ROOT\bin\plugins;$env:OVSTAGE_ROOT\bin\plugins\omni.client.lib;$env:OVSTAGE_ROOT\bin\plugins\omni.usd_resolver;$env:PATH"
```

See `samples/c_samples/` for more examples. Each sample has its own `CMakeLists.txt` that uses `find_package(ovphysx)`.

## AI agent / LLM instructions

This SDK ships step-by-step playbooks (skills) and code samples.
Start by reading [SKILLS.md](SKILLS.md) in this directory for a skills index.

Key resources:
- Skills (agent playbooks): `SKILLS.md` and `skills/`
- C code samples: `samples/c_samples/`
- Sample USD data: `samples/data/`
- Markdown documentation: `docs/`
- C API header: `include/ovphysx/ovphysx.h`

Important notes:
- OVStage is an application-supplied dependency. Keep its extracted native
  package separate from the OVPhysX package.
- This SDK ships no OpenUSD libraries. ovstage ingests USD scenes through its
  own internal namespaced OpenUSD runtime; the application owns whatever USD it
  authors with. The PhysX USD schemas ship as codeless plugins under
  `schemas/physx/`; register them with
  `ovstage_population_register_usd_schemas()` (path from
  `ovphysx_get_codeless_schema_root()`) before the first population call, as
  `samples/c_samples/common/ovstage_sample.h` does.
- `PhysX.read()` and `read_tokens()` return `warp.array` values on CPU and CUDA.
  Warp owns downstream DLPack interoperability with other frameworks; the native
  C API and compatibility tensor-binding surface retain DLTensor descriptors.
