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

The native SDK automatically resolves OVStage. Build and run a bundled sample:

```bash
# Run from the extracted SDK root directory
python samples/c_samples/common/fetch_physx_schemas.py --dest ../physx-schemas
export PHYSX_USD_SCHEMAS_ROOT="$(cd ../physx-schemas && pwd)"
cmake -B build -S samples/c_samples/hello_world_c -DCMAKE_PREFIX_PATH="$PWD"
cmake --build build
./build/hello_world_c
```

`find_package(ovphysx)` resolves OVStage itself: if your application already
provides an OVStage package, put it on `CMAKE_PREFIX_PATH` and that one is used;
otherwise OVPhysX downloads the pinned build, verifies it against a committed
SHA-256, and caches it per user so later configures are offline.

| Option | Effect |
| --- | --- |
| `-DCMAKE_PREFIX_PATH="$PWD;/path/to/ovstage"` | Use your own OVStage. Nothing is downloaded. |
| `-DOVPHYSX_FETCH_OVSTAGE=OFF` | Never download. Configure fails if OVStage is not already findable. |
| `-DOVPHYSX_OVSTAGE_CACHE_DIR=<dir>` | Put the downloaded package somewhere specific. |

Downloading needs Python 3.8+ on `PATH`. Keep the OVStage package root separate
from the OVPhysX one — do not overlay the two trees. For manual provisioning,
the SDK records its pinned release in `ovphysx_OVSTAGE_VERSION` in
`lib/cmake/ovphysx/ovphysxConfig.cmake`. CMake accepts a compatible supplied
package as defined by OVStage's package-version metadata.

On Windows, set `$env:PHYSX_USD_SCHEMAS_ROOT` to the extracted schema directory.
OVStage loads its plugins relative to its own location, so its
directories must be on `PATH` when the sample runs. `ovphysx_set_ovstage_runtime_env(<target>)`
sets the Visual Studio debugger's `PATH` and generates `run_<target>.cmd` beside
the executable. When debugging, also set the working directory for relative
scene paths and provide `PHYSX_USD_SCHEMAS_ROOT` in the debugger environment.
The bundled samples call it. With Visual Studio, build with
`cmake --build build --config Release`, then run
`build\Release\run_hello_world_c.cmd`. With Ninja, the launcher is directly in
`build\`. `ovphysx_ovstage_runtime_path(<var>)` returns the paths for custom
launchers. Linux samples use RPATH.

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
- OVStage is an application-owned dependency. An application that already has one
  keeps it — put it on `CMAKE_PREFIX_PATH` and OVPhysX will not fetch a second.
  Keep its package root separate from the OVPhysX one.
- This SDK ships no OpenUSD libraries. ovstage ingests USD scenes through its
  own internal namespaced OpenUSD runtime; the application owns whatever USD it
  authors with. Install `physx-usd-schemas` separately and register both
  resource directories with `ovstage_population_register_usd_schemas()` before
  the first population call, as
  `samples/c_samples/common/ovstage_sample.h` does. Scenes that author Newton
  `newton:*` attributes also need the Newton USD schema
  (https://github.com/newton-physics/newton-usd-schemas, or the
  `newton-usd-schemas` PyPI package) registered in the same call; the SDK does
  not ship it and the C API does not check for it.
- Python session reads (`PhysX.read()`, `read_sync()`) expose `warp.array` values on CPU and CUDA.
  Warp owns downstream DLPack interoperability with other frameworks; the native
  C API and compatibility tensor-binding surface retain DLTensor descriptors.
