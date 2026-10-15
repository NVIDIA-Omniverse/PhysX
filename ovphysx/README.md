<!-- SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# ovphysx — USD-native physics simulation library, no Omniverse Kit installation required

> **`pip install ovphysx`** · Real-time, GPU-accelerated physics from a USD scene to ML-ready tensors in a handful of lines. Powered by the NVIDIA PhysX SDK.
>
> *Pre-release notice: ovphysx is a pre-release software and not yet mature. Parts of the API are still being completed and may change before 1.0.*

[![PyPI](https://img.shields.io/pypi/v/ovphysx)](https://pypi.org/project/ovphysx/)
[![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue)](https://pypi.org/project/ovphysx/)
[![Linux | Windows](https://img.shields.io/badge/platform-linux%20%7C%20windows-lightgrey)](https://pypi.org/project/ovphysx/)

ovphysx is a standalone library for USD-based physics simulation, offering a C API with Python bindings. It wraps NVIDIA PhysX, consumes caller-owned ovstage data, runs simulation, and exposes simulation output as Warp arrays in Python and DLTensor descriptors in C; no Omniverse installation is required.

## 1. What is ovphysx?

**ovphysx** is a standalone library for **USD-based physics simulation**, exposing a **C API with Python bindings**. It wraps NVIDIA PhysX SDK, consumes caller-owned ovstage data, runs the simulation, and exposes state through **Warp arrays in Python** and **DLTensor descriptors in C**, all with **no Omniverse Kit installation required**. It ships, loads, and links no OpenUSD library of its own: ovstage ingests the USD scene, and the application owns whatever USD it authors with. The result is USD-native physics in Python or C/C++ applications without a Kit installation.

**NVIDIA PhysX SDK** is the popular and stable **real-time physics simulation engine** underneath — the same GPU-accelerated rigid-body, articulation, and contact solver used across robotics, simulation, and interactive 3D for over a decade. PhysX is open source under Apache 2.0. ovphysx is the path that brings that engine to developers as a lightweight, USD-first, kitless library.

---

## 2. What functionalities are available, and who are the target users?

**What you can do with it:**

- **Load and simulate USD scenes through ovstage** -- populate an `ovstage.Stage`, attach and parse its initial ordinal, drain later changes with `update_from_ovstage(from_ordinal, to_ordinal)`, then `step(dt, ...)`.
- **Device-neutral output through Warp** — session reads (`PhysX.read()`, `PhysX.read_sync()`) expose same-device `warp.array` values on CPU and CUDA; Warp provides downstream DLPack interoperability with NumPy, PyTorch, and other frameworks. The native C API retains device-neutral DLTensor descriptors.
- **Session read/write** — `PhysX.read()` / `PhysX.read_sync()` and `PhysX.write()` / `PhysX.write_sync()` exchange per-attribute columns as Warp arrays. Tensor bindings (`RIGID_BODY_POSE` and the other packed types) still work and are deprecated.
- **Environment cloning for batched RL** — replicate environments for high-throughput reinforcement-learning workloads.
- **CPU or GPU simulation** — run with an NVIDIA GPU for acceleration, or fall back to CPU-only.
- **Standalone C/C++ SDK** — pre-built packages on GitHub Releases with ready-to-build samples (`find_package(ovphysx)`): hello world, session read/write, cloning, contacts, and **OmniPVD** recording for debugging.


**Who benefits:**

- **Robotics & RL developers** (Isaac Lab / Isaac Sim style workflows) who need fast, batched, headless physics with direct tensor access for policy training and inference.
- **Simulation & digital-twin engineers** who want USD-native physics without pulling in a full Omniverse/Kit stack.
- **C/C++ application developers** integrating real-time physics into non-Python engines and tools via the standalone SDK.
- **Researchers and ML practitioners** who live in NumPy/PyTorch and want simulation state as plain tensors.

---

## 3. Documentation and reference links

- **User Guide & Tutorials:** <https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/index.html>
- **C API Reference:** <https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/api.html>
- **Python API Reference:** <https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/python_api.html>
- **PhysX project home:** <https://nvidia-omniverse.github.io/PhysX/>
- **Source (GitHub):** <https://github.com/NVIDIA-Omniverse/PhysX/tree/main/ovphysx>
- **PyPI:** <https://pypi.org/project/ovphysx/>
- **Pre-built C/C++ SDK (Releases):** <https://github.com/NVIDIA-Omniverse/PhysX/releases>
- **Community & support:** [Discussions](https://github.com/NVIDIA-Omniverse/PhysX/discussions) · [Issues](https://github.com/NVIDIA-Omniverse/PhysX/issues) · [#physics Discord](https://discord.com/invite/XWQNJDNuaC)

---

## 4. System requirements

- Python **3.10+**
- **Linux** (x86_64, aarch64) or **Windows** (x86_64)
- **x86_64 CPUs must support AVX** (Advanced Vector Extensions). Pre-built Linux and Windows x86_64 binaries are compiled with AVX enabled and do not include a non-AVX fallback. `ovphysx_initialize()` / creating a Python `PhysX()` instance fails fast with a clear error when AVX is unavailable. Linux aarch64 wheels are unaffected. Check Linux x86_64 hosts with `grep -qw avx /proc/cpuinfo`; on Windows, confirm AVX support in your CPU specifications.
- NVIDIA GPU + driver **recommended** for acceleration (CPU-only simulation also supported)
- **A writable Warp cache directory.** Python `PhysX.read()` and `PhysX.write()`
  create [Warp](https://github.com/NVIDIA/warp) arrays, initializing Warp on first
  use. Warp initializes its cache at a platform-specific default location. If
  that location is not writable, initialization can fail with `PermissionError`
  or `OSError`. Set `WARP_CACHE_PATH` to a writable directory before Warp is
  initialized, for example before the first `read()`/`write()` call.

---

## 5. Licensing

- **Source code:** Apache License 2.0 — permissive, free for commercial and non-commercial use.
- **Pre-built binaries** (SDK packages and Python wheels): distributed under the **NVIDIA Omniverse License**.

> ovphysx is pre-release; APIs may change before 1.0. Applications install
> `physx-usd-schemas` separately and register it before USD population or
> schema-registry access. Scenes using `newton:*` attributes also need
> `newton-usd-schemas`. See [Physics Schemas](docs/physics_schemas.md).

## Quick Start

```bash
pip install ovphysx physx-usd-schemas newton-usd-schemas
```

`newton-usd-schemas` is the Newton USD schema the snippet below registers next
to the external PhysX schemas; scenes that author no `newton:*` attributes can leave it out
and drop the `newton_schema_root()` entry.

```python
from pathlib import Path

import ovphysx
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
import ovstage
from ovphysx import PhysX

usd_path = (
    Path(ovphysx.__file__).resolve().parent
    / "samples"
    / "data"
    / "simple_physics_scene.usda"
)
if not usd_path.is_file():
    raise FileNotFoundError(f"ovphysx sample data is missing: {usd_path}")

# Register external PhysX and Newton schemas before the first population.
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

Applications that own an `ovstage.Stage` attach it directly and push committed
ovstage ranges into PhysX explicitly. This helper takes a populated Stage whose
initial ordinal and later edit range have already been sealed:

```python
from ovphysx import PhysX
import ovstage


def step_ovstage_range(
    stage: ovstage.Stage, initial_ordinal: int, from_ordinal: int, to_ordinal: int
) -> None:
    physx = PhysX()
    try:
        # Attach parses the initial snapshot at initial_ordinal.
        physx.wait_task(physx.attach_ovstage(stage, read_ordinal=initial_ordinal))
        # Drain only later application-authored edits.
        physx.wait_task(physx.update_from_ovstage(from_ordinal, to_ordinal))
        physx.step_sync(1.0 / 60.0)
    finally:
        physx.destroy()
```

## Environment Cloning

Clone environments for batched reinforcement-learning workloads:

```python
from pathlib import Path

import ovphysx
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
from ovphysx import PhysX
from ovphysx.types import ObjectScope, SimObjectType
import ovstage

usd_path = (
    Path(ovphysx.__file__).resolve().parent
    / "samples"
    / "data"
    / "basic_simulation.usda"
)
if not usd_path.is_file():
    raise FileNotFoundError(f"ovphysx sample data is missing: {usd_path}")

PhysX.set_cpu_mode(True)
physx = PhysX()
# Register the codeless PhysX schemas and the Newton schema before the first population call.
ovstage.population.register_usd_schemas(
    [*[str(p) for p in physx_schema_paths()], str(ovphysx.newton_schema_root())]
)
stage = ovstage.Stage("scene")
ovstage.population.open_usd(
    stage, str(usd_path), ordinal=1, domains=ovstage.PopulationDomain.ALL
)
# attach_ovstage() reads at a sealed ordinal.
stage.advance_write_floor(ordinal=1).wait()
physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))
physx.wait_all()

# Clone before warmup or the first simulation step. clone() returns a Task
# receipt, not the number of copies; wait_task() collects its outcome.
targets = ["/World/envs/env1", "/World/envs/env2", "/World/envs/env3"]
clone_task = physx.clone("/World/envs/env0", targets)
physx.wait_task(clone_task)

# Read rigid-body positions from the source and every clone.
# CPU mode aliases the read buffer. Copy the values and drop the read
# before destroy(); a live borrow raises and skips teardown.
with physx.read_sync(SimObjectType.RIGID_BODY, ["position"], scope=ObjectScope.ALL) as result:
    positions = [group.tensors[0].numpy().copy() for group in result.groups]
del result

physx.wait_task(physx.detach_ovstage())
physx.destroy()
stage.destroy()
```

## C/C++ SDK

A standalone C SDK is available for integration into non-Python applications.
Source builds fetch the pinned native ovstage archive automatically. For the prebuilt
SDK, download OVPhysX from the
[PhysX GitHub Releases](https://github.com/NVIDIA-Omniverse/PhysX/releases)
page and the matching native OVStage archive from the
[OVStage GitHub Releases](https://github.com/NVIDIA-Omniverse/ovstage/releases)
page. Extract them as separate package roots; do not overlay their files.

After extracting the SDK, install the external provider as described in
[Physics Schemas](docs/physics_schemas.md#installing-the-external-physx-schemas)
and set `PHYSX_USD_SCHEMAS_ROOT`. Then build and run a bundled sample:

```bash
# /path/to/ovphysx-sdk is the extracted SDK package (pre-built binaries from GitHub Releases)
cmake -B build -S /path/to/ovphysx-sdk/samples/c_samples/hello_world_c \
  -DCMAKE_PREFIX_PATH="/path/to/ovphysx-sdk;/path/to/ovstage"
cmake --build build
./build/hello_world_c
```

The SDK includes ready-to-build samples in `samples/c_samples/` covering core
workflows (hello world, session read/write, cloning, contacts, OmniPVD recording).
Each sample has its own `CMakeLists.txt` that uses `find_package(ovphysx)`.

## Documentation

- [User Guide & Tutorials](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/index.html)
- [C API Reference](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/api.html)
- [Python API Reference](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/python_api.html)

## Requirements

- Python 3.10+
- Linux (x86_64, aarch64) or Windows (x86_64)
- **x86_64 only:** CPU must support AVX (see [System requirements](#4-system-requirements) above). Linux aarch64 is unaffected. ovphysx fails fast at initialize when AVX is missing.
- NVIDIA GPU + driver recommended (CPU-only simulation also supported)


## Links

- [PyPI](https://pypi.org/project/ovphysx/)
- [Documentation](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/index.html)
- [Issues](https://github.com/NVIDIA-Omniverse/PhysX/issues)
- [Discord](https://discord.com/invite/XWQNJDNuaC)

## Working in this repository

This section is for developers who build, test, or modify ovphysx from source.

### Prerequisites

- A Windows (x86_64) or Linux (x86_64, aarch64) development environment
- CMake 3.22+ on Linux, CMake 4.1+ on Windows
- C++17 compatible compiler (GCC/Clang on Linux, MSVC 2019+ on Windows). On Linux, use a compiler from the tested matrix in the PhysX SDK [Linux platform readme](https://github.com/NVIDIA-Omniverse/PhysX/blob/main/physx/documentation/platformreadme/linux/README_LINUX.md) rather than the newest available — a too-new GCC can exceed the CUDA host-compiler ceiling.
- **On Linux,** `binutils` (`strip`, `readelf`). The install and wheel steps use `readelf` to detect unstripped ELF binaries and `strip` to remove those symbols. `readelf` also verifies the ABI baseline. `binutils` usually arrives with the compiler.
- UV (https://docs.astral.sh/uv/getting-started/installation/) for Python management
- CUDA Toolkit and a compatible NVIDIA driver for GPU simulation (CPU-only builds do not need CUDA). Match the version in the PhysX SDK [Linux platform readme](https://github.com/NVIDIA-Omniverse/PhysX/blob/main/physx/documentation/platformreadme/linux/README_LINUX.md); if `nvcc` is not on `PATH`, CMake fails with `No CMAKE_CUDA_COMPILER could be found`.
- **Linux ABI baseline:** the SDK and wheel target a glibc 2.35 (`manylinux_2_35`) baseline, enforced by a readelf check during install/wheel/validate. On a newer distro (glibc > 2.35) that check fails fast; pass `SKIP_GLIBC_CHECK=ON` for local development, e.g. `cmake -DSKIP_GLIBC_CHECK=ON -P scripts/install.cmake`. Meeting that baseline is the build environment's job, and the source tree ships
  no container image for it: on Linux the compile uses whatever toolchain it runs under, so an Ubuntu
  22.04 (or equivalent glibc 2.35) environment has to be arranged.
- On Windows, `build.bat` defaults to the Visual Studio generator; `set GENERATOR=ninja` to opt into Ninja.

### Build

Run from the `ovphysx/` directory of the source checkout. On Linux:

```bash
./build.sh
```

On Windows, use PowerShell:

```powershell
.\build.bat
```

Install into `_install`:

```bash
cmake -P scripts/install.cmake
```

Build a Python wheel:

```bash
cmake -P scripts/build_wheel.cmake
```

### Build options

| Flag | Long | Description |
|------|------|-------------|
| `-c` | `--clean` | Clean artifacts only (no build) |
| `-x` | `--rebuild` | Clean then build |
| `-d` | `--debug` | Build debug configuration |
| `-r` | `--release` | Build release configuration (default) |
| `-t` | `--target <name>` | Build a specific CMake target |
| `-g` | `--generate` | Configure only (no build) |
| | `--devphysx` | Build PhysX SDK from source |
| | `--devschema` | Build and select a local schema provider wheel for tests/samples |
| | `-DOVPHYSX_USE_RELEASE_RUNTIME_DEPS=ON` | Use release runtime dependencies for Debug compilation (default and required) |

Source builds still fetch a namespaced monolithic OpenUSD package as a build-time
dependency: the ovruntime subproject resolves its own USD dependency. The Python
tests get their python USD from stock pip `usd-core`: the main session's one `pxr`
user opens its stage in a clean subprocess, and the pure-`pxr` `ovphysx.utils` suite
runs in its own pytest process, so a stock build is never resident next to ovstage;
no internal USD monolith is fetched. ovphysx itself links, loads, and pins no USD, so nothing in the product
constrains that choice.

`libovphysx` and `libovphysx_internal` link no USD library, and neither the SDK
nor the wheel ships one. ovstage ingests USD scenes through its own internal
namespaced OpenUSD runtime, and the application owns whatever USD it authors
with. There is no USD-linked variant; ovphysx attaches only through ovstage,
and exactly one USD image is ever loaded in the process.

**Changing `--devphysx` requires a clean rebuild.** It selects a native build
flavor, and incremental builds across that change are not supported. For example, `--devphysx` builds PhysX from the local
`physx/` source tree and caches `PHYSX_SDK_DIR` pointing at it; a later configure
without `--devphysx` keeps that cached path, so the `PhysXGpu_64.dll` copy step
still looks under the source tree instead of the packaged SDK and fails with
"No such file or directory". Pass `--rebuild` whenever the `--devphysx` setting
differs from the previous build — for instance `build.sh --devphysx` followed by
`build.sh --rebuild`. Switching only `--devschema` selects a
different external provider wheel and requires no clean rebuild.

If the build or first run fails, see
[Troubleshooting: common build failures](docs/local_development.md#troubleshooting).

### Debug build

Build with debug symbols from `ovphysx/`. On Linux:

```bash
./build.sh --debug
```

On Windows, use PowerShell:

```powershell
.\build.bat --debug
```

Install, wheel, and test commands must also specify Debug to match the build:

```bash
cmake -DBUILD_TYPE=Debug -P scripts/install.cmake
cmake -DBUILD_TYPE=Debug -P scripts/build_wheel.cmake
cmake -DBUILD_TYPE=Debug -P scripts/validate_all.cmake
```

Debug builds skip symbol stripping so that debuggers can resolve source locations.

Debug compilation uses Release runtime dependencies because the published
OVStage package supplies only a Release resolver/client runtime. A true-Debug
runtime build is unsupported; explicitly setting
`-DOVPHYSX_USE_RELEASE_RUNTIME_DEPS=OFF` fails at configuration time instead of
silently mixing Release OVStage binaries with Debug USD/TBB dependencies.

**Windows limitation:** Debug builds compile successfully on Windows, but tests are still failing. Full Windows debug support is planned for a future release.

### Testing

Run the full suite (build + install + wheel + all tests):

```bash
cmake -P scripts/validate_all.cmake
```

This builds, installs, creates the wheel, and runs all test suites. `cmake --build _build --target validate_all` is that install, wheel, and test run without calling `build.cmake` again. It still compiles missing or stale test dependencies, and the source-link and C++ sample tests compile during the run. The machine needs the compiler that configured `_build`.

```bash
cmake --build _build --target validate_all        # install + wheel + all tests
cmake --build _build --target validate_runtime    # runtime tests only
cmake --build _build --target validate_wheel      # wheel tests only
```

Or run specific suites with dedicated scripts in `scripts/`:

- `scripts/test_cpp.cmake`
- `scripts/test_python_runtime.cmake`
- `scripts/test_cpp_samples.cmake`
- `scripts/test_python_wheel.cmake`
- `scripts/test_python_samples.cmake`

Advanced: CMake presets (after `_build` exists and dependencies are fetched, e.g. via `build.sh`):

```bash
cmake --preset host-release                       # configure _build (preset cache vars)
cmake --build --preset validate-all               # builds validate_all in _build (validate-all preset → host-release binaryDir)
```

**CI vs local:** `validate_all.cmake` reproduces the full build + test pipeline locally.
CI runs the same build/test steps plus extra validation (`ci_validate.cmake`), docs build,
and packaging. The SPDX legal-blurb check is a separate `lint-ovphysx` job; run it locally
with `./repo.sh format --legal-only --verify`.

### Source-tree sample workflow

In the source repository, samples live under:

- `tests/c_samples/`
- `tests/python_samples/`

Python sample run with a locally built wheel:

```bash
cd tests/python_samples
uv venv
uv sync
uv pip install ../../_dist/ovphysx-*.whl
uv run --no-sync python hello_world.py
```

### Runtime library overrides for development

When developing locally, you can override runtime loading:

- `OVPHYSX_LIB` to select an ovphysx shared library. Python loads that library directly; native/source-link runs use its directory to find the plugins; schema resources come from the external provider.

### Registering the PhysX USD schemas

Applications install `physx-usd-schemas` separately and register both
resource directories before the first ovstage population call. Python examples
import the recipe from `ovphysx.samples.python_samples.schema_support`. Copying
`schema_support/` alongside a sample remains the alternative when adapting the
recipe outside the wheel. Native
examples use `samples/c_samples/common/schema_support.h` and
`PHYSX_USD_SCHEMAS_ROOT`, pointing at the independently extracted provider wheel.
Neither discovery route imports USD. Missing or incomplete providers fail before
population. See [Physics Schemas](docs/physics_schemas.md) for installation,
relocation, offline deployment, and stock-USD authoring.

ovphysx also reads the Newton USD schema's `newton:*` attributes (for example the
`NewtonJointAPI` velocity limit, armature, friction and damping, which apply to
every axis of a joint; `PhysxJointAxisAPI` values take precedence), but does not
ship that schema. Install it
(`pip install newton-usd-schemas`, or download it from
<https://github.com/newton-physics/newton-usd-schemas>) and register it in the
same call, `ovphysx.newton_schema_root()` in Python. Population drops every
`newton:*` attribute otherwise, and `PhysX.attach_ovstage()` warns when the
installed package was not registered before the first population.

### Development mode (`OVPHYSX_LIB`)

Development mode is intended for repository workflows and local iteration.
`OVPHYSX_LIB` points at an ovphysx shared library. In Python wheel workflows,
that is the library loaded by `ctypes`. In native/source-link workflows, the
executable may already load `libovphysx` via RPATH or DLL copy; `OVPHYSX_LIB`
still anchors runtime discovery. ovphysx looks for plugins in `plugins/` next to
the library directory or one level above it. Schema discovery uses the
separate application-owned provider installation.

- For installed SDK layouts, use `_install/lib/libovphysx.so` on Linux or `_install/bin/ovphysx.dll` on Windows.
- For raw build-tree layouts, ensure a compatible `plugins/` layout is present relative to the selected library.
- For source-link sample runs, prefer the installed SDK library so plugin discovery uses `_install/plugins`; install the schema provider separately.

### Dependency download

Dependencies auto-download during build (carb_sdk_plugins, omni_physics_dev, USD, etc.) by invoking packman and building the required PhysX runtime extensions.

Centralized extension cache in `_build/lib/deps/`, which is then installed into `_install/plugins/`.
Wheel packaging mirrors the install layout under `ovphysx/lib/`, `ovphysx/plugins/`, and `ovphysx/schemas/`;
Python exposes the session read/write API and retains deprecated tensor bindings;
the legacy `ovphysx.tensors` compatibility layer is no longer shipped.

### Architecture and deep dives

See `docs/` for architecture, settings, internal sidecar behavior, and versioning details.
