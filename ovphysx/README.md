# ovphysx — USD-native physics simulation library, no Omniverse Kit installation required

> **`pip install ovphysx`** · Real-time, GPU-accelerated physics from a USD scene to ML-ready tensors in a handful of lines. Powered by the NVIDIA PhysX SDK.
>
> *Pre-release notice: ovphysx is a pre-release software and not yet mature. Parts of the API are still being completed and may change before 1.0.*

[![PyPI](https://img.shields.io/pypi/v/ovphysx)](https://pypi.org/project/ovphysx/)
[![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue)](https://pypi.org/project/ovphysx/)
[![Linux | Windows](https://img.shields.io/badge/platform-linux%20%7C%20windows-lightgrey)](https://pypi.org/project/ovphysx/)

ovphysx is a standalone library for USD-based physics simulation, offering a C API with Python bindings. It wraps NVIDIA PhysX, consumes caller-owned ovstage data, runs simulation, and exchanges data via DLPack tensors; no Omniverse installation is required.

## 1. What is ovphysx?

**ovphysx** is a standalone library for **USD-based physics simulation**, exposing a **C API with Python bindings**. It wraps NVIDIA PhysX SDK, consumes caller-owned ovstage data, runs the simulation, and exchanges state with your code via **DLPack tensors**, all with **no Omniverse Kit installation required**. It packages its own OV-namespaced OpenUSD runtime, so you get USD-native physics in Python or C/C++ applications without a Kit installation.

**NVIDIA PhysX SDK** is the popular and stable **real-time physics simulation engine** underneath — the same GPU-accelerated rigid-body, articulation, and contact solver used across robotics, simulation, and interactive 3D for over a decade. PhysX is open source under BSD-3-Clause. ovphysx is the path that brings that engine to developers as a lightweight, USD-first, kitless library.

---

## 2. What functionalities are available, and who are the target users?

**What you can do with it:**

- **Load and simulate USD scenes through ovstage** -- populate an `ovstage.Stage`, attach and parse its initial ordinal, drain later changes with `update_from_ovstage(from_ordinal, to_ordinal)`, then `step(dt, ...)`.
- **Tensor data exchange via DLPack** — read/write simulation state (e.g. rigid-body poses) with same-device zero-copy access and transparent CPU/CUDA staging for NumPy, PyTorch, and other ML frameworks.
- **Tensor bindings** — bind to scene patterns (e.g. `RIGID_BODY_POSE`) to stream state in and out efficiently.
- **Environment cloning for batched RL** — replicate environments for high-throughput reinforcement-learning workloads.
- **CPU or GPU simulation** — run with an NVIDIA GPU for acceleration, or fall back to CPU-only.
- **Standalone C/C++ SDK** — pre-built packages on GitHub Releases with ready-to-build samples (`find_package(ovphysx)`): hello world, tensor bindings, cloning, contacts, and **OmniPVD** recording for debugging.


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

---

## 5. Licensing

- **Source code:** BSD-3-Clause License — permissive, free for commercial and non-commercial use.
- **Pre-built binaries** (SDK packages and Python wheels): distributed under the **NVIDIA Omniverse License**.

> **Note:** ovphysx is pre-release and not yet mature. When sharing a process with other OV USD-aware subsystems, register each subsystem's schema paths before the first USD stage or schema-registry access. Parts of the API may change before 1.0.

---

> **Note:** Pre-release notice: ovphysx is pre-release software and not yet mature. ovphysx packages an OV namespaced OpenUSD runtime; when sharing a process with other OV USD-aware subsystems, register each subsystem's schema paths before the first USD stage or schema-registry access. Parts of the API are still being completed and may change before 1.0.

## Quick Start

```bash
pip install ovphysx
```

```python
import ovstage
from ovphysx import PhysX

stage = ovstage.Stage("scene")
ovstage.population.open_usd(
    stage, "scene.usda", ordinal=1, domains=ovstage.PopulationDomain.PHYSICS
)

physx = PhysX()
physx.attach_ovstage(stage, read_ordinal=1)
physx.step(1.0 / 60.0)
physx.detach_ovstage()
physx.release()
stage.destroy()
```

Applications that own an `ovstage.Stage` attach it directly and push committed
ovstage ranges into PhysX explicitly:

```python
from ovphysx import PhysX

physx = PhysX()
# Attach parses the initial snapshot at initial_ordinal.
physx.attach_ovstage(stage, read_ordinal=initial_ordinal)
# Drain only later application-authored edits.
physx.update_from_ovstage(from_ordinal, to_ordinal)
physx.step(1.0 / 60.0)
physx.release()
```

## Environment Cloning

Clone environments for batched reinforcement-learning workloads:

```python
from ovphysx import PhysX
from ovphysx.types import TensorType
import numpy as np
import ovstage

PhysX.set_cpu_mode(True)
physx = PhysX()
stage = ovstage.Stage("scene")
ovstage.population.open_usd(
    stage, "scene.usda", ordinal=1, domains=ovstage.PopulationDomain.PHYSICS
)
physx.attach_ovstage(stage, read_ordinal=1)
physx.wait_all()

# Clone before GPU warmup or the first simulation step. clone() returns an
# operation index, not the number of copies.
targets = ["/World/envs/env1", "/World/envs/env2", "/World/envs/env3"]
clone_op = physx.clone("/World/envs/env0", targets)
physx.wait_op(clone_op)

# Read rigid body poses from the source and every clone.
pose_binding = physx.create_tensor_binding(
    pattern="/World/envs/env*/table",
    tensor_type=TensorType.RIGID_BODY_POSE,
)
poses = np.zeros(pose_binding.shape, dtype=np.dtype(str(pose_binding.dtype)))
pose_binding.read(poses)

pose_binding.destroy()
physx.detach_ovstage()
physx.release()
stage.destroy()
```

## C/C++ SDK

A standalone C SDK is available for integration into non-Python applications.
Source builds fetch OVStage automatically from public PyPI. For the prebuilt
SDK, download OVPhysX from the
[PhysX GitHub Releases](https://github.com/NVIDIA-Omniverse/PhysX/releases)
page and the matching native OVStage archive from the
[OVStage GitHub Releases](https://github.com/NVIDIA-Omniverse/ovstage/releases)
page. Extract them as separate package roots; do not overlay their files.

After extracting the SDK, you can build and run a bundled sample directly:

```bash
# /path/to/ovphysx-sdk is the extracted SDK package (pre-built binaries from GitHub Releases)
cmake -B build -S /path/to/ovphysx-sdk/samples/c_samples/hello_world_c \
  -DCMAKE_PREFIX_PATH="/path/to/ovphysx-sdk;/path/to/ovstage"
cmake --build build
./build/hello_world_c
```

The SDK includes ready-to-build samples in `samples/c_samples/` covering core
workflows (hello world, tensor bindings, cloning, contacts, OmniPVD recording).
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
- CMake 3.16+ on Linux, CMake 4.1+ on Windows
- C++17 compatible compiler (GCC/Clang on Linux, MSVC 2019+ on Windows). On Linux, use a compiler from the tested matrix in the PhysX SDK [Linux platform readme](https://github.com/NVIDIA-Omniverse/PhysX/blob/main/physx/documentation/platformreadme/linux/README_LINUX.md) rather than the newest available — a too-new GCC can exceed the CUDA host-compiler ceiling.
- **On Linux,** the `file` utility (the OS file-type tool, not CMake's `file()` command) and `binutils` (`strip`, `readelf`). The install and wheel steps use them to detect and strip unstripped ELF binaries and to verify the ABI baseline. `binutils` usually arrives with the compiler, but `file` is absent from minimal container images: `apt-get install file binutils`.
- UV (https://docs.astral.sh/uv/getting-started/installation/) for Python management
- CUDA Toolkit and a compatible NVIDIA driver for GPU simulation (CPU-only builds do not need CUDA). Match the version in the PhysX SDK [Linux platform readme](https://github.com/NVIDIA-Omniverse/PhysX/blob/main/physx/documentation/platformreadme/linux/README_LINUX.md); if `nvcc` is not on `PATH`, CMake fails with `No CMAKE_CUDA_COMPILER could be found`.
- **Linux ABI baseline:** the SDK and wheel target a glibc 2.35 (`manylinux_2_35`) baseline, enforced by a readelf check during install/wheel/validate. On a newer distro (glibc > 2.35) that check fails fast; pass `SKIP_GLIBC_CHECK=ON` for local development, e.g. `cmake -DSKIP_GLIBC_CHECK=ON -P scripts/install.cmake`.
- On Windows, `build.bat` defaults to the Visual Studio generator; `set GENERATOR=ninja` to opt into Ninja.

### Build

Build ovphysx from the repository root:

```bash
build.{bat|sh}
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
| | `--devschema` | Use locally-built physics schema |
| | `-DOVPHYSX_USE_RELEASE_RUNTIME_DEPS=ON` | Use release runtime dependencies for Debug compilation (default and required) |

Source builds use namespaced monolithic USD. There is no classic USD build
switch.

**Changing `--devphysx` or `--devschema` requires a clean rebuild.** The flag
combination selects a build flavor, and incremental builds across a flavor
change are not supported. For example, `--devphysx` builds PhysX from the local
`physx/` source tree and caches `PHYSX_SDK_DIR` pointing at it; a later configure
without `--devphysx` keeps that cached path, so the `PhysXGpu_64.dll` copy step
still looks under the source tree instead of the packaged SDK and fails with
"No such file or directory". Pass `--rebuild` whenever the flag combination
differs from the previous build — for instance `build.sh --devphysx` followed by
`build.sh --rebuild --devschema`.

If the build or first run fails, see
[Troubleshooting: common build failures](docs/local_development.md#troubleshooting).

### Debug build

Build with debug symbols for stepping through code with a debugger:

```bash
build.{bat|sh} --debug
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

This builds, installs, creates the wheel, and runs all test suites—the same phases as `cmake --build _build --target validate_all`, with `build.cmake` run first so configure and compile stay in sync. Use native targets when you have already run `build.sh` or `cmake -P scripts/build.cmake` and want only install, wheel, and tests.

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
CI runs the same build/test steps plus formatting checks (`ci_validate.cmake`), docs build,
and packaging.

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

- `OVPHYSX_LIB` to select an ovphysx shared library. Python loads that library directly; native/source-link runs use its directory to find `config.toml`, plugins, and USD schema paths.

### Sharing USD schema discovery with ovrtx

When ovphysx shares a process with another USD-aware OV subsystem such as ovrtx,
publish both subsystems' schema paths before either subsystem opens a USD stage:

```c
#include <ovphysx/ovphysx.h>
#include <ovrtx/ovrtx.h>

int main(void)
{
    ovphysx_register_schema_paths();
    ovrtx_register_schema_paths();
    return 0;
}
```

Python applications can call `ovphysx.register_schema_paths()` before native
ovphysx bootstrap. Standalone ovphysx applications do not need the explicit call;
`ovphysx_create_instance()` registers the ovphysx path automatically.

### Development mode (`OVPHYSX_LIB`)

Development mode is intended for repository workflows and local iteration.
`OVPHYSX_LIB` points at an ovphysx shared library. In Python wheel workflows,
that is the library loaded by `ctypes`. In native/source-link workflows, the
executable may already load `libovphysx` via RPATH or DLL copy; `OVPHYSX_LIB`
still anchors runtime discovery. ovphysx loads `config.toml` beside that library
and looks for plugins in `plugins/` next to the library directory or one level
above it.

- For installed SDK layouts, use `_install/lib/libovphysx.so` on Linux or `_install/bin/ovphysx.dll` on Windows.
- For raw build-tree layouts, ensure `config.toml` and a compatible `plugins/` layout are present relative to the selected library.
- For source-link sample runs, prefer the installed SDK library so config/schema/plugin discovery uses the flattened `_install/plugins` layout.

### Dependency download

Dependencies auto-download during build (kit-kernel, omni_physics_dev, USD, etc.) by invoking packman and building the required PhysX runtime extensions.

Centralized extension cache in `_build/lib/deps/`, which is then installed into `_install/plugins/`.
Wheel packaging mirrors the install layout under `ovphysx/lib/` and `ovphysx/plugins/`;
Python now exposes only TensorBindingsAPI and no longer ships the legacy `ovphysx.tensors`
compatibility layer.

### Architecture and deep dives

See `docs/` for architecture, settings, internal sidecar behavior, and versioning details.
