<!-- SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# ovphysx Scripts

Cross-platform CMake scripts for building, testing, packaging, and installing ovphysx.

## Full local validation (one command)

```bash
# from ovphysx/
cmake -P scripts/validate_all.cmake
```

That runs, in order: incremental **build** (via `build.cmake`—configure, compile libovphysx
and the required ovruntime targets), **install** into `_install/`, **wheel** build into
`_dist/`, then **all** C++ and Python test suites (unit tests, samples, wheel smoke, etc.).
For a stage-by-stage breakdown, optional native CMake targets, and CI parity, see
**`validate_all.cmake`** under Test scripts below.

## Build scripts

### `build.cmake`
Builds the SDK: fetches deps, configures CMake in `_build/`, and compiles libovphysx plus
the required ovruntime subproject targets.

Most developers invoke **`build.sh`** (Linux) or **`build.bat`** (Windows) instead of
calling `build.cmake` directly—they are not plain pass-throughs. On both platforms,
**`--clean`**, **`--debug`**, and **`--devphysx`** are translated into the matching
**`build.cmake`** options.

- **Linux:** Run **`./build.sh`**. Layouts differ, but this is always the right entry point.
  The build runs directly on the host.
- **Windows:** **`build.bat`** clears Visual Studio environment variables that can
  override the packman MSVC toolchain, then runs `cmake -P scripts/build.cmake` with the
  mapped arguments above.

If your shell already has a suitable CMake and toolchain on `PATH`, you may call
`cmake -P scripts/build.cmake` directly.

**Agents / C++ compile-only:** `build.cmake` compiles all required targets (including
ovruntime) but does **not** install into `_install/` or run any tests. Use it
(via `build.sh` / `build.bat` / direct `cmake -P`) when you only need compile and link;
use `validate_all.cmake` (or `install.cmake` plus individual test scripts) for install and
test workflows.

Source builds use namespaced monolithic USD. There is no classic USD build
switch.

### `install.cmake`
Assembles the SDK into `_install/`: libraries, headers, CMake config, plugins,
licenses, docs, samples. Strips debug symbols on Linux. Verifies packaging lock and
glibc baseline.

Required before any test suite (C++ or Python runtime tests need `_install/`).

### `build_wheel.cmake`
Stages wheel contents in `_build/python_wheel_staging/` and builds the Python wheel
into `_dist/`. Requires `_install/` (run `install.cmake` first).

## Packaging scripts

### `package_sdk.cmake`
Creates a distributable SDK archive from `_install/` into `_dist/`. Distributable naming format. Smart rebuild based on timestamps.

## Test scripts

### `validate_all.cmake`
Full validation pipeline: runs `build.cmake` first (configure + incremental build), then
the `validate_all` CMake target (install, wheel, all tests). That target alone performs
the same install/wheel/test steps; use this script when you want build and validation in
one command without a separate `build.cmake` pass.

CI runs equivalent build and test steps, plus formatting (`ci_validate.cmake`),
docs build, and packaging.

**Rename:** `scripts/test.cmake` was removed; use `validate_all.cmake` instead (same role).

### `test_cpp.cmake`
C++ unit tests (GTest). Runs GPU and CPU passes in separate processes.

### `test_python_runtime.cmake`
Python runtime tests (pytest). Editable source against `_install/` + `_build/`.

### `test_cpp_samples.cmake`
Builds and runs C/C++ sample applications against the installed SDK.

### `test_python_wheel.cmake`
Wheel smoke tests: installs the wheel in fresh venvs across Python 3.10-3.13 and
runs `python -m ovphysx`.

### `test_python_samples.cmake`
Runs Python sample applications against the installed wheel across Python versions.

## Prerequisites

- CMake 3.16+ on Linux, CMake 4.1+ on Windows
- [uv](https://docs.astral.sh/uv/) for Python test/wheel management
- Dependencies auto-download during build via packman
