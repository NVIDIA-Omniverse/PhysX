<!-- SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# ovphysx Scripts

Cross-platform CMake scripts for building, testing, packaging, and installing ovphysx.

## Documentation validation without a native build

`test_documentation_package.py` stages the SDK and wheel documentation in temporary
directories using the same helpers as `install.cmake` and `build_wheel.cmake`.
It checks the portable file inventory, relative links, and rendered HTML with the
existing artifact validator, including decoding portable and rendered images.
It needs Python, Pillow and CMake, but no compiler, installed
OVPhysX package, or GPU. From `ovphysx/`, after rendering the docs:

```bash
python -m pip install -r scripts/documentation_requirements.txt
python scripts/test_documentation_package.py --html-docs _build/docs/ovphysx/latest
```

Use `--portable-only` for a local check when rendered HTML is unavailable. That
checks portable staging and links; CI requires the rendered output as well.


## Full local validation

From `ovphysx/`, `./build.sh` is the build.


The open-source drop ships `build_native.sh` as `build.sh`, so that public
script compiles on the host.

`./build.sh` produces `_build/` and stops. It does not install, build the
wheel, or run tests.


On a machine that can compile, install, the wheel, and the full test run are:

```bash
cmake --build _build --target validate_all
```

That target does not call `build.cmake` or reconfigure. It does compile. Its
dependencies include `ovphysx`, and `ovphysx` depends on the compile-only
public-header tests (`public_header_no_deprecated_test`,
`public_header_c_no_deprecated_test`, and on Linux
`c_api_compatibility_test`). Anything in that graph that is missing or stale
is compiled with the compiler stored in `_build`. CMake may regenerate
changed build inputs. The source-link sample configures in a clean directory
and the initial cache passes that stored compiler. The C/C++ samples
configure in a fresh `_build/sample_tests` directory and do not.
`validate_runtime` and `validate_wheel` share the dependency build, which
can still compile for all three targets. `validate_runtime` runs the runtime
tests, including source-link and the C/C++ samples. `validate_wheel` runs
the wheel tests only.

After an install, `tests/python_tests/run_pytest.sh` (Windows:
`run_pytest.bat`) is the fast Python loop. It does not compile.

On a machine that can compile, one command configures, compiles, and then runs
those same install, wheel, and test steps:

```bash
# from ovphysx/
cmake -P scripts/validate_all.cmake
```

Its first step is always `scripts/build.cmake`. There is no flag to skip that
compile. The script then installs into `_install/`, builds the wheel into
`_dist/`, and runs the C++ and Python test suites (unit tests, samples, wheel
smoke). Stage-by-stage detail and CI parity are under **`validate_all.cmake`**
below.

If that script fails, keep debugging it as one run. Separate install, wheel,
and test scripts hide which stage failed. `cmake -P scripts/validate_all.cmake`
is only for a host that can compile.


A host with newer glibc than the ovphysx baseline should skip the
ABI check (`install`/`wheel`/`validate` otherwise fail fast on `readelf`):

```bash
cmake -DSKIP_GLIBC_CHECK=ON -P scripts/validate_all.cmake
# env-var form also works: SKIP_GLIBC_CHECK=ON cmake -P scripts/validate_all.cmake
cmake -DSKIP_GLIBC_CHECK=ON -P scripts/install.cmake
```

Do **not** ship binaries built with the check skipped — CI runs it
unconditionally on the baseline image.

### Clean build

A clean tree starts with `./build.sh`, which produces `_build/`. On a machine
that can compile, `cmake --build _build --target validate_all` then installs,
builds the wheel, and runs the tests, compiling test-only targets as described
above. `cmake -P scripts/validate_all.cmake` does the host compile and the
tests together.

- After `git clean -dxf`, verify `_build/`, `_install/`, `_dist/`, and `_repo/`
  were actually removed. On Windows, `git clean` can skip nested dependency
  repos under `_build/`. For a real clean build, remove the resolved `_build/`
  path explicitly before validating.
- Make `uv --version` pass in the same shell before starting validation.
  Installing `uv` is not enough if its Scripts directory is not on `PATH`.
- `validate_all.cmake` runs CTest with `-LE benchmarks`; benchmarks are opt-in
  and require a benchmark-enabled rebuild.

**Windows:**

```powershell
# Confirm uv is on PATH before launching validation. If missing, install it and
# prepend the user Scripts directory for this shell.
py -3 -m pip install --user uv
$uvScripts = Join-Path (& py -3 -c "import site; print(site.USER_BASE)") "Scripts"
$env:Path = "$uvScripts;C:\Windows\System32;C:\Windows\System32\WindowsPowerShell\v1.0;$env:Path"
uv --version

# Prefer Ninja for full local validation; Visual Studio/MSBuild custom targets can
# lose PATH entries needed by nested cmake scripts (notably powershell.exe and uv.exe).
$env:GENERATOR = "ninja"
cmake -DSKIP_GLIBC_CHECK=ON -P scripts\validate_all.cmake
```

A validation report should include the branch and commit, the exact command and
key environment (`GENERATOR`, `PATH` changes, `uv --version`), whether
build, install, and wheel succeeded, the wheel path, the final CTest pass/fail
summary, and every failed CTest bucket with its inner failure line.

**Fast Python iteration** (after one build + install):

```bash
cd tests/python_tests && ./run_pytest.sh              # Windows: run_pytest.bat
```

Use the wrapper, not bare `uv run pytest` -- it passes `--locked` so `uv.lock`
is not rewritten during local test runs.

**Native CMake targets** (after `./build.sh`; `_build/` must already exist).
These do not call `build.cmake`. They do compile missing or stale dependencies,
and their CTest runs compile the source-link sample. The invoking machine
needs the compiler recorded in `_build`.

```bash
cmake --build _build --target validate_all        # install + wheel + all tests, plus those compiles
cmake --build _build --target validate_runtime    # install + C++ and Python runtime tests, plus those compiles
cmake --build _build --target validate_wheel      # install + wheel + wheel tests, plus those compiles
```

**CMake presets** (after `_build` exists and dependencies are fetched):

```bash
cmake --preset host-release                       # configure _build
cmake --build --preset validate-all               # validate_all in _build
```

## Build scripts

### `build.cmake`
Builds the SDK: fetches deps, configures CMake in `_build/`, and compiles libovphysx plus
the required ovruntime subproject targets.

Most developers invoke **`build.sh`** (Linux) or **`build.bat`** (Windows) instead of
calling `build.cmake` directly—they are not plain pass-throughs. On both platforms,
**`--clean`**, **`--debug`**, and **`--devphysx`** are translated into the matching
**`build.cmake`** options.

- **Linux:** Run **`./build.sh`**. It is the entry point in every layout.
  The open-source drop ships `build_native.sh` as `build.sh`, and that public
  script compiles on the host.
- **Windows:** **`build.bat`** clears Visual Studio environment variables that can
  override the packman MSVC toolchain, then runs `cmake -P scripts/build.cmake` with the
  mapped arguments above. `OVPHYSX_VS_ROOT=<installation path>` pins a specific Visual
  Studio; the pin is taken ahead of the packaged MSVC.

`build.cmake` compiles the SDK, including ovruntime. It does not install into
`_install/` or run tests. `./build.sh` is the usual way to run it. A shell that
already has CMake and a C++ toolchain on `PATH` can call
`cmake -P scripts/build.cmake` directly; that configures and compiles on the
host. `cmake -P scripts/validate_all.cmake` compiles and then tests, so it is
the command for a full host run rather than a compile by itself.

Source builds still fetch a namespaced monolithic OpenUSD package as a build-time
dependency (the ovruntime subproject's own USD dependency). The Python tests get
their python USD from stock pip `usd-core` on platforms with a PyPI
wheel (linux x86_64 and Windows; linux aarch64 has none, so the `pxr`-dependent
checks skip there). No internal USD monolith is fetched. The ovphysx libraries
link no USD and the SDK and wheel ship none. There is no classic USD build switch.

### `install.cmake`
Assembles the SDK into `_install/`: libraries, headers, CMake config, plugins,
licenses, docs, and samples. Strips debug symbols on Linux and verifies the
packaging lock and glibc baseline.

Required before any test suite (C++ or Python runtime tests need `_install/`).

### `build_wheel.cmake`
Stages wheel contents in `_build/python_wheel_staging/` and builds the Python wheel
into `_dist/`. Requires `_install/` (run `install.cmake` first).

## Packaging scripts

### `package_sdk.cmake`
Creates a distributable SDK archive from `_install/` into `_dist/`. Distributable naming format. Smart rebuild based on timestamps.

## Test scripts

### `validate_all.cmake`
Full validation on a machine that can compile. It always runs `build.cmake`
first (configure and compile on the host), then the `validate_all` CMake target
(install, wheel, all tests). It has no flag to skip that build.

Pass `-DDEV_SCHEMA=ON` to build and validate against the separate local schema
provider wheel; see [Developing Schemas Locally](../docs/physics_schemas.md#developing-schemas-locally).

After `./build.sh` has produced `_build/`, `cmake --build _build --target validate_all`
installs, builds the wheel, and runs the tests on a machine that can compile.
It does not call `build.cmake`. It still compiles missing or stale dependencies
and the source-link sample. `validate_runtime` and `validate_wheel` are the
narrower targets and share that compile behavior. Fast Python iteration after
an install is `tests/python_tests/run_pytest.sh`, which does not compile.

If `validate_all.cmake` fails, keep debugging that script as one run. Separate
install, wheel, and test scripts hide which stage failed. That script is the
full path only when the host can compile.


**Rename:** `scripts/test.cmake` was removed; use `validate_all.cmake` instead (same role).

### `test_cpp.cmake`
C++ unit tests (GTest). Runs GPU and CPU passes in separate processes. Its GPU
passes require a usable CUDA device. The isolated nonzero-ordinal case needs two
devices and deliberately skips when fewer are visible; the CPU-no-context case
also retains its documented OVStage/CUDA-context limitation. Each isolated
solver-thread case deliberately skips when the host's default tasking capacity
is below its requested worker count (1, 2, 4 or 8).

Full-driver result policy: each invoked pass or pytest suite must complete at
least one case successfully, apart from the documented fully-skipped exceptions.
A skip may follow useful assertions, so zero passed cases means the full
scenario remains unverified, not that no assertions ran. Healthy passes and
partial skips retain their existing outcome.

This policy does not change CPU simulation support or capability-aware skips
when invoking pytest directly. Use qualified CPU tests for targeted local
validation; the full validation pipeline requires the GPU prerequisites above.


### `test_python_runtime.cmake`
Python runtime tests (pytest). Editable source against `_install/` + `_build/`.
The full-driver result policy above applies to each pytest invocation's combined
JUnit report. Main, CPU-mode, utils and companion invocations can include
multiple files: a wholly skipped file is accepted when another case in the same
invocation passes. Lifecycle tests run one invocation per file, so a wholly
skipped lifecycle file is rejected. The Python driver has no fully-skipped
exceptions. This policy does not alter the test's own CUDA/CPU capability skips.

### `test_cpp_samples.cmake`

Add `-DBASIC_WORKFLOW_SMOKE=ON` to also build and run the installed
`samples/c_samples/basic_workflow_c` example referenced by the `ovphysx-basic-workflow`
skill. This check builds the installed source and CMake files directly with the
generator recorded in the SDK build cache; it does not read skill Markdown.
It runs with system and CUDA directories alone on Windows `PATH`. The Windows
Ninja and Visual Studio CI lanes enable it in their build-only and run-only passes.

Builds and runs C/C++ sample applications against the installed SDK. The build
uses the compiler recorded in `_build`. `SAMPLES_BUILD_ONLY` compiles and does
not run them. `SAMPLES_RUN_ONLY` runs executables already under
`_build/sample_tests` and does not compile.

### `test_source_link.cmake`
Configures and compiles `tests/c_samples/hello_world_source_link` in a build
directory it deletes first, then runs the executable against `_install/`.
`./build.sh` does not do this compile. There is no run-only mode.

### `test_python_wheel.cmake`
Wheel smoke tests: installs the wheel in fresh venvs across Python 3.10-3.13 and
runs `python -m ovphysx`.

### `test_python_samples.cmake`
Runs Python sample applications against the installed wheel across Python versions.

The Python 3.12 pass also runs the installed `omnipvd_inspection.py` workflow in
separate CPU/GPU subprocesses. It requires a separately installed SDK inspector;
missing tools fail the gate. `OVPHYSX_TEST_REQUIRE_CUDA=1` makes GPU absence a failure.
`OVPHYSX_SKIP_GPU_PASSES=ON` selects `--cpu-only`, even with CUDA present or
`OVPHYSX_TEST_REQUIRE_CUDA=1`. An explicit CMake value takes precedence over the
environment, so `-DOVPHYSX_SKIP_GPU_PASSES=OFF` overrides an inherited `ON`.
Otherwise the workflow uses `--require-gpu` only for `OVPHYSX_TEST_REQUIRE_CUDA=1`;
with neither mode selected, GPU absence can skip. The flags are never combined.
Captures, live/recorded evidence, logs and CPU/GPU comparison annotations remain in
`_build/test_results/ovd-workflow/`.

For focused CPU iteration, invoke `tests/ovd_workflow/test_workflow.py` with its
explicit `--inspector`, `--sample`, and `--output-root` paths plus `--cpu-only`.
This skips GPU and CPU/GPU comparison tests and is not full workflow acceptance.
`--cpu-only` and `--require-gpu` are mutually exclusive; the latter requires the
GPU workflow to run successfully rather than skip for missing CUDA.

`cmake -P scripts/build_ovd_inspector.cmake` builds and installs that headless tool
from this checkout's `physx/pvddom` into `<BUILD_DIR>/ovd-inspector-install`. The aggregate
validation targets and C++ sample build phase provision it automatically. Split
build/test jobs transfer that install with the selected build directory; run jobs consume it without a
compiler. A standalone OvPhysX source tree can instead set `OVPHYSX_OVD_INSPECTOR`
to an explicitly installed SDK executable before configure/validation. Neither the
wheel nor the sample depends on the SDK source tree. The companion SDK installation
also supplies `share/physx/SKILLS.md` and its inspection/comparison playbooks.

## `build.sh` options

```bash
./build.sh --clean                                # remove local build outputs (no build)
./build.sh --rebuild                              # clean then rebuild
./build.sh --debug                                # debug build
./build.sh --release                              # release build (default)
./build.sh --target <name>                        # build specific CMake target
./build.sh --generate                             # configure only (no build)
./build.sh --devphysx                             # build against local PhysX SDK source
./build.sh --devschema                            # use locally-built physics schema
cd tests/python_tests && ./run_pytest.sh cpu_tests/   # CPU-mode tests (separate invocation)
```

**`--devphysx` cannot be changed incrementally.** It selects a native build
flavor, and incremental builds across that change are unsupported. `--devphysx` builds PhysX from the local `physx/`
source tree and caches `PHYSX_SDK_DIR` pointing at it; a later configure
without `--devphysx` keeps that cached path, so the `PhysXGpu_64.dll` copy
step looks under the source tree instead of the packaged SDK and fails. Add
`--rebuild` whenever the `--devphysx` setting differs from the previous build.
`--devschema` only builds/selects the external schema wheel and can be switched
without a clean rebuild. OVPhysX consumes no PhysX schema headers or Packman schema package.

For a custom build directory, pass `-DBUILD_DIR=<path>` to CMake scripts or set
`OVPHYSX_BUILD_DIR` for scripts and `run_pytest.sh`/`.bat`. Relative paths are
resolved from `ovphysx/`; an explicit `BUILD_DIR` overrides the environment.
The directory must be `_build` or a subdirectory such as `_build/debug`, with
no symbolic links or junctions in that path. Absolute paths are accepted only
inside the same `_build` directory. Local build and test caches belong here.
Generated targets and CTest retain their build directory automatically.
`--clean` removes `_build`, including all custom build variants and their local
caches. It also retains the existing cleanup of `_install`, `_repo`, `_dist`,
`.venv`, and `__pycache__` directories throughout `ovphysx/`, directories named
`_build` in nested projects, immediate `ovruntime/_*` directories, and generated
`packaging_lock/*_new.json` snapshots. Committed packaging locks are preserved.
`--rebuild` does the same before building again; run
`cmake -P scripts/install.cmake` afterward to recreate `_install`.
Cleanup never traverses symbolic links or junctions. A matching cache link is
removed without deleting its target; other linked trees are left alone. Links
at the four top-level output directories, `ovruntime`, or `packaging_lock` are
rejected before any deletion. Failed cleanup stops the build.


The build (`build.sh`/`build.bat`) has two phases:

1. **ovruntime** (`ovphysx/ovruntime/`): built first via its own `repo build`
   system (premake). An incremental hash stamp skips this step when ovruntime
   sources are unchanged.
2. **ovphysx** (`ovphysx/`): built second via CMake. Ovruntime binaries land in
   `_build/release/`; `--devphysx` builds PhysX in its `checked` config but
   redirects the output back to the same `release/` directory.

Changes to files under `ovphysx/ovruntime/` therefore require a rebuild from
`ovphysx/` to take effect.

**VS Code / Cursor test explorer:** from `ovphysx/`, run
`tests/python_tests/run_pytest.sh --collect-only` (Windows:
`tests/python_tests/run_pytest.bat --collect-only`) to prepare the test environment.
Select `_build/venv/python_tests/bin/python` on Linux or
`_build/venv/python_tests/Scripts/python.exe` on Windows. With a custom build
variant, use `venv/python_tests/` beneath that build directory. `conftest.py`
auto-detects `_install/` and handles native library preloading.

## Updating the pinned ovstage version

`OVSTAGE_VERSION` in `scripts/fetch_ovstage_release.py` pins the native build,
including its git hash. Keep the pin in code; do not repeat its value in docs.
The fetcher verifies each released native zip against its platform's SHA-256 in
`OVSTAGE_DIGESTS`. Python consumers install the wheel separately; each uv
project's `ovstage` index and lock record its URL and SHA-256.
`validate_python_test_uv_lock.py` checks those locks. Native packages are never
reconstructed from wheels (ADR-0033).

Retries are opt-in: `OVSTAGE_FETCH_RETRY_TIMEOUT=1800` allows a 30-minute
window. The fetcher's `--retry-timeout` overrides the environment; `0` means
one attempt. The same variable and option set the window of the schema installer
(`tests/c_samples/common/fetch_physx_schemas.py`).

uv's own requests (index pages, wheel downloads, build requirements) are opt-in the
same way, through uv's `UV_HTTP_RETRIES`, which the test drivers leave to the
environment. With uv 0.9.7 the default of 3 retries gives up after about 19 s, 5 after
105 s and 6 after 249 s. CI sets `OVSTAGE_FETCH_RETRY_TIMEOUT=300` and
`UV_HTTP_RETRIES=6` so a repository outage of a few minutes does not fail a job.


## Prerequisites

- CMake 3.22+ on Linux, CMake 4.1+ on Windows
- C++17 compiler
- [uv](https://docs.astral.sh/uv/) for Python test/wheel management
- Dependencies auto-download during build via packman
