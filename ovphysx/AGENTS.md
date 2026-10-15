<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# AGENTS.md -- ovphysx

Router for AI agents working in `ovphysx/`. Load the file for the job; do not
treat this page as the build manual, the PLC convention, or the changelog spec.

ovphysx is a self-contained C API and Python library for USD-based physics
simulation with Warp-array Python output and DLTensor-based native
interoperability. It lives at `ovphysx/` inside the Omniverse Physics monorepo.

Capabilities:

- Load USD scenes and simulate rigid bodies / articulations via PhysX
- Load from local paths, Omniverse Nucleus, S3 (HTTPS), or Azure Blob -- see
  [Remote USD Loading](docs/developer_guide.md#remote-usd-loading)
- Read simulation state as `warp.array` on CPU / CUDA and interoperate through
  Warp's DLPack support
- Clone environments for batched RL workloads (1000s of parallel instances)

Supported platforms: Linux x86_64, Linux aarch64 (arm64), Windows x64. x86_64
builds require AVX at runtime; `ovphysx_initialize()` fails fast when AVX is
missing (documented; no non-AVX fallback). GPU features require CUDA and an
NVIDIA GPU. All code should support all platforms, use as few
platform-specific sections as possible.

## Where to read next

| Job | File |
|---|---|
| Build, validate_all, glibc skip, Windows recipe, `build.sh` flags, OVStage and PhysX schema pins, wheel bumps, source switching | [`scripts/README.md`](scripts/README.md) |
| C++ / doc style | [`docs/coding_style.md`](docs/coding_style.md) |
| Product skill playbooks | [`SKILLS.md`](SKILLS.md) |


## Build and test

All commands run from `ovphysx/`.

Build with `./build.sh`. Do not use `cmake -P scripts/build.cmake` or
`cmake -P scripts/validate_all.cmake` for that build.


The open-source drop ships `build_native.sh` as `build.sh`, so that public
script compiles on the host.

`./build.sh` does not compile the source-link sample or the C++ samples. Those
compile when their tests run. The compile-only public-header tests are
dependencies of `ovphysx`. `cmake --build _build --target validate_all` still
compiles any of them that are missing or stale, using the compiler that
configured `_build`.

Do not run either of these on a machine that cannot compile:

```bash
cmake --build _build --target validate_all
cmake -P scripts/validate_all.cmake
```

`validate_all.cmake` always runs `scripts/build.cmake` on the invoking machine
first. It has no flag to skip that. The `validate_all` target does not call
`build.cmake`. It still runs the native build tool. CMake may regenerate
changed build inputs. Its CTest run compiles `source-link` from a clean
directory, and that configure still receives the compiler stored in `_build`.
`validate_runtime` runs that sample and the C/C++ samples.
`validate_wheel` runs the wheel tests only. The dependency build can still
compile for all three targets. `scripts/test_cpp_samples.cmake` configures
each sample in a fresh directory, so it does not use the compiler stored in
`_build`, and it compiles unless `SAMPLES_RUN_ONLY` is set.

After `./build.sh`, these do not compile. They fail if the install tree or a
required binary is missing. They are not the full suite:

```bash
cmake -P scripts/install.cmake
cmake -P scripts/build_wheel.cmake
cmake -P scripts/test_cpp.cmake
tests/python_tests/run_pytest.sh <test files>   # Windows: run_pytest.bat
```

Use `run_pytest.sh` for the Python tests a change touches. Use
`cmake -P scripts/test_cpp.cmake` for the C++ unit binaries `./build.sh`
already produced. There is no changed-files target.


`cmake -P scripts/validate_all.cmake` is only for a host that can compile.
`SKIP_GLIBC_CHECK=ON` skips the binary ABI check for a local host compile. Do
not ship binaries built with it. Do not replace a failed
`validate_all.cmake` run with separate install, wheel, and test invocations
unless you are debugging that failed stage. Details live in
[`scripts/README.md`](scripts/README.md).


## Public API

Pre-release. C (`include/ovphysx/ovphysx.h`, `ovphysx_types.h`) is the primary
ABI; deprecated families in `ovphysx_legacy.h`. C++ (`experimental/ovphysx.hpp`)
may change without notice. Python: `python/ovphysx/api.py` (`PhysX`).

New code uses `PhysX.read` / `PhysX.write`. TensorBindingsAPI is deprecated
(`ovphysx_legacy.h`, `PhysX.create_tensor_binding`). Some calls return a task
receipt; wait only for out-of-stream work -- that wait is also where an accepted
task's execution failure is reported.

## Navigation

- Samples: `tests/python_samples/`, `tests/c_samples/`
- Shipped samples: `samples/`
- Tutorials: `docs/tutorials/`
- Developer guide: `docs/developer_guide.md`
- Benchmarks (opt-in): `tests/benchmarks/`, `tests/python_benchmarks/` -- see
  [`tests/benchmarks/README.md`](tests/benchmarks/README.md)
- Rendered docs: https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/index.html


## MR hygiene

- A `docs/changelog.md` entry is not required in a feature MR. User-facing
  rationale belongs in the MR title/description (the release generator reads
  those). Hand-written entries are optional and must stay public/user-level.

## Common footguns

Details in `docs/developer_guide.md` unless noted.

- **A task outcome is collected once.** `wait_task()` collects only that task;
  `wait_all()` collects every uncollected task in submission order. Repeating a
  collected wait returns `NOT_FOUND`.
- **Settings and hard CPU-only mode are per-process.** `ovphysx_set_cpu_mode(true)`
  cannot be reverted in that process.
- **No OpenUSD in the shipped product.** `libovphysx` links no USD (ADR-0027).
  ovstage ingests USD; the app owns whatever USD it authors with. Register
  PhysX codeless schemas before the first population call. Never load two USD
  images in one process. See `docs/physics_schemas.md`.
- **Thread safety.** A single instance is not thread-safe. Different instances
  may step their own attachments concurrently. Do not wait on the same task
  from multiple threads.
- **One attach per instance.** Several instances may be attached at once.
  Detach and destroy affect only that instance. The same ovstage instance
  cannot be attached twice. Hard CPU-only mode, tolerance scale, and the
  recording transport stay process-wide.
- **Error string ownership.** C: `ovphysx_get_last_error()` on the same thread
  for rejected submissions; task failures via `ovphysx_get_last_task_error` until
  `ovphysx_destroy_wait_task_result()`. Never shallow-copy the result struct.
- **Population domain files are generated.** Edit the JSON contract and run
  `ovruntime/physics_population_domain/tools/gen_contract.py --write`. See
  `ovruntime/physics_population_domain/README.md`.
- **Error/lifecycle checks should be C-first.** Keep Python thin.

## Other

- Keep changes scoped to the requested project area.
- Prefer existing APIs and patterns in the nearest `README.md` / `AGENTS.md`.
- Do not add secrets or credentials to files or logs.
