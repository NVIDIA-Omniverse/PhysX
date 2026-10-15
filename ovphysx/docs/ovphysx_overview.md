<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# ovphysx Overview

ovphysx is a self-contained PhysX runtime exposed as a C API with Python bindings.
It consumes application-owned ovstage data, runs simulation, and returns same-device
Warp arrays from the Python output-read API on CPU and CUDA. Native C transport and
the deprecated tensor-binding surface retain DLTensor interoperability.
It currently supports Windows (x86_64) and Linux (x86_64, aarch64) platforms.
On x86_64, pre-built binaries require a CPU with **AVX** (Advanced Vector Extensions);
refer to [System Requirements](#system-requirements).

This library is AI-agent friendly: `SKILLS.md` and the `skills/` directory ship with both the Python wheel and the C SDK package. `AGENTS.md` is also available in the source repository for quick codebase onboarding.

This page explains how ovphysx fits into the stack, what workflows it supports, and where to find build, tutorial, and API details.

## Who This Is For

- If you integrate ovphysx in Python apps or pipelines using the wheel.
- If you integrate the SDK in C or C++ native applications.

## What You Can Do

- Consume caller-owned ovstage data and drain committed ordinal ranges into the simulation.
- Replicate subtrees for large-scale parallel environments with `clone()` / `ovphysx_clone()` (PhysX-side replication in the internal representation only; USD untouched).
- Write simulation inputs (positions, velocities, controls) through the session write API (`PhysX.write` / `ovphysx_write`) or, when paired with `ovstage`, through attribute writes drained with `update_from_ovstage()`.
- Step the simulation.
- Read results (updated simulation state) back into your tensors. ovphysx does not write results back to the attached stage; the application owns writing state back to `ovstage`.

To get started, refer to the [Quickstart](tutorials/quickstart.md).

## Authoring Physics and Physics Concepts

ovphysx simulates pre-authored USD physics content. For how to author the physics
scenes it loads — and the physics concepts behind them — refer to the Simulation Setup
pages: [Physics Scene](simulation_setup/physics_scene.md),
[Colliders](simulation_setup/collision.md),
[Rigid Bodies](simulation_setup/rigid_bodies.md),
[Joints](simulation_setup/joints.md),
[Articulations](simulation_setup/articulations.md),
[Deformables](simulation_setup/deformables.md), and
[Particles](simulation_setup/particles.md). Physics schema layers and how the
application registers the separately installed codeless PhysX schemas are covered in
[Physics Schemas](physics_schemas.md). For tuning, refer to the Guides:
[Performance](guides/performance.md), [Collision Behavior](guides/collision_tuning.md),
and [Articulation Stability](guides/articulation_stability.md).

## Relationship to PhysX SDK and ovstage

ovphysx packages USD-aware scene ingestion, a streaming execution model, and tensorized data access on top of the PhysX SDK.
Applications own scene population through ovstage. ovphysx consumes that stage through `attach_ovstage()` / `ovphysx_attach_ovstage()` and drains committed ordinal ranges with `update_from_ovstage()` / `ovphysx_update_from_ovstage()`.

## Core Concepts

- Stream-ordered execution: calls run in submission order and see prior writes without extra sync.
- Session read/write: `PhysX.read` / `PhysX.write` (C `ovphysx_read` / `ovphysx_write`) move physics data (rigid-body poses, velocities, joint state, and the other columns listed in [Readable data](read_write/readable.md)) for the objects an object-type query (`PhysX.query` / `ovphysx_query`) selects. The older path-pattern tensor-binding API is deprecated (refer to [Migrating off Tensor Bindings](tensor_binding_migration.md)).
- Thread safety: each instance holds at most one ovstage attach, and several
  instances may be attached at once. Different instances may step their own
  attachments concurrently. A single instance is not safe for concurrent calls.
  The same ovstage instance cannot be attached twice.
- Warp output interoperability: session read groups expose `warp.array` tensors; use
  Warp's DLPack support to share memory with NumPy, PyTorch, and other consumers.

## End-to-End Usage in Python

Typical usage of ovphysx:

- Create and destroy instances with `PhysX()` and `physx.destroy()`.
- After population, seal the ordinal with `stage.advance_write_floor(ordinal).wait()`, then attach with `physx.attach_ovstage(stage, read_ordinal=ordinal)`. Apply later committed edits with `physx.update_from_ovstage(from_ordinal, to_ordinal)`.
- Attach, update and detach return `Task` receipts. Use `physx.wait_task(task)`
  to check completion before relying on success; keep the Stage alive until
  successful detach or instance destruction. They currently execute inline.
- Write and read simulation state with the session API — select objects with `physx.query()`,
  then open `physx.write()` / `physx.read()` sessions for one or more attributes; each group
  exposes the mapped buffers as `warp.array` on CPU or CUDA. These methods return an `Operation`
  without waiting: call its `wait()` or `physx.wait_all()`. `physx.read_sync()` /
  `physx.write_sync()` create their own query and wait internally. CUDA events preserve
  device-stream ordering. (The deprecated `physx.create_tensor_binding()` path still
  works; refer to
  [Migrating off Tensor Bindings](tensor_binding_migration.md).)
- Step the simulation with `physx.step()` — it is **asynchronous** (returns a `Task` receipt);
  use `physx.step_sync()` to step and wait in one call, or `wait_task()` / `wait_all()` to collect
  its outcome. Later native calls observe earlier work in submission order. Session outputs
  still require explicit `Operation` collection, and a later operation's wait does not collect
  an earlier step's failure.

Refer to the [Python API Reference](python_api.rst) for the full Python API surface.

## C/C++ SDK

The C API mirrors the Python flow:
- Initialize/shutdown the process lifecycle with `ovphysx_initialize()` and `ovphysx_shutdown()`.
- Create and destroy instances with `ovphysx_create_instance()` and `ovphysx_destroy_instance()`.
- Attach ovstage with `ovphysx_attach_ovstage()` and apply committed edits with
  `ovphysx_update_from_ovstage(handle, range)`, where `range` is an
  `ovstage_ordinal_range_t`. Attach, update and detach return
  `ovphysx_enqueue_result_t`; check admission and collect each task outcome.
- Write and read simulation state with the session API — `ovphysx_write()` / `ovphysx_read()` —
  selecting objects through `ovphysx_query()` and one or more attributes, then filling or reading the
  mapped groups. Query, open, fetch, commit and release return task receipts;
  wait before using outputs and collect release outcomes too. They currently execute
  inline. Successful fetch completion with a null group means EOF. (The deprecated `ovphysx_create_tensor_binding()` path still works.)
- Step the simulation with `ovphysx_step()` — it is **asynchronous** (returns an `ovphysx_enqueue_result_t` task receipt); use `ovphysx_step_sync()` to step and wait in one call, or `ovphysx_wait_task()` / `ovphysx_wait_all()` when consuming results outside the stream.


Refer to the [C API Reference](api.md) for the full C API surface.

## Release and Distribution

- Python: install the wheel as the primary distribution (`pip install ovphysx`).
- C/C++: consume the SDK package with headers and shared libraries (refer to the [GitHub release page](https://github.com/NVIDIA-Omniverse/PhysX/releases)).

### Wheel Contents and Environment Notes

The Python wheel (`pip install ovphysx`) contains the physics runtime and declares
an exact dependency on the matched `ovstage` wheel. Together those two wheels
provide the simulation runtime. Applications install the PhysX schema provider separately for USD population.

#### What the Wheel Bundles

- **ovphysx shared libraries** (`libovphysx.so` / `ovphysx.dll`)
- **Carbonite runtime** (embedded static framework plus bootstrap plugins)
- **PhysX runtime** (statically linked simulation and tensor implementation)

The exact matched `ovstage` wheel supplies ovstage together with the USD
resolver and the internal namespaced OpenUSD runtime ovstage uses to ingest USD
scenes. The ovphysx wheel ships no OpenUSD library, USD plugin registry, or
resolver of its own. Python dependencies are `packaging`, that exact `ovstage`
wheel, and the bounded `warp-lang` feature line. CUDA read-event waits use the
bundled native runtime on the current Warp stream without adding a Python CUDA
package. CPU groups and zero events need no CUDA wait.

### USD Coexistence and Schema Registration

ovphysx ships no OpenUSD runtime and never loads, links, preloads, or version-checks one. ovstage ingests USD scenes through its own internal namespaced OpenUSD runtime; the application owns whatever USD it authors with, for example stock `usd-core` from PyPI. ovphysx does not inspect or validate the USD libraries present in the process.

Applications install `physx-usd-schemas` separately, locate its resource
directories through package metadata (Python) or an explicit external root
(native), and register both directories before the first USD schema read
(population or export) in each process, including workers. Neither the ovphysx
SDK nor wheel delivers schema resources or exposes their location. By default,
attach refuses if ovstage
reports that its USD schema registry was built without either required PhysX
schema family. Registration after that point cannot repair the registry or recover
dropped settings: restart and register before populating or exporting. The legacy
Carbonite setting `/ovphysx/schemas/requireRegistration = false` permits attach with a
warning; it does not restore missing data. See
[Physics Schemas](physics_schemas.md#attach-time-registration-guard) for the
guard's scope, and dependency configuration, relocation, and registration recipes
that do not import USD during discovery.

## Versioning and Compatibility

ovphysx follows semantic versioning for the SDK surface (C API and Python bindings).

Backward compatibility guarantees that come with semantic versioning apply only to releases at or after v1.0; pre-1.0 releases may include breaking changes between minor and patch versions.

Functions exposed in experimental or internal folders (or prefixes) have no guarantees and may change or be removed without notice.

### Version Management

The project version is defined in `VERSION` (for example, `0.1.0` or `0.1.4-specialFeature`).

- `CMakeLists.txt` reads `VERSION` and generates `include/ovphysx/version.h`
- Python wheels convert to PEP 440 (`X.Y.Z-suffix` → `X.Y.Z.suffix`)
- C++ archives keep the original semver format

### API/ABI Policy

- **Major**: breaking C API or ABI changes, removed symbols, or incompatible data layout changes.
- **Minor**: backwards-compatible API additions, new symbols, or optional features.
- **Patch**: backwards-compatible bug fixes and internal changes only.

For v1.0 and later releases, backward compatibility is expected across patch and minor releases for the published C API and Python API. ABI compatibility is maintained within a major line; SONAME uses the major version on Linux to make ABI expectations explicit.

When a breaking change is required at the C API level, consider adding a versioned entrypoint (for example, `ovphysx_create_instance_v2`) to preserve compatibility, similar to CUDA-style `*_v2` APIs. Deprecated APIs should remain for at least one minor release with clear deprecation notes.

Python bindings validate that the package version and native library version match at runtime (base semver), and will raise a clear error if they diverge. This check can be bypassed for advanced cases with `ignore_version_mismatch=True` on `PhysX`.


## System Requirements

In addition to the platform list above:

- **Python 3.10+** for the wheel; a C++17 toolchain for the SDK. For the tested compiler and CUDA Toolkit matrix, refer to the PhysX SDK [Linux platform readme](https://github.com/NVIDIA-Omniverse/PhysX/blob/main/physx/documentation/platformreadme/linux/README_LINUX.md); building the SDK/wheel on a newer-than-baseline glibc (> 2.35) requires `SKIP_GLIBC_CHECK=ON`.
- **x86_64 (Linux and Windows): AVX required.** Shipped `libovphysx` / `ovphysx.dll`
  builds use AVX instructions unconditionally. There is no runtime CPU-feature
  dispatch and no non-AVX code path. Hosts without AVX (some older x86-64 CPUs,
  certain virtual machines or emulators with AVX disabled) are unsupported.
  `ovphysx_initialize()` (and Python `PhysX()` construction) returns an error
  with an explicit AVX requirement message before simulation starts.
- **Linux aarch64:** AVX does not apply (ARM has its own SIMD); use the aarch64 wheel.
- **GPU (optional):** NVIDIA GPU + CUDA-capable driver recommended for GPU dynamics;
  CPU-only simulation is supported on supported CPUs.

On Linux x86_64, this command reports whether the host CPU advertises AVX. Run it
before you install the wheel or SDK:

```bash
grep -qw avx /proc/cpuinfo && echo "AVX present" || echo "AVX missing"
```

On Windows, confirm AVX in your processor specifications before installing the
x86_64 wheel or SDK.

## Runtime Warnings

The native SDK bundles PhysX and Carbonite; the separately downloaded ovstage
package supplies ovstage, OmniClient, its connection library, and its internal
USD runtime. The Python distribution provides
the same runtime through the coordinated ovphysx and exact `ovstage` wheels. On startup and during
simulation, you may see warnings from these downstream dependencies such as:

- `[Warning] PhysXFoundation: Unable to create GPU Foundation` — appears on machines
  without an NVIDIA GPU; the SDK falls back to CPU simulation transparently.

These messages do not indicate a problem with your application. They originate from
the bundled runtime dependencies.

To suppress most startup noise, set the log level before creating an instance:
- **C:** `ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_LOG_LEVEL, ovphysx_config_value_int64(OVPHYSX_LOG_ERROR));`
- **Python:** `ovphysx.set_global_config(ovphysx.GlobalConfigKey.LOG_LEVEL, ovphysx.LogLevel.ERROR)`

## PhysX SDK Documentation

ovphysx uses the PhysX SDK and a statically linked Omni PhysX runtime under the hood.
For broader PhysX and Omni PhysX guidance, refer to the respective developer resources:
- [Omni PhysX User Guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/index.html)
- [PhysX Documentation](https://nvidia-omniverse.github.io/PhysX/)

## Conclusion

You now have a high-level view of ovphysx capabilities, compatibility policy, and distribution options. For build and runtime details, refer to the [Developer Guide](developer_guide.md). For hands-on usage, continue with [Hello World](tutorials/hello_world.md).
