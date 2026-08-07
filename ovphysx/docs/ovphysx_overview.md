# ovphysx Overview

ovphysx is a self-contained PhysX runtime exposed as a C API with Python bindings.
It consumes application-owned ovstage data, runs simulation, and reads or writes data through DLPack with same-device zero-copy access and transparent CPU/CUDA staging.
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
- Write simulation inputs (positions, velocities, controls) through tensor bindings or, when paired with `ovstage`, through attribute writes drained with `update_from_ovstage()`.
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
[Particles](simulation_setup/particles.md). Physics schema layers and how ovphysx
registers the codeless PhysX schemas are covered in
[Physics Schemas](physics_schemas.md). For tuning, refer to the Guides:
[Performance](guides/performance.md), [Collision Behavior](guides/collision_tuning.md),
and [Articulation Stability](guides/articulation_stability.md).

## Relationship to PhysX SDK and ovstage

ovphysx packages USD-aware scene ingestion, a streaming execution model, and tensorized data access on top of the PhysX SDK.
Applications own scene population through ovstage. ovphysx consumes that stage through `attach_ovstage()` / `ovphysx_attach_ovstage()` and drains committed ordinal ranges with `update_from_ovstage()` / `ovphysx_update_from_ovstage()`.

## Core Concepts

- Stream-ordered execution: calls run in submission order and see prior writes without extra sync.
- Tensor bindings: synchronous read/write operations for physics data (rigid body poses, velocities, etc.). Includes pattern matching (glob patterns) to bind to multiple prims at once (`/World/robot*`, `/World/env[N]/robot`).
- Thread safety: instances share the underlying physics runtime; serialize simulation,
  stage mutation, and binding creation across instances. A single instance is not
  safe for concurrent calls.
- DLPack interoperability: share memory with NumPy, PyTorch, and other DLPack consumers.

## End-to-End Usage in Python

Typical usage of ovphysx:

- Create and release instances with `PhysX()` and `physx.release()`.
- Attach caller-owned ovstage data with `physx.attach_ovstage(stage)` and apply committed edits with `physx.update_from_ovstage(from_ordinal, to_ordinal)`.
- Create tensor bindings with `physx.create_tensor_binding()` specifying tensor type and a prim path pattern or explicit prim paths.
- Write and read tensor data with `binding.write()` and `binding.read()` using NumPy arrays or dlpack-compatible buffers.
- Step the simulation with `physx.step()` — it is **asynchronous** (returns an `op_index`); use `physx.step_sync()` to step and wait in one call, or `wait_op()` / `wait_all()` when consuming results outside the ovphysx stream. In-stream tensor reads wait automatically.

Refer to the [Python API Reference](python_api.rst) for the full Python API surface.

## C/C++ SDK

The C API mirrors the Python flow:
- Initialize/shutdown the process lifecycle with `ovphysx_initialize()` and `ovphysx_shutdown()`.
- Create and destroy instances with `ovphysx_create_instance()` and `ovphysx_destroy_instance()`.
- Attach ovstage with `ovphysx_attach_ovstage()` and apply committed edits with
  `ovphysx_update_from_ovstage(handle, range)`, where `range` is an
  `ovstage_ordinal_range_t`.
- Create tensor bindings with `ovphysx_create_tensor_binding()` specifying tensor type and prim paths.
- Write and read tensor data with `ovphysx_write_tensor_binding()` and `ovphysx_read_tensor_binding()`.
- Step the simulation with `ovphysx_step()` — it is **asynchronous** (returns an `op_index`); use `ovphysx_step_sync()` to step and wait in one call, or `ovphysx_wait_op()` when consuming results outside the stream.


Refer to the [C API Reference](api.md) for the full C API surface.

## Release and Distribution

- Python: install the wheel as the primary distribution (`pip install ovphysx`).
- C/C++: consume the SDK package with headers and shared libraries (refer to the [GitHub release page](https://github.com/NVIDIA-Omniverse/PhysX/releases)).

### Wheel Contents and Environment Notes

The Python wheel (`pip install ovphysx`) contains the physics runtime and declares
an exact dependency on the matched `ovstage` wheel. Together those two wheels
provide the complete runtime; no additional NVIDIA or USD package is required.

#### What the Wheel Bundles

- **ovphysx shared libraries** (`libovphysx.so` / `ovphysx.dll`)
- **Carbonite runtime** (embedded static framework plus bootstrap plugins)
- **PhysX runtime** (statically linked simulation and tensor implementation)
- **OmniClient and `omniverse_connection`** (sourced from the exact matched
  OVStage package for PhysX-first startup)

The exact matched `ovstage` wheel supplies OVStage, the USD resolver and its
registry, and the namespaced USD runtime. The ovphysx wheel intentionally omits
duplicate resolver and USD singleton binaries. Python dependencies are
`packaging` and that exact `ovstage` wheel.

### USD Coexistence and Version Checking

ovphysx includes runtime safeguards for processes where an OV namespaced USD runtime may already be loaded by another package, such as another NVIDIA Omniverse library. Classic host USD, including `usd-core`, is intentionally separate and is not reused as ovphysx's runtime.

When ovphysx shares a process with another OV USD-aware subsystem such as ovrtx,
call `ovphysx_register_schema_paths()` and the peer subsystem's equivalent
before the first USD stage open so USD's schema registry sees all plugin roots.

When ovphysx starts, it checks whether a namespaced USD runtime is already loaded in the process. ovphysx then takes one of three paths:
- **No USD loaded**: ovphysx preloads the exact matched OVStage-provided USD
  runtime automatically (from the native SDK or the `ovstage` wheel).
- **Compatible OV namespaced USD loaded**: ovphysx uses the existing USD (skips preload).
- **Incompatible OV namespaced USD loaded**: ovphysx fails with a detailed error message showing the required vs. found version and remediation steps.

Detection uses `dlopen(RTLD_NOLOAD)` on Linux and `GetModuleHandle` on Windows to inspect process memory without side effects.
The required USD version is specified in `config.toml` using PEP 440 version specifiers (for example, `==25.11`).

You can control this behavior with `/ovphysx/skipUsdLibPreload` to bypass automatic USD preload.

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

- **Python 3.10+** for the wheel; a C++17 toolchain for the SDK. For the tested compiler and CUDA Toolkit matrix, see the PhysX SDK [Linux platform readme](https://github.com/NVIDIA-Omniverse/PhysX/blob/main/physx/documentation/platformreadme/linux/README_LINUX.md); building the SDK/wheel on a newer-than-baseline glibc (> 2.35) requires `SKIP_GLIBC_CHECK=ON`.
- **x86_64 (Linux and Windows): AVX required.** Shipped `libovphysx` / `ovphysx.dll`
  builds use AVX instructions unconditionally. There is no runtime CPU-feature
  dispatch and no non-AVX code path. Hosts without AVX (some older x86-64 CPUs,
  certain virtual machines or emulators with AVX disabled) are unsupported.
  `ovphysx_initialize()` (and Python `PhysX()` construction) returns an error
  with an explicit AVX requirement message before simulation starts.
- **Linux aarch64:** AVX does not apply (ARM has its own SIMD); use the aarch64 wheel.
- **GPU (optional):** NVIDIA GPU + CUDA-capable driver recommended for GPU dynamics;
  CPU-only simulation is supported on supported CPUs.

**Verify AVX on Linux x86_64:**

```bash
grep -qw avx /proc/cpuinfo && echo "AVX present" || echo "AVX missing"
```

On Windows, confirm AVX in your processor specifications before installing the
x86_64 wheel or SDK.

## Runtime Warnings

The native SDK bundles PhysX, Carbonite, and the matched OVStage-provided USD
runtime components. The Python distribution provides the same runtime through
the coordinated ovphysx and exact `ovstage` wheels. On startup and during
simulation, you may see warnings from these downstream dependencies such as:

- `[Warning] PhysXFoundation: Unable to create GPU Foundation` — appears on machines
  without an NVIDIA GPU; the SDK falls back to CPU simulation transparently.

These messages do not indicate a problem with your application. They originate from
the bundled runtime dependencies.

To suppress most startup noise, set the log level before creating an instance:
- **C:** `ovphysx_set_log_level(OVPHYSX_LOG_ERROR);`
- **Python:** `ovphysx.set_log_level(ovphysx.LogLevel.ERROR)`

## PhysX SDK Documentation

ovphysx uses the PhysX SDK and a statically linked Omni PhysX runtime under the hood.
For broader PhysX and Omni PhysX guidance, refer to the respective developer resources:
- [Omni PhysX User Guide](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/index.html)
- [PhysX Documentation](https://nvidia-omniverse.github.io/PhysX/)

## Conclusion

You now have a high-level view of ovphysx capabilities, compatibility policy, and distribution options. For build and runtime details, refer to the [Developer Guide](developer_guide.md). For hands-on usage, continue with [Hello World](tutorials/hello_world.md).
