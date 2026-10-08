<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Developer Guide

Use this guide when you integrate, build, and operate ovphysx in your own application. You learn how to build the SDK, run samples, and apply core runtime rules for synchronization, threading, and resource ownership.

## On this page

- [Relationship and Consumption Model](#relationship-and-consumption-model)
- [Samples and Tutorials](#samples-and-tutorials)
- [Execution Model](#execution-model)
- [Configuration](#configuration)
- [Process-Global Lifecycle](#process-global-lifecycle)
- [Multi-Instance Support](#multi-instance-support)
- [Operation Indices and Polling](#operation-indices-and-polling)
- [Threading](#threading)
- [Tensor Binding Diagnostics](#tensor-binding-diagnostics)
- [Ownership and Lifetimes](#ownership-and-lifetimes)
- [Dependency Management](#dependency-management)
- [Error Handling](#error-handling)
- [Warmup and Determinism](#warmup-and-determinism)
- [Scene Cloning](#scene-cloning)
- [Remote USD Loading](#remote-usd-loading)
- [Scene Queries](#scene-queries)
- [PhysX Pointer Interop](#physx-pointer-interop)
- [Contact Data](#contact-data)
- [Running With Other Carbonite Users](#running-with-other-carbonite-users)
- [Logging](#logging)
- [OmniPVD Recording](#omnipvd-recording)
- [NVTX Profiling With Nsight Systems](#nvtx-profiling-with-nsight-systems)

## Relationship and Consumption Model

ovphysx provides a stable C API and Python bindings for ovstage-driven PhysX simulation.
Use ovphysx when you need USD-based physics simulation outside of Omniverse Kit with a small native SDK.

- Python: `pip install ovphysx` and `from ovphysx import PhysX`
- C/C++: download the ovphysx SDK and matching native ovstage archive as
  described in the [SDK Quickstart](tutorials/quickstart.md), keep the package
  roots separate, and link through CMake

**CPU note (x86_64):** pre-built ovphysx binaries require **AVX**. Missing AVX is
detected at `ovphysx_initialize()` with a clear error. Refer to
[System Requirements](ovphysx_overview.md#system-requirements) in the overview for
compatibility details and how to verify AVX on Linux x86_64.

This guide focuses on the runtime and integration surface. For authoring the USD
physics content that ovphysx simulates and the concepts behind it (scenes,
colliders, rigid bodies, joints, articulations, deformables, particles), refer to the
Simulation Setup pages starting with
[Physics Scene](simulation_setup/physics_scene.md), and the tuning
[Guides](guides/performance.md).

## Samples and Tutorials

The ovphysx samples are runnable references for SDK and wheel usage, designed to run in a clean environment matching how end users consume the wheel or SDK.

> **Note:** In the source repository, samples live under `tests/c_samples/` and
> `tests/python_samples/`. The Python wheel installs the Python samples under
> `ovphysx/samples/python_samples/`; the C/C++ SDK installs the C/C++ samples
> under `samples/c_samples/`. Both artifacts install shared sample data under
> their respective `samples/data/` directory.

**Python Samples** (`tests/python_samples/`):

| Sample | Tutorial | Feature |
|---|---|---|
| `hello_world.py` | [Hello World](tutorials/hello_world.md) | Load USD + step (minimal workflow) |
| `tensor_bindings.py` | [Tensor Bindings](tutorials/tensor_bindings.md) | Read/write simulation data through tensor bindings (**deprecated**; use `PhysX.read` / `PhysX.write`) |
| `kinematic_support.py` | [Kinematic Support Geometry](simulation_setup/kinematic_support.md) | ovstage-driven kinematic supports, surface-velocity conveyors, and combined rider motion |
| `contact_binding.py` | [Contact Binding](tutorials/contact_binding.md) | Read contact forces through sensor/filter bindings |
| `tensor_bindings_views.py` | | Build lightweight view wrappers on TensorBindingsAPI (advanced; **deprecated**) |
| `omnipvd_recording.py` | [OmniPVD Recording](tutorials/omnipvd_recording.md) | Record physics internals to .ovd files |
| `output_read.py` | [ovstage Integration](ovstage_integration.md) | Closed loop: author control into ovstage, drain it explicitly, step, and read rigid-body position and velocity from `boxes_falling_on_groundplane.usda` |

**Extra Python Samples** (`tests/python_samples_extra/`):

| Sample | Tutorial | Feature |
|---|---|---|
| `visual_rerunio_sample/visualize_rigid_bodies.py` | [Rendering Handoff](tutorials/render_handoff.md) | Visualize rigid body simulation with Rerun |

**C/C++ Samples** (`tests/c_samples/`):

| Sample | Tutorial | Feature |
|---|---|---|
| `hello_world_c/` | [Hello World](tutorials/hello_world.md) | Minimal C hello world |
| `tensor_bindings_c/` | [Tensor Bindings](tutorials/tensor_bindings.md) | CPU tensor read/write (**deprecated**; use `ovphysx_read` / `ovphysx_write`) |
| `tensor_bindings_gpu_c/` | | GPU tensor read/write with CUDA (**deprecated**) |
| `kinematic_support_c/` | [Kinematic Support Geometry](simulation_setup/kinematic_support.md) | ovstage-driven kinematic supports, surface-velocity conveyors, and combined rider motion |
| `contact_binding_c/` | [Contact Binding](tutorials/contact_binding.md) | Contact force reading |
| `omnipvd_recording_cpp/` | [OmniPVD Recording](tutorials/omnipvd_recording.md) | Record physics internals to .ovd files |
| `physx_interop_cpp/` | [PhysX Interop](tutorials/physx_interop.md) | Direct PhysX SDK pointer access |
| `output_read_c/` | [ovstage Integration](ovstage_integration.md) | Closed loop on the same scene; queries several output types, including `OVPHYSX_OBJECT_ARTICULATION`, which is empty because the USD has no articulations |

For SDK setup, refer to [SDK Quickstart](tutorials/quickstart.md).

The tensor-binding API is **deprecated**; new code should use the session read/write API
(`ovphysx_read` / `ovphysx_write`). For the deprecated tensor binding shape/read/write semantics,
refer to [Tensor Bindings](tutorials/tensor_bindings.md).
Canonical enum-level definitions are in `include/ovphysx/ovphysx_types.h` (`ovphysx_tensor_type_t`).

## Execution Model

ovphysx uses a stream-ordered execution model:

- Calls are enqueued in submission order and observe prior writes without extra sync.
- Asynchronous calls return an `op_index`; wait on it before consuming results outside the stream.
- Synchronous calls complete before returning. `clone()` retains the operation-index-shaped API
  and returns an already-complete `op_index`; waiting on it is valid and returns immediately.

Concretely, `step()` / `ovphysx_step()` is **asynchronous** and returns an
`op_index`, while `step_sync()` / `ovphysx_step_sync()` steps and waits in one
call (and `step_n_sync()` batches N synchronous steps). After an async `step()`,
in-stream ovphysx operations (tensor `read()` / `write()`) see the result without
extra synchronization; only out-of-stream consumers (external GPU work, logging,
rendering, network I/O) need an explicit `wait_op()` / `wait_all()`.

Use `ovphysx_wait_op()` (or `PhysX.wait_op()` in Python) to:
- synchronize before reading or modifying tensors on CPU/GPU if they are currently accessed by asynchronous operations inside ovphysx
- ensure correctness before external side-effects (logging, rendering, network I/O)

## Configuration

The SDK uses a typed config system for known settings, plus a raw Carbonite-setting override for arbitrary paths. Config entries are built
with convenience functions from `ovphysx_config.h` and passed at instance creation or set at runtime.

### C

```c
#include <ovphysx/ovphysx.h>
#include "ovphysx/ovphysx_config.h"
#include <stdint.h>

int main(void)
{
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_config_entry_t entries[] = {
        ovphysx_config_entry_disable_contact_processing(true),
        ovphysx_config_entry_num_threads(4),
        ovphysx_config_entry_carbonite(
            OVPHYSX_LITERAL("/physics/updateToUsd"),
            OVPHYSX_LITERAL("false")),
    };
    args.config_entries = entries;
    args.config_entry_count = 3;

    if (ovphysx_initialize().status != OVPHYSX_API_SUCCESS)
        return 1;

    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    if (ovphysx_create_instance(&args, &handle).status != OVPHYSX_API_SUCCESS) {
        ovphysx_shutdown();
        return 1;
    }

    ovphysx_result_t result =
        ovphysx_set_global_config(ovphysx_config_entry_num_threads(8));
    int32_t threads = 0;
    if (result.status == OVPHYSX_API_SUCCESS)
        result = ovphysx_get_global_config_int32(
            OVPHYSX_CONFIG_NUM_THREADS, &threads);

    ovphysx_result_t destroy_result = ovphysx_destroy_instance(handle);
    ovphysx_result_t shutdown_result = ovphysx_shutdown();
    return result.status == OVPHYSX_API_SUCCESS &&
                   destroy_result.status == OVPHYSX_API_SUCCESS &&
                   shutdown_result.status == OVPHYSX_API_SUCCESS
               ? 0
               : 1;
}
```

### Python

```python
from ovphysx import PhysX, PhysXConfig, ConfigBool, ConfigInt32

def read_configuration() -> tuple[int, bool]:
    physx = PhysX(config=PhysXConfig(
        disable_contact_processing=True,
        num_threads=4,
        carbonite_overrides={"/physics/updateToUsd": False},
    ))
    try:
        threads = physx.get_config_int32(ConfigInt32.NUM_THREADS)
        enabled = physx.get_config_bool(ConfigBool.DISABLE_CONTACT_PROCESSING)
        return threads, enabled
    finally:
        physx.destroy()
```

The ``carbonite_overrides`` dict accepts arbitrary Carbonite setting paths for
settings not yet covered by the typed fields. Value types are auto-detected from the
string representation. Using a ``carbonite_overrides`` key that targets a path already
covered by a typed field raises ``ValueError``.

> **Note:** `carbonite_overrides` is write-only. There is no getter for arbitrary Carbonite paths; use the typed getters (`get_config_bool`, `get_config_int32`, and the other `get_config_*` methods) for known settings.

Config is process-global (Carbonite-backed). All instances in the same process share the
same config state.

### Cooked-Collider Cache (UJITSO)

Collision "cooking" converts a mesh into the representation PhysX simulates (convex hull,
triangle mesh, SDF, or convex decomposition). It is CPU-expensive, so ovphysx caches the
cooked result on disk and reuses it on the next run through UJITSO, a local content-addressed
cache. The cache key includes the mesh geometry, the cooking parameters, the cooking type, and
a cooker/PhysX version token, so a changed mesh, changed parameters, or a different PhysX build
automatically re-cooks instead of returning a stale shape.

The cache directory is **application-provided**: ovphysx does not derive a persistent cache
location from the environment and does not persist to a location of its own choosing. Set it to
enable cross-run persistence:

```python
from ovphysx import PhysX, PhysXConfig

def create_physx_with_cache(cache_directory: str) -> PhysX:
    return PhysX(config=PhysXConfig(cooked_collider_cache_dir=cache_directory))
```

In C, set the `OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY` string config key. The first run with
a given cache cooks and stores the result; later runs with an unchanged scene are served from the
cache without re-cooking.

With no directory configured, nothing is persisted: cooked colliders go to a **process-private**
directory under the OS temp directory, so every run re-cooks. ovphysx removes that directory at
normal process exit. Python destroys still-live `PhysX` instances from `atexit`, including after
an uncaught Ctrl-C (`KeyboardInterrupt`), then shuts down the process lifecycle even if a
`read()` Warp array is still globally reachable, so cleanup follows the same last-instance
shutdown path as explicit `destroy()`. Abrupt termination that cannot run exit handlers, such as `SIGKILL` or a
hard crash, may leave the directory for the host's temp-directory cleanup. The default `SIGTERM`
action also bypasses Python `atexit`; applications that need graceful `SIGTERM` cleanup should
translate it into their normal shutdown flow. Do not `os.fork()` or use
`multiprocessing` with the `fork` start method after constructing `PhysX`. The
runtime, CUDA, and Carbonite are not fork-safe; a child that inherited live
instances would run the parent's `atexit` teardown against native state it does
not own. Start workers with `spawn`, or use a process that never constructed
`PhysX`. On Windows removal is
best-effort because the datastore may still hold files open. The cache still needs a real path in
that case -- the underlying datastore has no memory-only mode and would otherwise report an error
for a path it cannot open -- but nothing is left behind for the next run to reuse after successful
cleanup. The temp location itself follows the platform's `TMPDIR`/`TMP`/`TEMP` convention; what
ovphysx will not do is derive a *persistent* cache location from the environment. The same
process-private fallback is used when a configured directory cannot be created or written.

The configured directory is applied when the ovphysx runtime first starts in a process. Once the
runtime is up, a later `PhysXConfig(cooked_collider_cache_dir=...)` in the same process is not
picked up -- the datastore is built once. Set it on the first instance, or use a separate process
per cache location.

Cooked colliders are **not** reused across runs when:

- No `cooked_collider_cache_dir` is configured: cooking still runs, but nothing is persisted, so it re-cooks every run.
- ovphysx cannot create or write the configured directory: it falls back to the process-private cache, so nothing is persisted (a warning is logged).
- The mesh, the cooking parameters/type, or the PhysX build changed: the key differs, so the shape is re-cooked (this is correct, not a fault).
- Collision cooking has been disabled (it is enabled by default).

ovphysx uses UJITSO as a local, in-process cache. (UJITSO can also operate as a distributed cache;
ovphysx does not use that.) The cache is per-machine, shared only through the directory you
configure. It is bounded: the datastore runs disk garbage collection against
`/UJITSO/datastore/localDataStore/largeChunkDiskBudgetMB` (default 102,400 MB) and
`smallChunkDiskBudgetMB` (default 1,024 MB), evicting toward the budget once it is exceeded.
Raise those settings for a larger cache, or delete the directory to reclaim space immediately. If
collision cooking is enabled but the required UJITSO cache plugins are not loaded, instance creation
fails fast with a clear error rather than silently cooking uncached.

## Process-Global Lifecycle

C callers can use `ovphysx_initialize()` to initialize process-global lifecycle
state, and `ovphysx_shutdown()` to shut it down. Only one initialize can be
active at a time; a second initialize before shutdown returns
`OVPHYSX_API_ERROR`. C callers must call `ovphysx_initialize()` before
`ovphysx_create_instance()`.

`ovphysx_shutdown()` does not destroy live handles and does not balance
`ovphysx_create_instance()`. Call `ovphysx_destroy_instance()` for every handle
returned by `ovphysx_create_instance()`. A successful shutdown always disables
and drains the application log callback, so its callback and user data may be
released immediately. A handle retained across shutdown is supported only for
explicit destruction; do not step it or start other instance work, and do not
expect its destruction-time records through the disabled callback.

## Multi-Instance Support

The SDK exposes a per-instance handle (`ovphysx_handle_t`) so callers can manage lifetime and tensor/contact bindings on a per-instance basis. **However, the underlying simulation state is process-global**: only one ovstage-backed USD scene can be attached at a time across all instances in a process. While one instance owns the live attach, another instance's attach attempt returns `OVPHYSX_API_ERROR` and leaves the owner's stage and bindings unchanged. Detach the owning instance before attaching a stage on another instance.

Concretely, this means:

- Multiple `ovphysx_handle_t` values can coexist, but they share one simulation backend and one attached stage.
- Config, the hard CPU-only policy, and the attached stage are process-global. Different instances cannot use different config values or stages in the same process; outside hard CPU-only mode, CPU/GPU dynamics are authored per scene.
- For isolated stages or parallel simulations, run each in a separate subprocess.

Multiple `UsdPhysicsScene` prims within the same loaded stage **are** supported. `ovphysx_step()` advances all scenes together; per-scene independent stepping is not currently exposed through the ovphysx C API.

In-process destroy/create cycles **are supported** within one active process
lifecycle: per-instance destruction leaves Carbonite and the direct PhysX
runtime available for the next instance. The hard CPU-only request is the
exception: as soon as `ovphysx_set_cpu_mode(true)` succeeds, it is sticky and
cannot be reverted in that process. Its CPU-only and ovphysx no-CUDA guarantees
require setting it before the first instance. Outside hard CPU-only mode, CPU/GPU
dynamics remain authored per scene in USD rather than selected by
`ovphysx_create_instance()`. Call `ovphysx_shutdown()` once when the current
process-lifecycle scope is done. With no live handles it clears the
`ovphysx_initialize()` token and drains the direct PhysX runtime; the Carbonite
framework remains resident for its process-exit hook.

## Operation Indices and Polling

`op_index` values are single-use. A wait processes unconsumed indices in order
through the requested index: every completed or failed index it reaches is
consumed and must not be used again. A timeout leaves the first still-pending
index and later indices unconsumed. An index completed by internal stream
synchronization may be acknowledged once with `wait_op`; that acknowledgement
consumes it.

C and C++ callers use `ovphysx_timeout_t`, a `uint64_t` nanosecond type. Pass
`OVPHYSX_TIMEOUT_POLL` for a non-blocking readiness check, a positive value for
a finite wait, or `OVPHYSX_TIMEOUT_INFINITE` to wait until a terminal result.
Zero does not wait for simulation readiness, and a positive value bounds only
that readiness wait. Once a result is ready, the call may still finalize and
publish it before returning, so total wall-clock duration can exceed a finite
timeout. Python uses the equivalent `timeout_ns=0`, a positive integer, or
`None`.

## Threading

- A single instance is **not** thread-safe. Serialize access externally.
- Do not wait on the same `op_index` from multiple threads.
- Because all instances share one simulation backend and one attached stage, calling simulation APIs concurrently from different instances is not safe either. Serialize across instances as well.

### Python: concurrent `step()` and tensor I/O

> **Warning: the GIL does not protect ovphysx calls.** The Python API reaches
> native code through ctypes. The GIL is released during those calls, so two
> Python threads can execute ovphysx C code at the same time. Calling
> `step()` / `step_sync()` on one thread while another thread calls
> `TensorBinding.read()` / `write()` (or any other simulation API on the
> same instance) is a data race. Results range from torn reads to segfaults.
> This pattern is common in multi-threaded RL frameworks that dispatch
> simulation steps and observation collection to separate thread pools.

Serialize all simulation and tensor I/O on a single instance (for example with
a `threading.Lock`). If different threads participate, complete the step first
and only read or write tensors after that step is finished:

```python
import threading

def run_workers(physx, binding, output_buffer, dt):
    lock = threading.Lock()

    def sim_thread():
        for _ in range(1000):
            with lock:
                physx.step_sync(dt)  # mutation completes before lock is released

    def read_thread():
        for _ in range(1000):
            with lock:
                binding.read(output_buffer)  # no concurrent step on another thread

    sim_worker = threading.Thread(target=sim_thread)
    read_worker = threading.Thread(target=read_thread)
    sim_worker.start()
    read_worker.start()
    sim_worker.join()
    read_worker.join()
```

When using asynchronous `step()`, the same rule applies: do not call
`binding.read()` / `binding.write()` concurrently with an in-flight step.
Either wait first (`wait_op()` / `wait_all()` / `step_sync()`), or keep all
step and tensor calls on one thread. In-stream tensor reads submitted after
`step()` on the same thread are stream-ordered and safe without an explicit
wait; cross-thread use always requires external serialization.

## Tensor Binding Diagnostics

- Pattern bindings can intentionally match zero physics objects, so expected TensorAPI
  no-match diagnostics are quieted on the simulation view used to create that
  binding.
- For rigid-body and articulation pattern bindings, TensorAPI resolves candidate
  paths before filtering them to the requested physics-object type. Same-named
  candidates of another type are skipped without per-candidate warnings.
- Explicit object paths supplied through `prim_paths` keep the default
  error-level no-match diagnostics for typo detection.
- A single pattern component (the text between two slashes; a parenthesized
  group counts as one component even if it contains a slash) may be at most
  4096 characters long. Every pattern-taking call rejects a longer component up
  front with `OVPHYSX_API_INVALID_ARGUMENT` (`RuntimeError` in Python).
- To detect partial misses programmatically, compare requested explicit paths
  with Python `binding.prim_paths` or C `ovphysx_tensor_binding_get_prim_paths()`
  after binding creation.

## Ownership and Lifetimes

- Tensor bindings, contact bindings, and attribute bindings own internal resources. They are automatically destroyed when the parent instance is destroyed through `ovphysx_destroy_instance()`. Explicit per-binding destruction (`ovphysx_destroy_tensor_binding`, `ovphysx_destroy_contact_binding`) is available for releasing resources earlier.
- In Python, close a binding deterministically with a `with` block or an explicit `binding.destroy()`. New code should exchange simulation state through the session read/write API, which manages its own lifetimes (`with physx.read(...) as result:` / `with physx.write(...) as session:`); the tensor-binding form `with physx.create_tensor_binding("/World/robot*", tensor_type=TensorType.RIGID_BODY_POSE) as binding:` is **deprecated** but still works during the deprecation window. Contact bindings — `with physx.create_contact_binding(["/World/sensor*"]) as binding:` — are a separate, current API. If garbage collection cleans up a `TensorBinding` or `ContactBinding` first, ovphysx emits `ResourceWarning`. Python suppresses this warning category by default; run with `-W default` or set `PYTHONWARNINGS=default` to show it.
- Create tensor and contact bindings once outside simulation loops and reuse them across steps. Creating a new binding every step allocates new native resources and is slower than reusing the existing binding.
- Tensor and contact bindings are views of the currently realized physics objects. `step()` and `step_sync()` do not invalidate them, but application-owned stage lifecycle changes do. Before `reset_stage()`, before removing USD data that contains bound objects, or before replacing/reparsing the stage so bound objects are destroyed and recreated, destroy cached bindings when practical. If a stale binding survives one of those lifecycle operations, only destroy it; do not read or write through it. Create replacement bindings after the operation has completed. Do not keep cached bindings across topology changes and probe them defensively; the application knows when it changed the stage.
- On failure, call `ovphysx_get_last_error()` on the same thread to retrieve the error string (valid until the next ovphysx call on that thread).
- PhysX pointers from `ovphysx_get_physx_ptr()` are owned by ovphysx — do not call `release()` on them. Refer to [PhysX Pointer Interop](#physx-pointer-interop) for lifetime details.
- **Contact report buffers** from `ovphysx_get_contact_report()` (and the
  default Python `get_contact_report()`) are zero-copy views into internal
  buffers — valid **only** until the next `step()` / `step_sync()` call.
  Accessing them after the next step is undefined behavior (silent
  corruption or segfault). Python: pass `copy=True` to
  `physx.get_contact_report(copy=True)` to receive Python-owned lists of
  dicts that are safe to retain across simulation steps. Recommended for RL
  training loops or any code path that holds contact data across a step.
- **PhysX instance lifecycle (Python)**: call `physx.destroy()` explicitly,
  normally from a `finally` block, to guarantee deterministic cleanup of native
  resources. `destroy()` is checked, so a cleanup failure raised from `finally`
  becomes the active exception while Python preserves any in-flight exception
  as chained context. Applications that need different exception precedence
  must catch and log cleanup failures explicitly. ovphysx 0.6 removes the
  former `release()` spelling and the main-object context-manager protocol. If
  an instance is garbage-collected *during* the run (dropped mid-program rather
  than held to interpreter shutdown), `__del__` emits a `ResourceWarning`
  before calling `destroy()` itself. The warning is silent by default (filtered
  out unless `python -W default::ResourceWarning` or a test suite captures it)
  and mirrors Python file-object semantics. If both best-effort native cleanup
  attempts fail before returning a status, the finalizer emits a default-visible
  `RuntimeWarning`; the native instance may remain registered and unreachable.
  A process-exit hook applies the same non-throwing finalizer policy to
  still-live instances before interpreter finalization. This is a safety net
  for normal exit, not a replacement for deterministic `destroy()`: abrupt
  termination, a `fork` after construction, and cleanup failures can still leave
  native or disk resources.

## Dependency Management

The native distribution consists of two separate package roots: the ovphysx SDK
and the matching native ovstage archive. The Python distribution instead uses
the ovphysx wheel and its exact `ovstage` wheel dependency:

- ovphysx runtime dependencies are loaded from its packaged plugins directory
  (`_install/plugins/` in a source install, `ovphysx/plugins/` in the wheel).
- ovstage supplies its own headers, shared library, CMake package, and runtime
  tree. Native users download its archive separately and keep the package roots
  separate.
- The ovphysx wheel includes the physics runtime. It does not bundle or load
  OmniClient or its connection library.
- The exact `ovstage` wheel supplies the ovstage runtime together with OmniClient, its
  connection library, the USD resolver, and
  the internal namespaced OpenUSD runtime ovstage uses to ingest USD scenes; the
  ovphysx wheel ships no OpenUSD library, USD plugin registry, or resolver of
  its own.
- Python's bulk data access is the session read/write API (`PhysX.read` / `PhysX.write`); the
  deprecated `TensorBindingsAPI` remains for the deprecation window, and the legacy
  `ovphysx.tensors` compatibility layer is no longer shipped.
- `pip install ovphysx` resolves the ovstage wheel automatically. Native SDK
  users manually download both archives.
- Auto-detects library location through `getLibraryDirectory()`
- On Linux, pre-loads `libovstage.so` with `RTLD_GLOBAL` so `libovphysx.so` can
  bind its ovstage dependency from the separate Python package
- Offline-capable once both native packages or both declared wheels are installed

Source builds still fetch a namespaced monolithic OpenUSD package as a build-time
dependency (the ovruntime subproject's own USD dependency). The Python tests get
their python USD from stock pip `usd-core`; no internal USD monolith is fetched.
ovphysx itself links, loads, and pins no USD.

`libovphysx` and `libovphysx_internal` have no link-time dependency on USD, and
neither the SDK nor the wheel ships a USD runtime, a USD plugin registry, or a
`config.toml`. ovphysx never loads, preloads, or version-checks USD: ovstage
ingests USD scenes through its own internal namespaced OpenUSD runtime, and the
application owns whatever USD it authors with. There is no USD-linked variant
of ovphysx; scenes are attached through ovstage only, so exactly one USD image
is ever loaded in the process.

## Error Handling

**For C:**
- Check `result.status` on every call.
- On failure, call `ovphysx_get_last_error()` on the same thread to retrieve the error string (valid until the next ovphysx call on that thread).
- For `ovphysx_wait_op()`, iterate over `error_op_indices` and call `ovphysx_get_last_op_error()` per failed op index, then free the result with `ovphysx_destroy_wait_result()`.

**For Python:**
- Runtime errors are raised on failed calls.
- Use try/finally or context managers to ensure bindings are destroyed.
- Pass `raise_if_empty=True` to `create_tensor_binding()` when a zero-count binding should be treated as a configuration error.
- For optional or broad queries, keep the default `raise_if_empty=False` and check `binding.count` before reading.

## Warmup and Determinism

Tensor reads require a warmup step that initializes PhysX structures (and
DirectGPU buffers in GPU mode). This is done automatically on the first tensor
operation. If deterministic initial state matters, call `ovphysx_warmup()`
explicitly after USD load and before the first tensor read.

> **Ordering constraint:** all `clone()` calls must happen **before** warmup.
> Calling `clone()` after `warmup()` or after the first `step()` /
> `step_sync()` / `step_n_sync()` reallocates physics structures and corrupts already-initialized
> solver state. The C runtime rejects this ordering with
> `OVPHYSX_API_INVALID_ARGUMENT`, surfaced in Python as `RuntimeError`. If you
> need to clone after warmup, call `reset_stage()`, wait for it to complete,
> then reload the source scene or reattach its ovstage before cloning again.

GPU dynamics and DirectGPU TensorAPI are separate choices.

**Process-level CPU-only mode** must be set before creating any instance. To keep
ovphysx itself from opening the CUDA driver (for example on a CPU-only deployment):

```python
from ovphysx import PhysX

def create_cpu_only_physx() -> PhysX:
    PhysX.set_cpu_mode(True)  # before the first PhysX() so ovphysx stays off CUDA
    assert PhysX.get_cpu_mode() is True
    return PhysX()
```

`PhysX.get_cpu_mode()` / `ovphysx_get_cpu_mode()` report the effective hard
CPU-only policy (`set_cpu_mode(True)` or `OVPHYSX_DISABLE_GPU`). They do **not**
report per-scene USD `physxScene:enableGPUDynamics`, a CUDA ordinal, or
attach-time resolved CPU versus GPU dynamics -- those remain separate controls
by design. Successful instance creation also emits an INFO line containing
`process_cpu_only=...` (and, when not hard CPU-only, create-args intent such as
`active_cuda_gpus=no_override` for empty input versus `-1` for explicit
automatic selection). This hard-policy slice does not replace a host that still
needs attach-time confirmation of scene dynamics before dropping a defensive
`set_cpu_mode(True)`.

`OVPHYSX_DISABLE_GPU` is read live before `ovphysx_initialize` and latched when
initialize succeeds, so an observational pre-init `get_cpu_mode()` cannot
permanently miss a later setenv before init.

For deployment environments, setting `OVPHYSX_DISABLE_GPU` before ovphysx
initialization provides the equivalent process-wide CPU-only policy.

Other libraries in the process may still open the CUDA driver. Loading an
ovstage-backed stage currently does, and constructing a `warp.array` through
`PhysX.read()` or `PhysX.write()` does when Warp was built with CUDA. Use a
CPU-only Warp build to keep those Python paths driverless.

This forces all PhysX scenes in the process to use CPU dynamics, overriding
any `physxScene:enableGPUDynamics=true` in the USD stage. The mode is sticky
for the process lifetime as soon as enabling it succeeds. For per-scene CPU
control (mixed CPU/GPU process without this flag), author each scene explicitly:
`physxScene:enableGPUDynamics=false` and `physxScene:broadphaseType="MBP"`.

**GPU dynamics** (whether the PhysX simulation pipeline runs on GPU) is enabled
by default (`physxScene:enableGPUDynamics` defaults to `true` in the PhysX
schema). ovphysx does not read or write this setting itself.

High-throughput CUDA TensorBinding and ContactBinding views additionally require
opting into DirectGPU before instance creation:

```python
from ovphysx import PhysX, PhysXConfig

def create_direct_gpu_physx() -> PhysX:
    return PhysX(
        config=PhysXConfig(
            carbonite_overrides={
                "/physics/suppressReadback": True,
            },
        ),
    )
```

Use this for IsaacLab-style tensor workloads that read state or contact tensors
every step. Leave DirectGPU off for scenes that need contact modification, such
as surface velocity or custom contact callbacks.

After creating a tensor binding, query Python `binding.native_device` or C
`ovphysx_get_tensor_binding_native_device()` before allocating its DLPack
buffer. The result is the binding's actual no-staging TensorAPI placement:
state bindings are CUDA only with DirectGPU and are CPU otherwise, even when
the scene uses GPU dynamics. CPU-only property bindings remain on the host in
DirectGPU mode. Treat this binding query as authoritative instead of inferring
placement from `active_cuda_gpus` or `/physics/suppressReadback`.

## Scene Cloning

Two paths duplicate a scene subtree for large-scale parallel simulation:

1. **Direct ovphysx API.** After an ovstage-backed scene has been attached and
   drained, callers may use `ovphysx_clone()` (C), `PhysX::clone()` (C++), or
   `PhysX.clone()` (Python). Pass a source path,
   a list of target paths, and an optional flat array of per-target
   transforms `(px, py, pz, qx, qy, qz, qw)` (position + imaginary-first
   quaternion; `NULL` co-locates every clone on the source). Each
   `anchor_transforms` entry is the absolute world pose of the exact target
   subtree root. For example, when cloning
   `/env0/Robot` to `/env1/Robot`, pass the final world pose of `/env1/Robot`,
   not the world pose of `/env1`. Descendants keep their poses relative to the
   source subtree root. PhysX environment ids isolate clones only under GPU
   dynamics + GPU broadphase. In CPU mode, co-located clones share one collision
   space, so pass spatially disjoint anchor transforms. The runtime logs
   `EnvIds requested but gpu dynamic is disabled` when the isolation asked for is
   not in effect; that record reaches the Carbonite log rather than Python's
   `warnings` module, so observe it through `enable_python_logging()` or
   `ovphysx_set_log_callback()` ([Logging](#logging)). Replication executes
   inline. C and Python return an
   already-complete op index (`ovphysx_wait_op` remains valid and returns immediately), while the
   C++ wrapper reports it through the optional `outOpIndex` out-param. Cloning is
   driven by the PhysX replicator: each target becomes a
   real physics object — articulations are reconstructed, not flattened —
   addressable by its target path. It happens in the simulation's internal
   representation only; no USD prims are authored at the target paths.
   When one logical environment is assembled from **several** clone calls (a
   heterogeneous ClonePlan: first every environment's robot, then every
   environment's object), pass the optional per-target `env_ids` with the same
   ids in every call — objects that share an id share a runtime environment and
   keep colliding with each other while staying isolated from other
   environments. Without `env_ids` each call numbers its copies afresh, so
   same-environment objects cloned by different calls would never collide. The
   [cloning tutorial](tutorials/cloning.md) includes the complete Python and C
   examples.

2. **Build the scene up front through ovstage.** Apps that own the ovstage
   `Stage` can duplicate the source subtree on that Stage (`ovstage_clone`)
   *before* `update_from_ovstage()`, then hand the populated scene to ovphysx.
   This is scenegraph-level authoring that keeps all scene edits in one
   producer-owned stream and sidesteps the "clone before warmup" ordering hazard.

Use whichever fits your call-site. The direct API provides inline multi-target
convenience after the scene has been attached and builds full physics (including
articulations) for each clone; up-front ovstage duplication is the choice when
your app already orchestrates every scene edit through ovstage.

Neither cloning nor `step()` implicitly drains later ovstage edits. Applications
must seal those edit ordinals and call `ovphysx_update_from_ovstage()` before
cloning or stepping.


## Remote USD Loading

ovstage population accepts any URI that the Omniverse Client Library supports:

- **Local paths:** `/path/to/scene.usd` (as before)
- **Omniverse Nucleus:** `omniverse://server/path/scene.usd`
- **S3 (HTTPS):** `https://my-bucket.s3.us-east-1.amazonaws.com/path/scene.usd`
- **Azure Blob:** `https://account.blob.core.windows.net/container/scene.usd`

ovstage loads its client library when the application creates an ovstage
`Stage`. No additional setup is needed for Nucleus URIs when the server allows
anonymous access.

> **Note:** Use HTTPS S3 URLs (virtual-hosted style), not `s3://` URIs. The OmniClient library resolves S3 assets through HTTPS and adds AWS authentication automatically when credentials are configured.

### Configuring credentials

Remote-asset credentials live on the **Omniverse Client Library (OmniClient)** —
the asset resolver ovstage population uses to fetch S3 / Azure assets — not on
ovphysx. The application owns population, so it configures credentials directly on
OmniClient before populating ovstage from a private URL. ovphysx does not wrap
this (it only consumes an already-populated Stage). The examples below use
placeholder strings; do not commit real credentials in source code.

- **Kit-hosted Python:** when the host provides the `omni.client` module, use
  `omni.client.set_s3_configuration` and `omni.client.set_azure_sas_token`.
- **Standalone Python / C / C++:** create the ovstage `Stage` first so ovstage
  loads its client, then bind or call OmniClient directly through
  `omniClientSetS3Configuration2` and `omniClientSetAzureSASToken`.

```python
import ovphysx
import ovstage

def step_remote_scene(scene_url: str) -> None:
    # ovphysx never registers its codeless PhysX schemas; do it before the
    # first population call in the process.
    ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])
    stage = ovstage.Stage("remote-scene")
    physx = None
    try:
        # For a private URL, configure OmniClient credentials here. Creating
        # the Stage has loaded the client, and population has not started yet.
        physx = ovphysx.PhysX()
        ovstage.population.open_usd(
            stage,
            scene_url,
            ordinal=1,
            domains=ovstage.PopulationDomain.PHYSICS,
        )
        # attach_ovstage() reads at a sealed ordinal.
        stage.advance_write_floor(ordinal=1).wait()
        physx.attach_ovstage(stage, read_ordinal=1)
        physx.step_sync(1.0 / 60.0)
        physx.detach_ovstage()
    finally:
        if physx is not None:
            physx.destroy()
        stage.destroy()
```

### Notes

- OmniClient credentials are **process-global** — configure them **before** ovstage
  population of the remote URL.
- The `domains` mask in the `step_remote_scene` example is `PHYSICS`, which matches
  the shipped samples when the USD is known not to put physics under native
  scene-graph instances. For arbitrary content prefer `ALL` (equivalently
  `PHYSICS | RENDERING`) — refer to
  [Population domains](ovstage_integration.md#population-domains).
- OmniClient must be loaded before credentials are configured. Kit-hosted
  Python can import the host-provided `omni.client` module. Standalone callers
  create the ovstage `Stage` first, then bind the client API that ovstage loaded
  before configuring credentials and calling `open_usd()`. Constructing
  `PhysX()` neither loads nor validates OmniClient. `carb.omniclient.plugin` is
  not part of the static ovphysx bootstrap path.

## Scene Queries

ovphysx provides three scene query functions covering raycast, sweep, and overlap:

| Function | Purpose | Geometry input |
|---|---|---|
| `ovphysx_raycast` | Cast a ray | Origin + direction |
| `ovphysx_sweep` | Move a shape along a direction | Geometry desc + direction |
| `ovphysx_overlap` | Test shape overlap at a position | Geometry desc |

Each function accepts a **mode** parameter:

- `CLOSEST` -- return the single closest hit (0 or 1 result). Valid for
  `raycast` and `sweep` only; overlap has no direction, so the Python API
  raises `ValueError` for `overlap(mode=CLOSEST)`. The C API treats
  `CLOSEST` as `ALL` for overlap.
- `ANY` -- return whether any hit exists (0 or 1, hit fields zeroed)
- `ALL` -- return all hits

**Geometry types** for sweep/overlap:

- `SPHERE` -- radius + center position
- `BOX` -- half-extents + position + orientation quaternion
- `SHAPE` -- any UsdGeomGPrim by prim path (meshes use convex approximation internally)

Hit results are stored in an **internal buffer** owned by the ovphysx instance, valid
until the next scene query call on the same instance. C and C++ callers holding the
raw pointer from `ovphysx_raycast()` / `ovphysx_sweep()` / `ovphysx_overlap()` must
copy any hits they need before issuing another query.

The Python API copies hit fields into Python-owned dicts before returning, so
retaining results from earlier `raycast()` / `sweep()` / `overlap()` calls across
subsequent queries is safe.

### Python Example

```python
import ovphysx
from ovphysx import SceneQueryMode, SceneQueryGeometryType

def run_scene_queries(physx: ovphysx.PhysX) -> None:
    hits = physx.raycast(
        origin=[0, 10, 0],
        direction=[0, -1, 0],
        distance=100.0,
        mode=SceneQueryMode.CLOSEST,
    )
    if hits:
        print(f"Hit at distance {hits[0]['distance']}")

    physx.sweep(
        geometry_type=SceneQueryGeometryType.SPHERE,
        direction=[1, 0, 0],
        distance=50.0,
        radius=0.5,
        position=[0, 1, 0],
    )

    physx.overlap(
        geometry_type=SceneQueryGeometryType.BOX,
        half_extent=[1, 1, 1],
        position=[0, 0, 0],
        rotation=[0, 0, 0, 1],
    )
```

### Path Encoding

Hit results contain `collision`, `rigid_body`, and `material` fields as an opaque
`omni::physics::parse::ObjectKey.handle` -- a runtime-assigned identity, not a
uint64-encoded SdfPath. There is no client-side bit-cast that reproduces this
encoding, so a consumer cannot compare a hit's identity fields against a known
prim path by re-deriving the old path hash. Resolve these fields with
`ovphysx_scene_query_get_paths_from_ids()` (Python:
`PhysX.get_scene_query_paths_from_ids()`), the scene-query counterpart of
`ovphysx_contact_binding_get_other_actor_paths_from_ids()` for contact
bindings; see `changelog.md`'s `ovphysx_scene_query_hit_t` entry for the full
break this resolver closes.


## PhysX Pointer Interop

For advanced use cases that go beyond the TensorBindingsAPI -- such as custom joint
manipulation or direct body property access -- ovphysx can return raw PhysX SDK object
pointers by physics-object path and type.

```c
#include <ovphysx/ovphysx.h>

static ovphysx_result_t get_scene_and_physics(
    ovphysx_handle_t handle, void** out_scene, void** out_physics)
{
    ovphysx_result_t result = ovphysx_get_physx_ptr(
        handle, OVPHYSX_LITERAL("/World/physicsScene"),
        OVPHYSX_PHYSX_TYPE_SCENE, out_scene);
    if (result.status != OVPHYSX_API_SUCCESS) {
        return result;
    }

    ovphysx_string_t no_path = { NULL, 0 };
    return ovphysx_get_physx_ptr(
        handle, no_path, OVPHYSX_PHYSX_TYPE_PHYSICS, out_physics);
}
```

`PxPhysics` is process-global and has no physics-object path. Select it with
`OVPHYSX_PHYSX_TYPE_PHYSICS` and a zero-length string view. Both C
representations, `{ NULL, 0 }` and `{ "", 0 }`, have the same meaning.

A non-empty selector for `OVPHYSX_PHYSX_TYPE_PHYSICS` is invalid. Every other
PhysX object type continues to require a non-empty physics-object path.

The `ovphysx_physx_type_t` enum specifies which PhysX type to look up
(scene, actor, articulation link, joint, shape, and material, among others).
The C API returns `void*`; the caller casts to the appropriate PhysX SDK type.

For high-level classification without casting PhysX pointers, use
`ovphysx_get_object_type()` (Python: `PhysX.get_object_type()`), which returns
the `ovphysx_object_type_t` taxonomy. Standalone UsdPhysics joints classify as
`OVPHYSX_OBJECT_TYPE_JOINT`; plugin-registered custom joints classify as
`OVPHYSX_OBJECT_TYPE_CUSTOM_JOINT`; articulation joints classify as
`OVPHYSX_OBJECT_TYPE_ARTICULATION_JOINT`. Each pairs with the matching
`ovphysx_get_physx_ptr` selector (refer to `ovphysx_object_type_t` and
`ovphysx_physx_type_t`). `OVPHYSX_OBJECT_TYPE_INVALID` means no classified
simulation object at the path, not that the call failed. This enum is separate
from `ovphysx_sim_object_type_t`, which drives the session read/write API.

Casting requires the PhysX SDK C++ headers (for example `PxScene.h`,
`PxRigidDynamic.h`). The ovphysx SDK ships these headers under
`include/physx/`; `find_package(ovphysx)` sets `ovphysx_PHYSX_INCLUDE_DIR`
to point there. No PhysX library linking is needed.

Refer to the [PhysX Interop tutorial](tutorials/physx_interop.md) for a
complete sample that uses `setKinematicTarget()` on a `PxRigidDynamic*`.

The C++ experimental API provides a type-safe overload that deduces the
enum from the pointer type, preventing mismatches at compile time:

```cpp
#include <ovphysx/experimental/ovphysx.hpp>

static ovphysx_api_status_t get_physx_roots(
    ovphysx::PhysX& sdk,
    physx::PxScene*& out_scene,
    physx::PxPhysics*& out_physics)
{
    out_scene = nullptr;
    out_physics = nullptr;

    ovphysx_api_status_t status =
        sdk.getPhysXPtr("/World/physicsScene", out_scene);
    if (status != OVPHYSX_API_SUCCESS) {
        return status;
    }

    return sdk.getPhysXPtr("", out_physics);
}
```

PhysX pointer interop is exposed only through the C and C++ APIs. The Python
binding does not expose raw PhysX pointers; use the session read/write API
(`PhysX.read` / `PhysX.write`) for Python data-access workflows instead.

### Pointer Lifetime

Returned pointers are borrowed. Treat them as invalid after stage reset or
detachment, or after instance destruction, and reacquire them after attaching
and initializing another stage. Calls to `ovphysx_step()` do **not** invalidate
existing pointers. Do not call `release()` on returned pointers -- ovphysx owns
them. These rules also apply to the process-global `PxPhysics` pointer. Use the
PhysX SDK headers shipped for the same ovphysx build to preserve ABI compatibility.
Objects that callers explicitly create through `PxPhysics` follow the PhysX SDK's
ownership rules; the no-release rule applies to the pointer returned by ovphysx.

### Thread Safety

PhysX APIs on returned pointers must only be called between simulation
steps — specifically after `wait_op()` completes for the preceding step
and before the next `ovphysx_step()` call. Calling PhysX APIs while a
step is in-flight is a data race.

### Disabling simulation

Do not set or clear `PxActorFlag::eDISABLE_SIMULATION` on a
`PxRigidDynamic*` obtained from `ovphysx_get_physx_ptr()`. That path is
unsupported: ovphysx does not observe the change, and on DirectGPU scenes
the next read or write may address the wrong body without error.

Use the official path instead: ovstage `disableSimulation` through `ovphysx_write()` /
`OVPHYSX_ATTR_DISABLE_SIMULATION`.

That path disables a **standalone** rigid body. A point-instancer **instance** cannot be disabled
individually in this release -- `disableSimulation` is not an instancer-writable column and there is
no per-instance tensor route -- so disabling a single instance of an instancer is unsupported.

### Object-Change Notifications

To manage cached pointers across the events that invalidate them, subscribe
to physics-object change notifications. The C API exposes
`ovphysx_subscribe_object_changes()` / `ovphysx_unsubscribe_object_changes()`;
the experimental C++ wrapper returns an RAII
`ovphysx::ObjectChangeSubscription`. Subscribers receive three optional
callbacks:

- `on_object_created` — fires AFTER the object exists. Safe to call
  `ovphysx_get_physx_ptr()` from a deferred handler.
- `on_object_destroyed` — fires BEFORE the object is destroyed. Drop any
  cached pointer at this point; do not call `release()` on it.
- `on_all_objects_destroyed` — fires BEFORE a bulk teardown, for example
  `ovphysx_reset_stage()`. Flush the entire pointer cache; no per-object
  destruction events follow.

The pathless `PxPhysics` object is not identified by per-path create or destroy
callbacks. Invalidate any cached `PxPhysics` pointer at the explicit reset,
detach, and instance-destruction boundaries described above.

Subscriptions are **process-global**: a single subscription receives events
from every ovphysx instance in the process. There is no per-instance filter,
and callback payloads do not include a stage identifier — only `prim_path`.
Today ovphysx is single-attached-stage-per-process, so treat these callbacks
as one-stage-at-a-time and do not rely on them for cross-stage bookkeeping.

```c
#include <ovphysx/ovphysx.h>

typedef struct pointer_cache_t {
    unsigned int destroyed_count;
} pointer_cache_t;

static void on_destroyed(
    ovphysx_string_t path, ovphysx_physx_type_t type, void* user_data)
{
    (void)path;
    (void)type;
    pointer_cache_t* cache = (pointer_cache_t*)user_data;
    ++cache->destroyed_count;
}

static void on_all_destroyed(void* user_data)
{
    pointer_cache_t* cache = (pointer_cache_t*)user_data;
    cache->destroyed_count = 0;
}

static ovphysx_result_t start_observing_object_changes(
    pointer_cache_t* cache,
    ovphysx_subscription_id_t* out_subscription)
{
    ovphysx_object_change_callbacks_t callbacks = {0};
    callbacks.on_object_destroyed = on_destroyed;
    callbacks.on_all_objects_destroyed = on_all_destroyed;
    callbacks.user_data = cache;
    return ovphysx_subscribe_object_changes(&callbacks, out_subscription);
}

static ovphysx_result_t stop_observing_object_changes(
    ovphysx_subscription_id_t subscription)
{
    return ovphysx_unsubscribe_object_changes(subscription);
}
```

```cpp
#include <ovphysx/experimental/ovphysx.hpp>
#include <string_view>
#include <utility>

struct PointerCache {
    void drop(std::string_view) {}
    void clear() {}
};

static ovphysx::ObjectChangeSubscription observe_object_changes(
    PointerCache& cache)
{
    ovphysx::ObjectChangeCallbacks callbacks;
    callbacks.onDestroyed = [&cache](
        std::string_view path, ovphysx_physx_type_t) { cache.drop(path); };
    callbacks.onAllDestroyed = [&cache]() { cache.clear(); };
    return ovphysx::subscribeObjectChanges(std::move(callbacks));
}
```

**Threading:** callbacks may fire from internal worker threads during
`ovphysx_step()`, `ovphysx_step_sync()`, `ovphysx_clone()`, or
`ovphysx_reset_stage()`. Do **not** call other ovphysx APIs from inside a
callback — defer follow-up work to after the next `wait_op()` /
`wait_all()` returns, or after the synchronous call (`ovphysx_step_sync()`,
`ovphysx_reset_stage()`) returns.

**Known limitation:** `ovphysx_clone()` does not currently emit
`on_object_created` notifications. Pointer caches that need to track cloned
objects must be refreshed after the synchronous clone call returns.
`ovphysx_reset_stage()` reliably
emits `on_all_objects_destroyed` and is the supported path for bulk
pointer invalidation.

## Contact Data

ovphysx provides two complementary APIs for reading contact information.
Choose the one that fits your use case:

| | Contact Binding | Contact Report |
|---|---|---|
| **Use when** | You need aggregate force tensors for RL rewards/safety, or per-contact-point geometry with actor identities | You need raw per-step contact event data for custom sensors or collision debugging |
| **Data shape** | `[S, 3]` net forces, `[S, F, 3]` force matrix, or flat per-contact arrays plus a `[C, 2]` actor-ID tensor (all DLPack-compatible) | Variable-length arrays of event headers + contact points (raw buffers, zero-copy) |
| **API style** | Create a binding, then read tensors each step | Call once per step, receive pointers to internal buffers |
| **USD requirement** | Sensor prims must have `PhysxContactReportAPI` applied | Prims must have `PhysxContactReportAPI` applied |
| **Key functions** | `create_contact_binding()`, `read_contact_net_forces()`, `read_contact_force_matrix()`, `read_raw_contact_data()` | `get_contact_report()` |

### Contact Binding (Aggregate Force Tensors)

Contact bindings give you aggregate net force vectors between sets of sensor
and filter bodies, delivered as DLPack tensors that work on both CPU and GPU.
Every authored USD prim named by `sensor_patterns` must have
`PhysxContactReportAPI` applied; filter prims need no extra schema. Filter
patterns are still resolved against physics-registered objects, though (a
collider, rigid body, or other physics registration) — a pattern that only
names an arbitrary non-physics USD prim, for example a plain visual `Mesh` with
no physics schema at all, matches nothing.

Refer to the [Contact Binding tutorial](tutorials/contact_binding.md) for a full
walkthrough. Key points:

- Create the binding **before** the first step whose contacts you want to observe.
- Net forces shape: `[S, 3]` — one force vector per matched sensor.
- Force matrix shape: `[S, F, 3]` — per (sensor, filter) pair.
- CUDA output tensors require DirectGPU TensorAPI; `physxScene:enableGPUDynamics=true` by itself
  only selects GPU dynamics. Refer to [Warmup and Determinism](#warmup-and-determinism).
- `dt` for impulse-to-force conversion is taken automatically from the last
  successful `step()`, `step_sync()`, or `step_n_sync()` call.

### Contact Report (Per-Point Event Data)

The contact report exposes the raw per-step contact events collected by
ovphysx. This gives you every individual contact point with
position, normal, impulse, and separation — useful for custom contact sensors,
collision debugging, or building higher-level contact processing.

Prims must have `PhysxContactReportAPI` applied in the USD stage for contacts
to be reported.

```c
#include <ovphysx/ovphysx.h>

static ovphysx_result_t read_contact_report(ovphysx_handle_t handle)
{
    const ovphysx_contact_event_header_t* headers = NULL;
    const ovphysx_contact_point_t* points = NULL;
    const ovphysx_friction_anchor_t* anchors = NULL;
    uint32_t num_headers = 0;
    uint32_t num_points = 0;
    uint32_t num_anchors = 0;

    return ovphysx_get_contact_report(
        handle,
        &headers,
        &num_headers,
        &points,
        &num_points,
        &anchors,
        &num_anchors);
}
```

The friction anchor parameters are optional -- pass NULL to skip them.

The C API returns typed struct pointers defined in `ovphysx_types.h`.
The C++ experimental API provides aliases (`PhysX::ContactEventHeader`,
`PhysX::ContactPoint`, `PhysX::FrictionAnchor`) and a typed
`getContactReport()` method.

Data is valid until the next simulation step. A typical usage pattern:

1. Apply `PhysxContactReportAPI` to prims of interest.
2. `step()` + `wait_all()`.
3. Call `get_contact_report()` to read that step's contacts.
4. Parse event headers for contact pairs; index into contact data for per-point details.

> **⚠️ Lifetime hazard (default zero-copy mode).** The arrays / pointers
> returned by `get_contact_report()` with `copy=False` (the C default and the
> Python default) are views into internal C buffers that are valid **only
> until the next `step()` / `step_sync()`**. After the next step the buffers
> may be reallocated or reused — accessing the views is undefined behavior
> (silent data corruption or segfault). Python cannot detect this dangling
> state.
>
> For RL training loops or any code path that retains contact data across a
> step, use **`copy=True`** in Python to receive Python-owned `list[dict]`
> for each section (safe to hold across steps, small per-step overhead).

**Buffer capacity between steps.** The runtime keeps reusable C-heap vectors for
contact headers, contact points, and friction anchors. Each step clears and
refills them, but capacity grows to the peak contact count seen so far while
physics scenes remain loaded. That growth is outside the Python heap and is not
visible to `tracemalloc`. Call `reset_stage()` to tear down the attached physics
session and release native buffers. `reset_stage()` detaches the stage;
`resetSimulation()` / `releasePhysicsObjects()` delete all `PhysXScene` objects
(and their `ContactReport` state), which releases the grown vector capacity.

In Python (zero-copy, default):

```python
from ovphysx import PhysX

def print_borrowed_contacts(physx: PhysX) -> None:
    report = physx.get_contact_report()
    for i in range(report["num_headers"]):
        header = report["headers"][i]
        print(
            f"pair {i}: actor0={header.actor0:#x}, "
            f"{header.numContactData} points"
        )
    for j in range(report["num_points"]):
        point = report["points"][j]
        print(
            f"  pos=({point.position[0]:.3f}, "
            f"{point.position[1]:.3f}, {point.position[2]:.3f})"
        )
    # Finish access before the next step; native buffers will be reused.
```

In Python (safe across steps, `copy=True`):

```python
from ovphysx import PhysX

def print_owned_contacts_after_step(physx: PhysX, dt: float) -> None:
    report = physx.get_contact_report(copy=True)
    physx.step_sync(dt)  # native buffers are reused, but report is owned
    for header in report["headers"]:
        print(
            f"actor0={header['actor0']:#x}, "
            f"{header['numContactData']} points"
        )
    for point in report["points"]:
        position = point["position"]
        print(
            f"  pos=({position[0]:.3f}, "
            f"{position[1]:.3f}, {position[2]:.3f})"
        )
```

## Running With Other Carbonite Users

ovphysx can share a process with other OV libraries that use Carbonite.
ovphysx loads and links no USD library of its own, so it adds no USD image to
the process and does not inspect the ones other libraries, including ovstage,
bring with them.

ovphysx does not reuse a PhysX runtime loaded by another Carbonite user.
`IPhysxSimulation` is an internal direct runtime table, not a Carbonite-acquired
interface. ovphysx still uses Carbonite to load plugin binaries and dependencies,
then starts its statically linked `OvruntimePhysX` runtime directly. A foreign Carbonite
framework is allowed by default, but `OVPHYSX_COEXIST_REFUSE=1` makes ovphysx fail
fast, and foundation-interface mismatches still fail with clear probe errors.
Compatible OV libraries can share Carbonite without sharing the PhysX runtime.

ovphysx ships its PhysX USD schemas as codeless plugins and never registers
them itself. The application registers them with the USD runtime it owns before
the first ovstage population call in the process: obtain the root with
`ovphysx_get_codeless_schema_root()` and pass it to
`ovstage_population_register_usd_schemas()` (Python:
`ovstage.population.register_usd_schemas([str(ovphysx.codeless_schema_root())])`).
When another USD-aware subsystem, such as ovrtx, shares the process, register
its schemas through its own documented mechanism in the same early window,
before any subsystem populates a stage or otherwise touches the USD schema
registry; API names and initialization requirements are subsystem-specific.

There is no implicit registration: `ovphysx_create_instance()` does not
register the schemas, and a late registration cannot repair an already-built
schema registry. Refer to [Physics Schemas](physics_schemas.md).

Important process rules:

- ovphysx brings no USD library into the process and does not check which USD
  libraries other subsystems loaded.
- Register every subsystem's USD schemas before the first stage population or
  schema-registry access. Late calls cannot repair an already-populated schema
  registry.
- Sharing a pre-loaded Carbonite PhysX runtime stack is not supported.
- Kit or Isaac Sim processes that already loaded their physics extensions are
  not supported ovphysx embedding targets.
- Use `active_cuda_gpus` for CUDA device selection. ovphysx applies an explicit
  selection as part of serialized scene attachment; other subsystems must not
  reconfigure physics GPU selection during that attachment. A non-empty
  `active_cuda_gpus` value takes precedence over `scene_multi_gpu_mode`. The
  empty value makes no ovphysx ordinal override (a fresh/default PhysX process
  selects automatically); `"-1"` explicitly requests automatic single-device
  selection. Applications that require a deterministic ordinal pass it
  explicitly before the first GPU scene attaches. An already-created PhysX CUDA
  context manager is not retargeted; use a new process to select a different
  ordinal. The expert DirectGPU setting `/physics/suppressReadback` is
  host-managed and should be configured before instance creation rather than
  changed while physics is active.
- If another Carbonite user already set the app directory, ovphysx preserves it.

Schema-path registration governs coexistence in the *process*. If the subsystems
also share one ovstage Stage, that Stage must be populated with `ALL` (or
`PHYSICS | RENDERING`) — refer to
[Population domains](ovstage_integration.md#population-domains).

## Logging

ovphysx uses [Carbonite](https://docs.omniverse.nvidia.com/kit/docs/carbonite/) as its internal logging backend.
By default, the process-scoped level for libovphysx's named Carbonite sources,
`omni_physx_sdk`, `omni.physx`, and `ovphysx_internal`, is
`LogLevel.WARNING`. Any unnamed source and every host and dependency source
remain unchanged. The application callback observes the Carbonite process log
stream; its own minimum severity and channel filter control records from every
source. `LogLevel.NONE` mutes only the three named sources; it is not a
whole-runtime or process mute.

### Controlling the Log Level

#### Python

```python
import ovphysx

ovphysx.set_log_level(ovphysx.LogLevel.VERBOSE)
print(ovphysx.get_log_level())
```

#### C

```c
#include <ovphysx/ovphysx.h>

int main(void) {
    /* Applies to the omni_physx_sdk, omni.physx, and ovphysx_internal sources. */
    if (ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE).status != OVPHYSX_API_SUCCESS)
        return 1;

    uint32_t current = ovphysx_get_log_level();
    return current == OVPHYSX_LOG_VERBOSE ? 0 : 1;
}
```

### Custom Log Callbacks (C)

Set the single application callback to receive log messages programmatically.
Setting another callback replaces it; passing `NULL` disables it after accepted
invocations drain. Calls for one registration are serialized. The optional
comma-separated `channel=level` filter uses raw-prefix matching. The longest
matching prefix wins, with later equal-length rules winning ties. Message and
channel are valid, NUL-terminated views for the callback duration; timestamp is
Unix-epoch seconds.

```c
#include <ovphysx/ovphysx.h>
#include <stdint.h>
#include <stdio.h>

void my_logger(ovphysx_log_level_t level,
               ovphysx_string_t message,
               ovphysx_string_t channel,
               double timestamp,
               void* user_data) {
    (void)user_data;
    fprintf(stderr, "[%.*s][%d][%.3f] %.*s\n",
            (int)channel.length, channel.ptr, (int)level, timestamp,
            (int)message.length, message.ptr);
}

int main(void) {
    ovphysx_string_t filter = OVPHYSX_LITERAL(
        "omni_physx_sdk=verbose,omni.physx=verbose,ovphysx_internal=warning");
    if (ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE).status != OVPHYSX_API_SUCCESS)
        return 1;
    if (ovphysx_set_log_callback(
            OVPHYSX_LOG_WARNING, &filter, my_logger, NULL).status != OVPHYSX_API_SUCCESS)
        return 1;
    /* No simulation work is required to demonstrate callback configuration. */
    if (ovphysx_flush_log(OVPHYSX_TIMEOUT_INFINITE).status != OVPHYSX_API_SUCCESS)
        return 1;
    if (ovphysx_set_log_callback(
            OVPHYSX_LOG_DEFAULT, NULL, NULL, NULL).status != OVPHYSX_API_SUCCESS)
        return 1;
    /* Safe to destroy callback resources here. */
    return 0;
}
```

Invalid severity or filter arguments leave the prior callback unchanged. Any
other error may be reported after a replacement was published; conservatively
keep the candidate callback and user-data resources alive until a later
successful replace, disable, or shutdown drains the slot.

### Controlling Default Console Output

By default, Carbonite logs to the console. When a custom callback is registered
that also writes to the console, output may be doubled.
`ovphysx_enable_default_log_output()` controls Carbonite's process-global
built-in console logger, so disabling it affects every Carbonite tenant in the
process. Multi-tenant hosts should normally own that policy themselves. A
standalone application can suppress the built-in logger as follows:

#### Python

```python
import ovphysx

ovphysx.enable_python_logging()
ovphysx.enable_default_log_output(False)
```

#### C

```c
#include <ovphysx/ovphysx.h>

int configure_logging(ovphysx_log_callback_t callback) {
    if (ovphysx_set_log_callback(
            OVPHYSX_LOG_VERBOSE, NULL, callback, NULL).status != OVPHYSX_API_SUCCESS)
        return 1;
    if (ovphysx_enable_default_log_output(false).status != OVPHYSX_API_SUCCESS)
        return 1;
    return 0;
}
```

### Python Logging Bridge

Route native log messages into Python's standard `logging` module. The bridge
owns the single native callback slot, so enabling it replaces any C-level
callback previously set through the same process. Its default callback
threshold is `LogLevel.VERBOSE`, so the libovphysx source threshold remains the
only additional filter for `omni_physx_sdk`, `omni.physx`, and
`ovphysx_internal` records unless the application passes a stricter
`min_severity`:

```python
import logging
import ovphysx

# Route native messages to the "ovphysx" Python logger
ovphysx.enable_python_logging()

# Add a handler to see the output
logging.getLogger("ovphysx").addHandler(logging.StreamHandler())
logging.getLogger("ovphysx").setLevel(logging.DEBUG)

# Instance startup and shutdown emit native messages through the bridge.
physx = ovphysx.PhysX()
physx.destroy()
ovphysx.disable_python_logging()
```

Destroying the final `PhysX` instance successfully shuts down the current
Python process-lifecycle scope and disables the bridge. Call
`enable_python_logging()` again after creating an instance in a new lifecycle
scope. Final shutdown waits for an already-started bridge transition and keeps
the Python callback owner alive until native callback draining returns; bridge
enable/disable calls started during shutdown fail as overlapping transitions.
Calling `PhysX()` or `PhysX.destroy()` from inside the native log callback is
rejected before it can wait on or change process-lifecycle state; retry after
the callback returns. Process initialization and final shutdown publish
transitions without holding the process-lifecycle lock while native code may
deliver or drain callbacks. Concurrent constructors wait on a condition while
process initialization finishes. Construction racing final shutdown fails fast
so a callback dependency cannot wait on the shutdown that is draining it. A
callback must not synchronously wait for work that may emit into the same
serialized callback registration. If Python is interrupted when native process
initialization may already have committed, the wrapper conservatively owns and
shuts down that lifecycle before propagating the interruption. If a native
callback change may already have committed, the bridge retains every possibly
published callback owner until a later successful replace, disable, or shutdown
drains the native slot.

## OmniPVD Recording

ovphysx supports recording PhysX simulation internals to `.ovd` files through
OmniPVD. The resulting files can be opened in a compatible Kit application with
the OmniPVD extension for frame-by-frame inspection of shapes, contacts, and
solver state. The reader must support both the recording's OmniPVD stream
version and its independent PhysX OVD integration version; refer to the full
tutorial for the current versions and compatibility policy.

Startup OmniPVD output and late-recording capability are selected at instance
creation. The default FILE startup transport also uses a recording directory:

| Python field | C builder | Description |
|---|---|---|
| `omnipvd_ovd_recording_directory` | `ovphysx_config_entry_omnipvd_ovd_recording_directory()` | Directory for `.ovd` output |
| `omnipvd_output_enabled` | `ovphysx_config_entry_omnipvd_output_enabled()` | Enable recording |
| `omnipvd_recording_capable` | `ovphysx_config_entry_omnipvd_recording_capable()` | Permit recording to start later without enabling startup output |

Set `omnipvd_output_enabled` before creating the PhysX instance. For FILE, also set
`omnipvd_ovd_recording_directory`; it is auto-created if needed. TCP does not use it.
Startup output implicitly installs late-recording capability. To enable only
late recording, set `omnipvd_recording_capable=True` before creating the first
instance instead.

The capability choice is process-wide and latched when the shared runtime is
created. Default instances deliberately cannot start recording later: with both
startup output and capability disabled, ovphysx passes a null `PxOmniPvd` to
PhysX, avoiding incremental sampler, factory-listener, and sampling-mutex
overhead. This is the only zero-OmniPVD-overhead configuration. Explicit late
recording capability creates the provider and sampler at `PxPhysics` creation
and enables OVD and collision readback on attached scenes even while recording
is idle; DirectGPU scenes therefore retain that readback cost. A second live
instance cannot change this creation-time choice.

The startup transport has four additional typed fields:

| Python field | C builder | Description |
|---|---|---|
| `omnipvd_transport` | `ovphysx_config_entry_omnipvd_transport()` | Exact lowercase `"file"` or `"tcp"`; defaults to FILE |
| `omnipvd_tcp_address` | `ovphysx_config_entry_omnipvd_tcp_address()` | TCP listener address |
| `omnipvd_tcp_port` | `ovphysx_config_entry_omnipvd_tcp_port()` | TCP listener port, 1 through 65535 |
| `omnipvd_tcp_timeout_ms` | `ovphysx_config_entry_omnipvd_tcp_timeout_ms()` | Blocked-send timeout in milliseconds; 0 leaves it at the OS default and uses a 3000 ms connect window |

For TCP, ovphysx synchronously attempts to connect to a ready trusted-plaintext
listener. A failed open is logged and cleaned up; the instance may still succeed
with recording inactive. FILE alone renames/imports files.

```python
import ovstage
from ovphysx import PhysX, PhysXConfig, codeless_schema_root

physx = PhysX(
    config=PhysXConfig(
        omnipvd_ovd_recording_directory="/tmp/pvd_output",
        omnipvd_output_enabled=True,
    )
)

# Register the codeless PhysX schemas before the first population call.
ovstage.population.register_usd_schemas([str(codeless_schema_root())])
stage = ovstage.Stage("recorded-scene")
ovstage.population.open_usd(
    stage, "scene.usda", ordinal=1, domains=ovstage.PopulationDomain.PHYSICS
)
# attach_ovstage() reads at a sealed ordinal.
stage.advance_write_floor(ordinal=1).wait()
physx.attach_ovstage(stage, read_ordinal=1)
for i in range(100):
    physx.step_sync(1/60)

physx.detach_ovstage()  # finalizes recording -> <timestamp>_rec.ovd
physx.destroy()
stage.destroy()
```

To begin recording after the instance and scene already exist, use the
late-recording API. FILE takes an exact output path; TCP connects to a listener
that is already ready. After stopping, the same instance can start another
session with either transport. Refer to the
[complete late TCP-to-FILE Python sequence and C API
example](tutorials/omnipvd_recording.md#late-tcp-recording) in the OmniPVD
recording tutorial.

The C equivalents are `ovphysx_start_recording()`,
`ovphysx_is_recording()`, and `ovphysx_stop_recording()`. Only one recording may
be active in the shared runtime. Starting while active returns
`OVPHYSX_API_INVALID_STATE` and never replaces the current stream. A destination
validation, FILE open, or TCP connect failure may be retried, and a successful
stop permits another session to the same or a different FILE/TCP destination.
Late start requires a live physics stage in the shared runtime; calls before
its first attach or between detach and reattach return
`OVPHYSX_API_INVALID_STATE`.
Detaching the active stage or destroying its owning instance stops and finalizes
the shared session, including a session started by a peer handle, and clears its
public owner. After reattach, a capability-only runtime stays dormant and can
start a late recording immediately. Configured startup output instead starts a
new startup session owned by the reattaching instance; that instance must stop
the session before choosing a late destination. On cold startup, ownership is
reserved for the creating instance when creation succeeds and becomes observable
when the first stage attach starts sampling. Peer instances report inactive and
cannot stop the owner's session.

These synchronous APIs follow the ovphysx same-thread contract. The caller must
serialize recording calls on each handle and recording/attach/detach/destroy
transitions across all handles sharing the runtime; concurrent calls are not
supported. Startup, late, and restarted sessions each capture the current core
PhysX, PhysXExtensions (including joints and custom geometry), and PhysXVehicle
state. Stopping releases the session's Vehicle PVD state; the next start creates
fresh handles and records the live vehicles again.

The [OmniPVD Recording tutorial](tutorials/omnipvd_recording.md) has compact
Python and C examples for both TCP startup and late TCP attach.

No OmniPVD-specific dependencies are needed beyond the matching ovphysx and
ovstage packages; the writer is built into the packaged PhysX runtime.

For the full tutorial with C examples and Kit inspection instructions, refer to [OmniPVD Recording](tutorials/omnipvd_recording.md).

## NVTX Profiling With Nsight Systems

ovphysx can emit NVTX ranges so that an [Nsight Systems](https://developer.nvidia.com/nsight-systems) capture shows what your process was doing, correlated with the CUDA activity Nsight already records. Two NVTX domains appear in the timeline:

| Domain | Contents |
|---|---|
| `ovphysx` | The ovphysx API calls: `ovphysx_step`, `ovphysx_step_sync`, `ovphysx_step_n_sync`, `ovphysx_wait_op`, `ovphysx_attach_ovstage`, `ovphysx_clone`, and the tensor binding create / read / write calls |
| `PhysX` | The PhysX SDK's own profile zones, nested inside the step ranges above, for both CPU and GPU work |

The instrumentation is compiled into release builds and the shipped wheel, so profiling needs no rebuild. Emission is off by default and is enabled in one of two equivalent ways.

Set the environment variable, which needs no change to your code:

```bash
OVPHYSX_NVTX=1 nsys profile -t nvtx,cuda python my_workload.py
```

Or enable it programmatically, which is useful when the process decides at runtime whether it is being profiled:

```python
from ovphysx import PhysX, PhysXConfig

physx = PhysX(config=PhysXConfig(nvtx_enabled=True))
```

In C, use `ovphysx_config_entry_nvtx_enabled(true)` in the `config_entries` array of `ovphysx_create_args`.

Both forms resolve to the process-wide `/physics/nvtxEnabled` Carbonite setting and must be in place **before** the instance is created, because the PhysX SDK profiler callback is installed while the SDK is created. `ovphysx_get_global_config_bool(OVPHYSX_CONFIG_NVTX_ENABLED, &value)` (Python: `physx.get_config_bool(ConfigBool.NVTX_ENABLED)`) reports the effective state.

Because this is a create-time decision, profile a slice of a long run by scoping the collection with Nsight's delay, duration, and capture-range options rather than by toggling the instrumentation.

Like all Carbonite settings, `/physics/nvtxEnabled` is process-global and sticky: once a process enables it, a later instance created with no config entry and no environment variable stays enabled. Turning it off again is explicit, with `OVPHYSX_NVTX=0` or a false config entry.

Notes:

- The `PhysX` domain depends on the profile zones being compiled into the PhysX SDK that ovphysx links, which is the case for the shipped `checked` PhysX libraries. A build against a PhysX `release` package has no zones to forward, and that domain stays silent while the `ovphysx` domain still works.
- The PhysX SDK's high-level phases (`Basic.simulate`, `Basic.collision`, ...) are cross-thread ranges, which Nsight reports separately from the thread-local ones (`nvtx_startend_sum` versus `nvtx_pushpop_sum`).
- `ovphysx_step` only enqueues work, so its range is short and the simulation cost appears under `ovphysx_wait_op`. That is the asynchronous execution model, not a measurement artifact.
- NVTX 3 is header-only, so nothing extra is linked or shipped. When disabled, a zone costs a single branch.
- This is independent of the Carbonite profiler (`/physics/exposeProfilerData`): either, both, or neither sink can be active.

For a runnable sample, memory-traffic attribution, and troubleshooting, refer to [NVTX Profiling](tutorials/nvtx_profiling.md). For recording options and the timeline UI, refer to the [Nsight Systems User Guide](https://docs.nvidia.com/nsight-systems/UserGuide/index.html).

You now have the core integration rules for building, running, and operating ovphysx. For the full C API reference, refer to the [C API Reference](api.md). For Python, refer to the [Python API Reference](python_api.rst).
