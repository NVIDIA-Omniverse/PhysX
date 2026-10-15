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
- [Task Receipts and Polling](#task-receipts-and-polling)
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
- [Profiling](#profiling)

## Relationship and Consumption Model

ovphysx provides a C API and Python bindings for ovstage-driven PhysX simulation.
Use ovphysx when you need USD-based physics simulation outside of Omniverse Kit with a small native SDK.

- Python: `pip install ovphysx` and `from ovphysx import PhysX`
- C/C++: download the ovphysx SDK and matching native ovstage archive as
  described in the [SDK Quickstart](tutorials/quickstart.md), keep the package
  roots separate, and link through CMake
- Porting a 0.6 application: [Migration from 0.6](migrating_0_6_to_0_7.md)

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
| `session_write.py` | [Migrating off Tensor Bindings](tensor_binding_migration.md) | Write `linearVelocity` through a session write, commit it, step, and verify the consequence by reading `position` back |
| `clone.py` | [Cloning](tutorials/cloning.md) | Clone a subtree before the first step and read rigid-body positions |
| `profiling.py` | [Profiling](tutorials/profiling.md) | Chrome trace and NVTX profiler backends |
| `vehicle_read_write.py` | [Vehicles](read_write/vehicles.md) | Read and write vehicle controls and wheel state |

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
| `output_read_c/` | [ovstage Integration](ovstage_integration.md) | Closed loop on the same scene; queries several output types, including `OVPHYSX_QUERY_TARGET_ARTICULATION`, which is empty because the USD has no articulations |
| `clone_c/` | [Cloning](tutorials/cloning.md) | Same three-target clone in C |
| `vehicle_read_write_cpp/` | [Vehicles](read_write/vehicles.md) | Vehicle session read/write, including `ovphysx_write` |
| `ovstage_output_cpp/` | [ovstage Integration](ovstage_integration.md) | Publish simulated world transforms back to ovstage |
| `hello_world_source_link/` | [Source-link build](tutorials/source_link_build.md) | Link the SDK from a source build |

For SDK setup, refer to [SDK Quickstart](tutorials/quickstart.md).

The tensor-binding API is **deprecated**; new code should use the session read/write API
(`ovphysx_read` / `ovphysx_write`). For the deprecated tensor binding shape/read/write semantics,
refer to [Tensor Bindings](tutorials/tensor_bindings.md).
Canonical enum-level definitions are in `include/ovphysx/ovphysx_legacy_types.h` (`ovphysx_tensor_type_t`).

## Execution Model

ovphysx preserves submission and data ordering. Existing task-producing calls
return a receipt: `ovphysx_enqueue_result_t` in C and `Task` in Python. Keep the
whole receipt and use `ovphysx_wait_task()` / `PhysX.wait_task()` to collect that
operation's outcome. A receipt reports admission, not execution success.

`step()` / `ovphysx_step()` starts asynchronous simulation. `clone()`,
`attach_ovstage()`, `update_from_ovstage()` and `detach_ovstage()` currently
finish inline but still return receipts; their execution
failures are reported when collected. Native query, read/write session open,
fetch, commit, release and articulation-kinematic update also return receipts and
currently execute inline. No host-nonblocking guarantee follows from a task receipt.
Keep caller-owned inputs and output slots alive until completion; inspect output
handles and groups only after a successful wait. A successful fetch with a null
group is EOF. Rejected submissions instead return no
task and expose an immediate error. `step_sync()` / `ovphysx_step_sync()` steps
and waits in one call. Repeat it in a local loop when a workload uses decimation.

In-stream ovphysx operations preserve their existing synchronization and data
ordering. Before external CPU access, rendering, logging or other consumers need
the result, wait on the relevant task. Explicit CUDA producer-event and
consumer-stream dependencies remain necessary where documented; collecting a
task does not replace those handoffs.

Python tensor-binding, contact-binding, and SDF calls that accept caller-owned
CUDA device or managed-memory buffers finish pending work on the current Warp
stream or use the producer's
`__dlpack_device__` / `__dlpack__(stream=...)` handoff before accessing storage.
This includes output-buffer initialization and cached binding reuse. For pinned
host buffers, callers must finish pending CUDA work before the call. Raw
`DLTensor` descriptors carry no producer stream: callers must finish pending work
or pass the API's explicit stream/event arguments where available. Session write
commits still use their explicit CUDA stream/event arguments.

A failed task does not automatically make a later task fail. Waiting on successful
B reports only B; an earlier failed A remains reportable through its own wait or
`wait_all()`, even when A's original receipt was discarded. Later calls still
validate actual simulation state and prerequisites.

Run custom application code directly after successfully waiting on the relevant
task. Use `wait_all()` when that code needs every earlier outcome collected;
it also acknowledges earlier failures. The task API exposes completion of native
operations and does not schedule application callbacks. Log,
object-change and contact callbacks keep their own documented contracts.

### Stage lifecycle completion

Attach, update and detach use the same admission and completion contract as
other tasks. Invalid arguments, an already-attached instance or an update without
an attachment reject before changing the attachment. Detach also rejects
conflicting companion transitions before task admission. Runtime failures after
admission belong to the task's outcome.
Wait successfully before using a newly attached scene or the result of an update.
Keep the Stage and the sealed ordinal data needed by attach/update alive until
the task finishes. An attached Stage stays alive until successful detach or
instance destruction. A failed teardown may still own the Stage and may already
have retired some resources; keep it alive while handling the failure.

`detach_ovstage()` replaces the removed `reset_stage()`. It tears down physics,
preserves accumulated simulation time, and does not restore initial poses.
Repeated detach succeeds with a new task receipt. To restore episode state,
write the required poses, velocities and joint state; to rebuild physics,
successfully detach and then attach the desired Stage snapshot.

`is_stage_attached()` remains synchronous: it reads current attachment ownership
without waiting, creating a task, or collecting earlier outcomes. It is useful
after failures but does not establish simulation readiness or resource validity.
Wait for the relevant lifecycle task before asking for its settled state. A false
snapshot while attach is pending does not permit destroying that Stage.

## Configuration

Process-global keys (`ovphysx_global_config_key_t` / `GlobalConfigKey`) are
configured independently of instances. Set startup-only keys (`CPU_ONLY`,
`OVD_RECORDING_CAPABLE`, profiler, and cooked-cache location) **before the first
instance**. Other keys remain writable only while their backend can honor the change; scene-creation
keys reject writes while a stage is attached. Configure every setting with a
`GlobalConfigKey` through `set_global_config`, and create the instance with no
config arguments for those settings.
The only current per-instance key is CUDA GPU selection
(`OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS` / `ConfigKey.ACTIVE_CUDA_GPUS`): set it
**after** create and **before** attach. After attach, changing it returns
`OVPHYSX_API_INVALID_STATE` and keeps the prior value.

Environment variables stay authoritative over the matching global setter.
Known overrides: `OVPHYSX_DISABLE_GPU` (`CPU_ONLY`),
`OVPHYSX_PROFILER` (`PROFILER_BACKEND`), and
`OVPHYSX_PROFILER_OUTPUT` (`PROFILER_OUTPUT_PATH`).
`get_global_config` reports the effective value.

`PhysXConfig`, typed config-entry arrays, and `ovphysx_create_args` /
`ovphysx_create_instance_with_args` are the legacy path. New code uses the
scoped setters below where a matching key exists, including `OVD_RECORDING_CAPABLE`; see
[OmniPVD Recording](#omnipvd-recording).

### C

```c
#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_config.h>

int main(void)
{
    if (ovphysx_initialize().status != OVPHYSX_API_SUCCESS)
        return 1;

    ovphysx_result_t result = ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_DISABLE_CONTACT_PROCESSING,
        ovphysx_config_value_bool(true));
    if (result.status != OVPHYSX_API_SUCCESS)
        return 1;
    result = ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, ovphysx_config_value_int64(4));
    if (result.status != OVPHYSX_API_SUCCESS)
        return 1;

    ovphysx_instance_handle_t handle = OVPHYSX_INVALID_HANDLE;
    if (ovphysx_create_instance(&handle).status != OVPHYSX_API_SUCCESS) {
        ovphysx_shutdown();
        return 1;
    }

    result = ovphysx_set_config(
        handle,
        OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS,
        ovphysx_config_value_string(OVPHYSX_LITERAL("0")));
    ovphysx_config_value_t threads = ovphysx_config_value_int64(0);
    if (result.status == OVPHYSX_API_SUCCESS)
        result = ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_NUM_THREADS, &threads);

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
from ovphysx import ConfigKey, GlobalConfigKey, PhysX, get_global_config, set_global_config


def read_configuration() -> tuple[int | None, bool | None]:
    set_global_config(GlobalConfigKey.DISABLE_CONTACT_PROCESSING, True)
    set_global_config(GlobalConfigKey.NUM_THREADS, 4)
    physx = PhysX()
    try:
        physx.set_config(ConfigKey.ACTIVE_CUDA_GPUS, "0")
        threads = get_global_config(GlobalConfigKey.NUM_THREADS)
        enabled = get_global_config(GlobalConfigKey.DISABLE_CONTACT_PROCESSING)
        assert threads is None or isinstance(threads, int)
        assert enabled is None or isinstance(enabled, bool)
        return threads, enabled
    finally:
        physx.destroy()
```

Global keys are process-wide. `ACTIVE_CUDA_GPUS` is retained per instance until
attach.

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
from ovphysx import GlobalConfigKey, PhysX, set_global_config


def create_physx_with_cache(cache_directory: str) -> PhysX:
    set_global_config(GlobalConfigKey.COOKED_COLLIDER_CACHE_DIRECTORY, cache_directory)
    return PhysX()
```

In C, set `OVPHYSX_GLOBAL_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY` before
`ovphysx_create_instance()`. The first run with
a given cache cooks and stores the result; later runs with an unchanged scene are served from the
cache without re-cooking.

With no directory configured, nothing is persisted: cooked colliders go to a **process-private**
directory under the OS temp directory, so every run re-cooks. After all instances are destroyed,
`ovphysx_shutdown()` stops the datastore and removes that directory on Linux. Windows retains
the datastore until normal process exit. Its framework release callback attempts removal
after the datastore stops, but Windows cleanup remains best-effort: another dependency's
terminating exit hook can prevent that callback from running. Leftover private directories
may require normal temporary-directory cleanup. Python destroys still-live
`PhysX` instances from `atexit`, including after
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
`PhysX`. The cache needs a real path while the datastore is active. Its background writers and
open files must be released before directory removal. Filesystem removal
errors are reported on stderr. The temp location follows the platform's `TMPDIR`/`TMP`/`TEMP` convention; what
ovphysx will not do is derive a *persistent* cache location from the environment. The same
process-private fallback is used when a configured directory cannot be created or written.

The configured directory is applied when the framework first creates its datastore.
On Linux, destroy all instances and call `ovphysx_shutdown()`, then initialize and configure
the next lifecycle to change directories. Windows retains its datastore until process exit;
use a separate process for a different directory. Python's final `PhysX.destroy()` completes
SDK shutdown when no other lifecycle owners remain.

Cooked colliders are **not** reused across runs when:

- No cooked-collider cache directory is configured: cooking still runs, but nothing is persisted, so it re-cooks every run.
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

The SDK exposes a per-instance handle (`ovphysx_instance_handle_t`). Wrapper-owned state -- tensor and contact bindings, output query/read/write sessions, object-change subscriptions, task errors, debug-render settings, and scene-query hit buffers -- is isolated per handle. Foreign subordinate handles return `OVPHYSX_API_NOT_FOUND` and leave the owner untouched. Debug rendering is `OVPHYSX_QUERY_TARGET_DEBUG_RENDER` on that handle (settings and geometry through the read/write sessions); geometry is limited to scenes of the instance's attach.

Each instance holds at most one ovstage attach. Several instances may be attached at the same time, including stages that use the same prim paths. Attaching, stepping, detaching, or destroying one instance does not detach the others. The same ovstage instance cannot be attached twice.

Concretely, this means:

- Multiple `ovphysx_instance_handle_t` values can coexist. Instance operations take that handle first (`ovphysx_subscribe_object_changes`, `ovphysx_get_last_task_error`, and the rest of the instance surface).
- Process-global functions omit the handle: initialize/shutdown, `ovphysx_create_instance`, version queries, CPU-mode and global config, logging, `ovphysx_get_last_error`, `ovphysx_destroy_wait_task_result`, `ovphysx_writability`.
- The hard CPU-only policy, process-global config, logging, the SDK tolerance scale, and the recording transport are process-wide. Each attachment captures its own dispatcher and CUDA options at attach. Outside hard CPU-only mode, CPU/GPU dynamics are authored per scene.
- A single instance is not thread-safe: serialize calls on one handle. Different instances may step their own attachments at the same time. A recording start/stop needs every attached instance idle and returns an error if one is still stepping.
- The same ovstage instance must not be attached to two ovphysx instances. Use a separate stage per instance.

Multiple `UsdPhysicsScene` prims within the same loaded stage **are** supported. `ovphysx_step()` advances all scenes together; per-scene independent stepping is not currently exposed through the ovphysx C API.

In-process destroy/create cycles **are supported** within one active process
lifecycle: per-instance destruction leaves Carbonite and the direct PhysX
runtime available for the next instance. The hard CPU-only request is the
exception: as soon as setting `OVPHYSX_GLOBAL_CONFIG_CPU_ONLY` to `true`
succeeds, it is sticky and
cannot be reverted in that process. Its CPU-only and ovphysx no-CUDA guarantees
require setting it before the first instance. Outside hard CPU-only mode, CPU/GPU
dynamics remain authored per scene in USD rather than selected by
`ovphysx_create_instance()`. Call `ovphysx_shutdown()` once when the current
process-lifecycle scope is done. With no live handles it clears the
`ovphysx_initialize()` token and drains the direct PhysX runtime. Linux also releases its
Carbonite framework and dependency plugins, preserving earlier host `atexit` handlers and
implicit buffered-file flushing. Windows retains Carbonite's process-exit hook for safe
OVStage/Fabric DLL teardown; Windows hosts must flush buffered files explicitly before exit.
Independently loaded libraries such as OVStage may install their own terminating exit
handlers on Linux too. Explicitly flush host output when using those libraries;
ovphysx shutdown does not remove their handlers.
A later `ovphysx_initialize()` can start another lifecycle on either platform.
If ovphysx reused a framework already populated by a foreign host, that host retains
responsibility for its framework and datastore teardown.

## Task Receipts and Polling

A receipt can be copied and needs no destructor. Each terminal outcome is
collected once: `wait_task(task)` collects only that task, and a repeated direct
wait returns `NOT_FOUND` (`RuntimeError` in Python). Internal synchronization
retires execution resources without collecting outcomes or discarding errors.

`wait_all()` covers tasks accepted when the call begins. It scans uncollected
outcomes in submission order with one readiness deadline. If that deadline
expires, it stops at the earliest unfinished task; earlier completed outcomes
are collected, and later outcomes remain available. Previously collected
failures are not reported again. Waiting on an empty set succeeds.

C and C++ callers use `ovphysx_timeout_t`, a `uint64_t` nanosecond type. Pass
`OVPHYSX_TIMEOUT_POLL` for a readiness poll, a positive value for a finite wait,
or `OVPHYSX_TIMEOUT_INFINITE` for an infinite wait. Python uses `timeout_ns=0`,
a positive integer, or `None`. Ready-at-deadline wins. Finalization may take
longer than a finite readiness timeout. Timeout leaves unfinished tasks retryable.

C waits require a non-null, zero-initialized `ovphysx_wait_task_result_t`.
`error_tasks` contains full failed receipts, and `pending_task` is zero when
nothing is pending. Check the returned `ovphysx_result_t` separately: `TIMEOUT`
takes precedence over collected errors, `ERROR` reports execution or internal
failure, and `SUCCESS` means no collected task failed. Destroy every output with
`ovphysx_destroy_wait_task_result()`, including timeout outputs. Destruction
zeros the result and accepts null or already-cleaned results. Never shallow-copy
an owning wait result or reuse a nonempty one; C++ `WaitResult` is move-only.

Task errors belong to their wait result. `ovphysx_get_last_task_error(instance,
task)` is read-only and returns text retained by a live result; missing causes
return empty text. Borrowed text survives unrelated waits and instance teardown
until its result is destroyed. Fresh lookup requires a live matching instance.
Receipts and retained results do not keep simulation or CUDA resources alive.

Python returns immutable `Task` values from `step`, `clone`, `attach_ovstage`,
`update_from_ovstage` and `detach_ovstage`.
Wait using `physx.wait_task(task, timeout_ns=None)` or
`physx.wait_all(timeout_ns=None)`; both return `None` on success. `TaskError`
contains immutable `(Task, message)` entries in `errors`. `TaskTimeoutError`
contains `errors` collected before the timeout and `pending_task`. Python copies
those values before native cleanup, so exception details remain valid afterwards.

The Python session API returns an `Operation` from `query`, `read`, `write`,
`fetch_next`, `commit`, and `close` / `release`, without waiting. An operation holds
its real native receipts in `tasks` (`task` when there is exactly one; a close with
nothing native to release has none). `operation.wait()` collects them and returns the
typed result, raising `TaskError` for a failure. `operation.result` never waits: it
raises until the outcome has been collected. `wait_all()` also collects registered
operations, and a later `wait()` or `result` returns the cached result or raises the
cached failure again. The typed result is built once, at the first `wait()` or
`result` access, which must happen on the submitting thread; read arrays are ordered
onto the Warp stream current at that access. Inputs and output slots stay alive until
an operation is collected, even when the operation object is dropped, so call
`wait_all()` periodically in fire-and-forget loops to bound retained storage and
observe failures. [The session model](read_write/index.md#the-session-model) shows a
complete query, read, write and commit sequence.

`read_sync`, `read_tokens_sync`, `write_sync`, `commit_sync`, `close_sync`,
`step_sync`, `update_articulations_kinematic` and `evaluate_sdf_shapes` are the
waiting conveniences.
Closing a session or query, leaving its `with` block, and cleanup of an abandoned
object submit releases without waiting; `wait_all()` or instance destruction observes
their failures. A session returned by `read_sync` or `write_sync` waits for the
releases it submits when its `with` block exits. Read storage that arrays still
borrow is released only after the result and every alias are dropped, and a later
`wait_all()` collects that release. CUDA stream/event dependencies still apply.
Context exit preserves an exception from the body and chains cleanup failures.
Deferred finalization never calls native session release from a garbage-collector
thread; the owning thread drains it.

```python
from ovphysx import PhysX, TaskError, TaskTimeoutError

def poll_step(physx: PhysX, dt: float) -> bool:
    task = physx.step(dt)
    try:
        physx.wait_task(task, timeout_ns=0)
    except TaskTimeoutError:
        # The unfinished task is still valid; collect it with an infinite wait.
        physx.wait_task(task)
        return False
    except TaskError as error:
        for failed_task, message in error.errors:
            print(f"Task {failed_task.op_index}: {message}")
        raise
    return True
```

## Threading

- A single instance is **not** thread-safe. Serialize access externally.
- Calls may move between threads when externally serialized; do not wait on the same task concurrently.
- Different instances may step their own attachments at the same time. A recording start/stop needs every attached instance idle.

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
Either wait first (`wait_task()` / `wait_all()` / `step_sync()`), or keep all
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
- In Python, close a binding deterministically with a `with` block or an explicit `binding.destroy()`. New code should exchange simulation state through the session read/write API, whose queries and sessions are context managers (`with physx.read_sync(...) as result:` / `with physx.write_sync(...) as session:`, or the `Query`, `ReadResult` and `WriteSession` returned by the default `query` / `read` / `write` operations); the tensor-binding form `with physx.create_tensor_binding("/World/robot*", tensor_type=TensorType.RIGID_BODY_POSE) as binding:` is **deprecated** but still works during the deprecation window. Contact bindings — `with physx.create_contact_binding(["/World/sensor*"]) as binding:` — are a **deprecated** compatibility surface (C declarations live in `ovphysx_legacy.h`). If garbage collection cleans up a `TensorBinding` or `ContactBinding` first, ovphysx emits `ResourceWarning`. Python suppresses this warning category by default; run with `-W default` or set `PYTHONWARNINGS=default` to show it.
- Create tensor and contact bindings once outside simulation loops and reuse them across steps. Creating a new binding every step allocates new native resources and is slower than reusing the existing binding.
- Tensor and contact bindings are views of the currently realized physics objects. `step()` and `step_sync()` do not invalidate them, but application-owned stage lifecycle changes do. Before `detach_ovstage()`, before removing USD data that contains bound objects, or before replacing/reparsing the stage so bound objects are destroyed and recreated, destroy cached bindings when practical. If a stale binding survives one of those lifecycle operations, only destroy it; do not read or write through it. Create replacement bindings after the operation has completed. Do not keep cached bindings across topology changes and probe them defensively; the application knows when it changed the stage.
- For admission or synchronous API failures, read `ovphysx_get_last_error()` before
  the next call on that thread. Accepted task failures belong to their wait result.
- Native session release transfers ownership only when admitted. A rejected release
  can be retried; an accepted release must never be resubmitted even if its wait
  reports execution failure. The SDK owns remaining cleanup. Finish external buffer
  consumers before releasing their read session.
- A `physxPointer` value is a borrowed address. Do not call `release()` on it. Refer to [PhysX Pointer Interop](#physx-pointer-interop) for lifetime details.
- **PhysX instance lifecycle (Python)**: call `physx.destroy()` explicitly,
  normally from a `finally` block, to guarantee deterministic cleanup of native
  resources. Drop all output-read arrays and downstream views first; otherwise
  `destroy()` raises before teardown and leaves the instance usable for a retry. `destroy()` is checked, so a cleanup failure raised from `finally`
  becomes the active exception while Python preserves any in-flight exception
  as chained context. Applications that need different exception precedence
  must catch and log cleanup failures explicitly. ovphysx 0.6 removes the
  former `release()` spelling and the main-object context-manager protocol. If
  an instance is garbage-collected *during* the run (dropped mid-program rather
  than held to interpreter shutdown), `__del__` emits a `ResourceWarning`
  before native cleanup. The warning is silent by default (filtered
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
- OVStage supplies its own headers, shared library, CMake package, and runtime
  tree, in a package root kept separate from OVPhysX's. Native users either put
  their own on `CMAKE_PREFIX_PATH` or let `find_package(ovphysx)` fetch the
  pinned build.
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
- `pip install ovphysx` resolves the OVStage wheel automatically.
- The Newton USD schema is not a package dependency. Scenes that author
  `newton:*` attributes (read as fallbacks for the PhysX spellings) need
  `pip install newton-usd-schemas`, or the schema downloaded from
  <https://github.com/newton-physics/newton-usd-schemas>, registered with
  ovstage next to the codeless PhysX schemas; refer to
  [Physics Schemas](physics_schemas.md#registering-the-newton-usd-schema).
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
of ovphysx; scenes are attached through ovstage only, so ovphysx itself loads no
USD beyond ovstage's namespaced OpenUSD runtime -- any stock `usd-core` the
application authors with is loaded and owned separately by that application.

## Error Handling

**For C:**
- Check `result.status` on every call. For an accepted task, wait on the whole
  receipt and check that result before consuming any outputs.
- On admission or synchronous API failure, call `ovphysx_get_last_error()` on the same thread to retrieve the error string (valid until the next ovphysx call on that thread).
- For task waits, inspect `error_tasks` even on timeout, retrieve each cause with `ovphysx_get_last_task_error(instance, task)`, then free the output with `ovphysx_destroy_wait_task_result()`. Retain the result for as long as borrowed causes are needed.

**For Python:**
- Rejected submissions and wait validation/internal failures raise ordinary runtime errors. Collected execution failures raise `TaskError`; unfinished waits raise `TaskTimeoutError`, which can also contain partial failures.
- Use try/finally or context managers to ensure bindings are destroyed.
- Pass `raise_if_empty=True` to `create_tensor_binding()` when a zero-count binding should be treated as a configuration error.
- For optional or broad queries, keep the default `raise_if_empty=False` and check `binding.count` before reading.

## Warmup and Determinism

Deprecated tensor-binding reads require a warmup step that initializes PhysX
structures (and DirectGPU buffers in GPU mode). That binding path warms up
automatically on its first tensor operation. Session reads and writes never
auto-warm; complete the first step before reading DirectGPU device state or
committing a DirectGPU write. See
[Step-first precondition](read_write/data_model.md#step-first-precondition-directgpu).
If deterministic initial state matters, call `ovphysx_warmup()` explicitly after
scene attachment and before the first legacy tensor-binding read.

> **Ordering constraint:** all `clone()` calls must happen **before** warmup.
> Calling `clone()` after `warmup()` or after the first `step()` /
> `step_sync()` reallocates physics structures and corrupts already-initialized
> solver state. The C runtime rejects this ordering with
> `OVPHYSX_API_INVALID_ARGUMENT`, surfaced in Python as `RuntimeError`. If you
> need to clone after warmup, call `detach_ovstage()`, wait for it to complete,
> then reload the source scene or reattach its ovstage before cloning again.

GPU dynamics and DirectGPU TensorAPI are separate choices.

**Process-level CPU-only mode** must be set before creating any instance. To keep
ovphysx itself from opening the CUDA driver (for example on a CPU-only deployment):

```python
from ovphysx import GlobalConfigKey, PhysX, get_global_config, set_global_config

def create_cpu_only_physx() -> PhysX:
    set_global_config(GlobalConfigKey.CPU_ONLY, True)
    assert get_global_config(GlobalConfigKey.CPU_ONLY) is True
    return PhysX()
```

`get_global_config(GlobalConfigKey.CPU_ONLY)` /
`ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_CPU_ONLY, ...)` report the
effective hard CPU-only policy (a configured `true` value or
`OVPHYSX_DISABLE_GPU`). They do **not**
report per-scene USD `physxScene:enableGPUDynamics`, a CUDA ordinal, or
attach-time resolved CPU versus GPU dynamics -- those remain separate controls
by design. Successful instance creation also emits an INFO line containing
`process_cpu_only=...` (and, when not hard CPU-only, GPU-selection intent such as
`active_cuda_gpus=no_override` when `ACTIVE_CUDA_GPUS` is unset versus `-1` for explicit
automatic selection). This hard-policy slice does not replace a host that still
needs attach-time confirmation of scene dynamics before dropping a defensive
`set_cpu_mode(True)`.

`OVPHYSX_DISABLE_GPU` is read live before `ovphysx_initialize` and latched when
initialize succeeds, so an observational pre-init `get_cpu_mode()` cannot
permanently miss a later setenv before init.

For deployment environments, setting `OVPHYSX_DISABLE_GPU` before ovphysx
initialization provides the equivalent process-wide CPU-only policy.

Other libraries in the process may still open the CUDA driver. Loading an
ovstage-backed stage currently does, and constructing the `warp.array` values of a
session read or write does when Warp was built with CUDA. Use a
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
    # /physics/suppressReadback is still a Carbonite override on the legacy
    # PhysXConfig / create_args path; there is no scoped GlobalConfigKey for it.
    return PhysX(
        config=PhysXConfig(
            carbonite_overrides={
                "/physics/suppressReadback": True,
            },
        ),
    )
```

Use this for IsaacLab-style tensor workloads that read state or contact tensors
every step. Contact bindings continue to provide contact data with DirectGPU
enabled. Leave DirectGPU off if you need CPU-scene contact events. Contact
modification, such as surface velocity or custom contact modification callbacks,
also requires DirectGPU to be off.

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
   already-complete task receipt, also returned directly by the C++ wrapper.
   Wait on the receipt to collect completion and any execution failure. Cloning is
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
from ovphysx.samples.python_samples.schema_support import physx_schema_paths
import ovstage

def step_remote_scene(scene_url: str) -> None:
    # Register external PhysX and Newton schemas before population.
    ovstage.population.register_usd_schemas(
        [*[str(p) for p in physx_schema_paths()], str(ovphysx.newton_schema_root())]
    )
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
            domains=ovstage.PopulationDomain.ALL,
        )
        # attach_ovstage() reads at a sealed ordinal.
        stage.advance_write_floor(ordinal=1).wait()
        physx.wait_task(physx.attach_ovstage(stage, read_ordinal=1))
        physx.step_sync(1.0 / 60.0)
        physx.wait_task(physx.detach_ovstage())
    finally:
        if physx is not None:
            physx.destroy()
        stage.destroy()
```

### Notes

- OmniClient credentials are **process-global** — configure them **before** ovstage
  population of the remote URL.
- The `domains` mask in the `step_remote_scene` example is `ALL` (equivalently
  `PHYSICS | RENDERING`), the choice for arbitrary content. `PHYSICS` alone, as in
  the shipped samples, is enough when the USD is known not to put physics under
  native scene-graph instances — refer to
  [Population domains](ovstage_integration.md#population-domains).
- OmniClient must be loaded before credentials are configured. Kit-hosted
  Python can import the host-provided `omni.client` module. Standalone callers
  create the ovstage `Stage` first, then bind the client API that ovstage loaded
  before configuring credentials and calling `open_usd()`. Constructing
  `PhysX()` neither loads nor validates OmniClient. `carb.omniclient.plugin` is
  not part of the static ovphysx bootstrap path.

## Scene Queries

Scene queries are a deprecated compatibility surface declared in
`include/ovphysx/ovphysx_legacy.h`. `ovphysx.h` still includes that header so
existing `#include <ovphysx/ovphysx.h>` consumers compile. New code should
include the legacy header explicitly.

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
bindings. Refer to the [C API Reference](api.md) for the scene-query hit and
path-resolver contracts.


## PhysX Pointer Interop

Native PhysX addresses are the read-only `physxPointer` column
(`OVPHYSX_ATTR_PHYSX_POINTER`). Query the matching `ovphysx_query_target_t`
and read that attribute. The column is a host `uint64`, one borrowed address
per matched object. Python reads it through `PhysX.read` as a Warp `uint64`
array. There is no separate pointer lookup.

| Query type | Address |
|---|---|
| `OVPHYSX_QUERY_TARGET_SCENE` | `PxScene` |
| `OVPHYSX_QUERY_TARGET_PHYSX_MATERIAL` | `PxMaterial` |
| `OVPHYSX_QUERY_TARGET_SHAPE` | `PxShape` |
| `OVPHYSX_QUERY_TARGET_COMPOUND_SHAPE` | compound-shape wrapper |
| `OVPHYSX_QUERY_TARGET_RIGID_ACTOR` | `PxRigidDynamic` or `PxRigidStatic` |
| `OVPHYSX_QUERY_TARGET_JOINT` | standalone `PxJoint` |
| `OVPHYSX_QUERY_TARGET_CUSTOM_JOINT` | plugin custom joint |
| `OVPHYSX_QUERY_TARGET_ARTICULATION` | `PxArticulationReducedCoordinate` |
| `OVPHYSX_QUERY_TARGET_ARTICULATION_LINK` | `PxArticulationLink` |
| `OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT` | `PxArticulationJointReducedCoordinate` |
| `OVPHYSX_QUERY_TARGET_PARTICLE_SYSTEM` | particle system |
| `OVPHYSX_QUERY_TARGET_PARTICLE_SET` | particle set |
| `OVPHYSX_QUERY_TARGET_PHYSICS` | process-global `PxPhysics` |

`OVPHYSX_QUERY_TARGET_PHYSICS` is one row. Its path-list entry is `/`, the absolute
root, and that entry is not a USD prim. Prim-set selection
does not change that row. The other new types above publish only
`physxPointer`. `OVPHYSX_QUERY_TARGET_RIGID_BODY` stays dynamics-only and does not
carry actor pointers, including statics.

A missing object is absent from the result. A query type with no
`physxPointer` attribute skips that name the same way it skips any other
unknown attribute. A destroyed object disappears from the next query. A
`uint64` already copied into a live read session is not cleared.

The read is an ordinary read task. Drain pending simulation before using an
address, and use it only after that task completes. Do not call PhysX on the
pointer during an in-flight step.

The query target is the classification. Standalone joints, custom joints, and
articulation joints are `OVPHYSX_QUERY_TARGET_JOINT`,
`OVPHYSX_QUERY_TARGET_CUSTOM_JOINT`, and
`OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT`. A path of another kind is absent
from that query. `ovphysx_physx_type_t` names the kind carried by
object-change callbacks.

Casting requires the PhysX SDK C++ headers shipped under `include/physx/`.
`find_package(ovphysx)` sets `ovphysx_PHYSX_INCLUDE_DIR`. No PhysX library
linking is needed.

Refer to the [PhysX Interop tutorial](tutorials/physx_interop.md) for a
sample that reads `physxPointer` and calls `setKinematicTarget()`.

### Pointer Lifetime

The address names a borrowed PhysX object. Reading it does not grant
ownership, extend lifetime, or permit an SDK call the simulation or device
mode already forbids. Treat the object as invalid after stage detachment,
instance destruction, or that object's own destruction, and
reacquire it after attaching and initializing another stage. A completed
`ovphysx_step()` does not invalidate the object. The numeric column stays
valid until the read session is released, and a later step does not rewrite
it. Do not call `release()` on the address. These rules also apply to the
process-global `PxPhysics` pointer. Objects that callers explicitly create
through `PxPhysics` follow the PhysX SDK's ownership rules.

Cached addresses are invalidated by the instance-scoped object-destroyed and
all-objects-destroyed callbacks. `ovphysx_clone()` does not emit a creation
callback for the clones. Refresh cached addresses after the clone task
completes.

### Thread Safety

PhysX APIs on returned pointers must only be called between simulation
steps — specifically after `wait_task()` completes for the preceding step
and before the next `ovphysx_step()` call. Calling PhysX APIs while a
step is in-flight is a data race.

### Disabling simulation

Do not set or clear `PxActorFlag::eDISABLE_SIMULATION` on a
`PxRigidDynamic*` read from `physxPointer`. That path is
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

- `on_object_created` — fires AFTER the object exists. Safe to read
  `physxPointer` from a deferred handler.
- `on_object_destroyed` — fires BEFORE the object is destroyed. Drop any
  cached pointer before the callback returns; it is unusable after that
  return. Do not call `release()` on it.
- `on_all_objects_destroyed` — fires BEFORE a bulk teardown, for example
  `ovphysx_detach_ovstage()`. Flush the entire pointer cache before returning;
  every cached pointer is unusable after that return, and no per-object
  destruction events follow.

The pathless `PxPhysics` object is not identified by per-path create or destroy
callbacks. Invalidate any cached `PxPhysics` pointer at the explicit detach
and instance-destruction boundaries described above.

Subscriptions are **per instance**: pass the owning `ovphysx_instance_handle_t`. Callbacks fire only while that instance's own attachment is live, and only for that attachment. A peer instance cannot unsubscribe another instance's subscription (`OVPHYSX_API_NOT_FOUND`). Callback payloads still carry `prim_path` (and type), not a stage identifier -- key caches by instance plus path, because two live stages may use the same paths.

`ovphysx_unsubscribe_object_changes()` returns `OVPHYSX_API_SUCCESS` only after that subscription's callbacks have finished, including ones already running on another thread. `user_data` can be freed after that return. Calling unsubscribe from inside any object-change callback returns `OVPHYSX_API_INVALID_STATE` and leaves the subscription active. That check follows the Carbonite task when the callback is running as one, and the native thread otherwise. These notifications stay callbacks. They are not query or read records.

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
    ovphysx_instance_handle_t handle,
    pointer_cache_t* cache,
    ovphysx_subscription_id_t* out_subscription)
{
    ovphysx_object_change_callbacks_t callbacks = {0};
    callbacks.on_object_destroyed = on_destroyed;
    callbacks.on_all_objects_destroyed = on_all_destroyed;
    callbacks.user_data = cache;
    return ovphysx_subscribe_object_changes(handle, &callbacks, out_subscription);
}

static ovphysx_result_t stop_observing_object_changes(
    ovphysx_instance_handle_t handle,
    ovphysx_subscription_id_t subscription)
{
    return ovphysx_unsubscribe_object_changes(handle, subscription);
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
    ovphysx_instance_handle_t handle, PointerCache& cache)
{
    ovphysx::ObjectChangeCallbacks callbacks;
    callbacks.onDestroyed = [&cache](
        std::string_view path, ovphysx_physx_type_t) { cache.drop(path); };
    callbacks.onAllDestroyed = [&cache]() { cache.clear(); };
    return ovphysx::subscribeObjectChanges(handle, std::move(callbacks));
}
```

**Threading:** callbacks may fire from internal worker threads during
`ovphysx_step()`, `ovphysx_step_sync()`, or
`ovphysx_detach_ovstage()`. Do **not** call other ovphysx APIs from inside a
callback -- defer follow-up work until the operation's task wait succeeds, or
until `ovphysx_step_sync()` returns successfully. Unsubscribe from inside any
object-change callback returns `OVPHYSX_API_INVALID_STATE` instead of waiting.
That check follows the Carbonite task when the callback is running as one, and
the native thread otherwise. Another task on the same worker is not inside the
callback. Destroying the C++ handle there closes that subscription without
waiting, skips later callbacks, and frees its state after that callback has
unwound, including when the callback throws.

**Clone and population:** `ovphysx_clone()` does not emit `on_object_created`.
Pointer caches that need to track cloned objects must be refreshed after the
clone task wait succeeds. The initial ovstage attach/update population is also
silent. `ovphysx_detach_ovstage()` emits `on_all_objects_destroyed` and is the
supported path for bulk pointer invalidation.

## Contact Data

Use the session read API for new contact-data consumers: query `CONTACT_SENSOR`
for net forces, raw contact detail, or CPU contact events, or use
`ovphysx_query_contacts` to pair sensors with filters. In Python, use
`PhysX.query` with `SimObjectType.CONTACT_SENSOR` and the `contacts=` argument. Query creation can return an optional discovery summary, and reads can
report omitted attributes in their optional session summary. See
[Contact queries](read_write/index.md#contact-queries) and
[Contact sensors](read_write/readable.md#contact-sensors) for the complete contract.

The compatibility APIs below remain available for existing consumers:

| | Contact Binding |
|---|---|
| **Use when** | You need aggregate force tensors for RL rewards/safety, or per-contact-point geometry with actor identities |
| **Data shape** | `[S, 3]` net forces, `[S, F, 3]` force matrix, or flat per-contact arrays plus a `[C, 2]` actor-ID tensor (all DLPack-compatible) |
| **API style** | Create a binding, then read tensors each step |
| **Device modes** | CPU dynamics and GPU dynamics, including DirectGPU |
| **USD requirement** | Sensor prims must have `PhysxContactReportAPI` applied; child sensors also need reports from a participating actor (see [Child collider sensors](read_write/readable.md#child-collider-sensors)) |
| **Key functions** | `create_contact_binding()`, `read_contact_net_forces()`, `read_contact_force_matrix()`, `read_raw_contact_data()` |

### Contact Binding (Aggregate Force Tensors)

Contact bindings give you aggregate net force vectors between sets of sensor
and filter bodies, delivered as DLPack tensors that work on both CPU and GPU.
Every authored USD prim named by `sensor_patterns` must have
`PhysxContactReportAPI` applied. Runtime-only clones use the schema metadata
copied from the corresponding source object, so an untagged collider cannot
become a sensor just because its parent body has the schema. Filter prims need no
extra schema. Filter patterns are still resolved against physics-registered objects, though (a
collider, rigid body, or other physics registration) — a pattern that only
names an arbitrary non-physics USD prim, for example a plain visual `Mesh` with
no physics schema at all, matches nothing.

On CPU simulation and GPU dynamics with CPU readback, a tagged child collider is eligible as a
sensor but does not enable its owning body's reporting. Also enable reporting on the body (or the
owning articulation link or root) to avoid depending on reports from the other actor in the pair.
If neither actor reports matching contacts, the child remains eligible but reads zero force.
See [Child collider sensors](read_write/readable.md#child-collider-sensors) for the authoring example.

For these modes, enable reporting on the owner but select only the child as a
binding sensor. Selecting the owner as a sensor in this or another live contact
binding on the same attachment gives the owner precedence: contacts are credited
to the owner and the child reads zero. Check wildcard patterns for unintended
owner matches, and establish the bindings before stepping. Session reads support
selecting both the owner and child in these modes.

On DirectGPU, a separate child collider carrying `PhysxContactReportAPI` is
accepted by the deprecated contact binding, but the runtime logs an unsupported
collider warning and the child reads zero force. DirectGPU session queries omit
that child row; select the owning body or articulation link instead.

Refer to the [Contact Binding tutorial](tutorials/contact_binding.md) for a full
walkthrough. Key points:

- Create the binding **before** the first step whose contacts you want to observe.
- Net forces shape: `[S, 3]` — one force vector per matched sensor.
- Force matrix shape: `[S, F, 3]` — per (sensor, filter) pair.
- CUDA output tensors require DirectGPU TensorAPI; `physxScene:enableGPUDynamics=true` by itself
  only selects GPU dynamics. Refer to [Warmup and Determinism](#warmup-and-determinism).
- `dt` for impulse-to-force conversion is taken automatically from the last
  successful `step()` or `step_sync()` call that simulated. A zero-dt step
  simulates nothing: it leaves both the contacts and that `dt` unchanged, so
  reads across it are identical.

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

Applications install `physx-usd-schemas` separately and register both
resource directories before the first USD schema read, including population or
export. The application recipes in the samples locate and validate the required
resources without importing USD; see
[Physics Schemas](physics_schemas.md) for Python and native setup.
Register the separately installed Newton USD schema
(`ovphysx.newton_schema_root()`) in the same call when scenes author `newton:*`
attributes.
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
- Set `ACTIVE_CUDA_GPUS` after create and before attach for CUDA device
  selection. ovphysx applies an explicit selection as part of serialized scene
  attachment; other subsystems must not reconfigure physics GPU selection during
  that attachment. A non-empty value takes precedence over `SCENE_MULTI_GPU_MODE`.
  Unset makes no ovphysx ordinal override (a fresh/default PhysX process
  selects automatically); `"-1"` explicitly requests automatic single-device
  selection. Applications that require a deterministic ordinal pass it
  explicitly before the first GPU scene attaches. An already-created PhysX CUDA
  context manager is not retargeted; use a new process to select a different
  ordinal. The expert DirectGPU setting `/physics/suppressReadback` is
  host-managed (legacy Carbonite override on create-args / `PhysXConfig`) and
  should be configured before instance creation rather than changed while
  physics is active.
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

ovphysx.set_global_config(ovphysx.GlobalConfigKey.LOG_LEVEL, ovphysx.LogLevel.VERBOSE)
print(ovphysx.get_global_config(ovphysx.GlobalConfigKey.LOG_LEVEL))
```

#### C

```c
#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_config.h>

int main(void) {
    /* Applies to the omni_physx_sdk, omni.physx, and ovphysx_internal sources. */
    if (ovphysx_set_global_config(
            OVPHYSX_GLOBAL_CONFIG_LOG_LEVEL,
            ovphysx_config_value_int64(OVPHYSX_LOG_VERBOSE)).status != OVPHYSX_API_SUCCESS)
        return 1;

    ovphysx_config_value_t current = {0};
    if (ovphysx_get_global_config(
            OVPHYSX_GLOBAL_CONFIG_LOG_LEVEL, &current).status != OVPHYSX_API_SUCCESS)
        return 1;
    return current.value.int64_value == OVPHYSX_LOG_VERBOSE ? 0 : 1;
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
#include <ovphysx/ovphysx_config.h>
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
    if (ovphysx_set_global_config(
            OVPHYSX_GLOBAL_CONFIG_LOG_LEVEL,
            ovphysx_config_value_int64(OVPHYSX_LOG_VERBOSE)).status != OVPHYSX_API_SUCCESS)
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
`OVPHYSX_GLOBAL_CONFIG_DEFAULT_LOG_OUTPUT` controls Carbonite's process-global
built-in console logger, so disabling it affects every Carbonite tenant in the
process. Multi-tenant hosts should normally own that policy themselves. A
standalone application can suppress the built-in logger as follows:

#### Python

```python
import ovphysx

ovphysx.enable_python_logging()
ovphysx.set_global_config(ovphysx.GlobalConfigKey.DEFAULT_LOG_OUTPUT, False)
```

#### C

```c
#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_config.h>

int configure_logging(ovphysx_log_callback_t callback) {
    if (ovphysx_set_log_callback(
            OVPHYSX_LOG_VERBOSE, NULL, callback, NULL).status != OVPHYSX_API_SUCCESS)
        return 1;
    if (ovphysx_set_global_config(
            OVPHYSX_GLOBAL_CONFIG_DEFAULT_LOG_OUTPUT,
            ovphysx_config_value_bool(false)).status != OVPHYSX_API_SUCCESS)
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
the callback returns. Reading process-global configuration from that callback
is supported, including the bound profiler. Setting a startup-only
process-global value from it is rejected and does not wait on instance creation. Process initialization and final shutdown publish
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

Enable recording capability with module-level
`set_global_config(GlobalConfigKey.OVD_RECORDING_CAPABLE, True)` before
`PhysX()`. Native callers initialize, then call
`ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE,
ovphysx_config_value_bool(true))` before `ovphysx_create_instance(&handle)`.
The BOOL key defaults to false, is process-wide, and leaves output dormant
until explicit start. Its setter is rejected after the first successful create
until successful shutdown with no live instances, even after all instances are
destroyed. Shutdown while handles remain live does not reopen the setter.
The configured value is retained across shutdown, and the getter reports it.
Old typed and raw OVD startup configuration returns `INVALID_ARGUMENT` with
migration guidance; its numeric slots remain reserved. Python startup fields,
including `PhysXConfig.omnipvd_recording_capable`, are removed.
Python releases its native lifecycle when the last owner is destroyed.
Attach the scene, then use `start_recording()` with
an explicit FILE or TCP `OmniPvdDestination`; start before the first step to
capture the first simulation frame. Creation, attach, and reattach never start
output automatically. Stop before teardown, or start a new
recording on the same capable runtime.

Native callers use `ovphysx_instance_handle_t` and the
`ovphysx_ovd_start_recording`, `ovphysx_ovd_stop_recording`, and
`ovphysx_ovd_is_recording` entry points. The destination layout and ownership
are unchanged. Python retains `start_recording`, `stop_recording`, and
`is_recording`.

The [recording tutorial](tutorials/omnipvd_recording.md) contains complete
examples and the dormant capability cost. Existing startup-output callers
should follow the [migration guide](ovd_api_migration.md).

## Profiling

ovphysx can publish its profile zones to one of two consumers, so you can see what your process was doing. Three groups of zones appear:

| Group | Contents |
|---|---|
| `ovphysx_*` | The ovphysx API calls: `ovphysx_step`, `ovphysx_step_sync`, `ovphysx_wait_task`, `ovphysx_wait_all`, `ovphysx_attach_ovstage`, `ovphysx_clone`, and the tensor binding create / read / write calls |
| Runtime zones | The host-side work between them: USD parsing and stage load, collision cooking, PhysX task dispatch, per-step orchestration |
| PhysX SDK zones | The SDK's own profile zones, nested inside the step zones above, for both CPU and GPU work |

The instrumentation is compiled into release builds and the shipped wheel, so profiling needs no rebuild. No backend is selected by default, and while none is bound a zone costs an atomic load and a branch.

Select one with the environment variable, which needs no change to your code:

```bash
OVPHYSX_PROFILER=trace OVPHYSX_PROFILER_OUTPUT=run.json python my_workload.py
OVPHYSX_PROFILER=nvtx nsys profile -t nvtx,cuda python my_workload.py
```

Or programmatically, which is useful when the process decides at runtime whether it is being profiled:

```python
from ovphysx import GlobalConfigKey, PhysX, ProfilerBackend, set_global_config

set_global_config(GlobalConfigKey.PROFILER_BACKEND, int(ProfilerBackend.TRACE))
set_global_config(GlobalConfigKey.PROFILER_OUTPUT_PATH, "run.json")
physx = PhysX()
```

In C, call `ovphysx_set_global_config(OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND,
ovphysx_config_value_int64(OVPHYSX_PROFILER_BACKEND_TRACE))` before
`ovphysx_create_instance()`.

Both forms resolve to the process-wide `/physics/profilerBackend` Carbonite setting and must be in place **before** the instance is created, because the backend is bound and the PhysX SDK profiler callback installed while the SDK is created. Before a backend is bound, `ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND, &value)` (Python: `get_global_config(GlobalConfigKey.PROFILER_BACKEND)`) reports the environment value when `OVPHYSX_PROFILER` is set, otherwise the setter. After ovphysx binds a backend, the getter reports that bound backend and, for the trace backend, the output path captured at bind. A later environment change does not change the getter until the last instance shuts down. A failed bind reports `NONE` and an unset output path. If another Carbonite profiler already owns the process, the getter also reports `NONE`: the ovphysx selection is not the publisher, and the zones still go to the host profiler.

Because this is a create-time decision, profile a slice of a long run by scoping the collection with Nsight's delay, duration, and capture-range options rather than by toggling the instrumentation.

Like all Carbonite settings, `/physics/profilerBackend` is process-global and sticky: once a process selects a backend, a later instance created with no setter and no environment variable keeps it. Turning it off again is explicit, with `OVPHYSX_PROFILER=none` or `PROFILER_BACKEND` set to `NONE`. The first backend to bind also wins for as long as any instance is alive; a later instance asking for a different one, or for `NONE`, is told at `WARN` which is actually bound and publishes to that one. Once the last instance shuts down the profiler is released and the latch clears, so a process that destroys every instance can select a different backend for the next one.

Notes:

- The trace file is written when the **last** instance in the process shuts down, because that is when the profiler is released. It is plain JSON, not gzip.
- The PhysX SDK zones depend on the profile zones being compiled into the PhysX SDK that ovphysx links, which is the case for the shipped `checked` PhysX libraries. A build against a PhysX `release` package has no zones to forward, and that group stays silent while the others still work.
- The SDK's twelve cross-thread phases (`Basic.simulate`, `Basic.collision`, ...) are not reported at all, under any backend. The Carbonite profiler interface models a zone as a per-thread begin/end pair and cannot express a range closing on another thread, so the runtime discards them rather than corrupt a thread's zone stack. `Basic.simulate` spans the same work as `ovphysx_step` through `ovphysx_wait_task`, which is the closest substitute for the outermost one.
- `ovphysx_step` only enqueues work, so its zone is short and the simulation cost appears under `ovphysx_wait_task`. That is the asynchronous execution model, not a measurement artifact.
- The runtime zones include Carbonite's own task-system zones (`Running fiber`, `Executing task`), which wrap everything else and so dominate a summary sorted by total time. They are also not free to record: several build their names per prim, so a profiled stage attach is measurably slower than an unprofiled one.
- If the surrounding process already bound a Carbonite profiler backend, that one keeps the zones and ovphysx logs which backend bound, at `INFO`.
- The SDK zones reach the Carbonite profiler (`/physics/exposeProfilerData`, on by default) and from there whichever backend is bound, so they follow the same one route as every other zone.

For a runnable sample, memory-traffic attribution, and troubleshooting, refer to [Profiling](tutorials/profiling.md). For recording options and the timeline UI, refer to the [Nsight Systems User Guide](https://docs.nvidia.com/nsight-systems/UserGuide/index.html).

You now have the core integration rules for building, running, and operating ovphysx. For the full C API reference, refer to the [C API Reference](api.md). For Python, refer to the [Python API Reference](python_api.rst).
