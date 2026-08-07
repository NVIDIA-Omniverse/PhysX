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
- [GPU Warmup and Determinism](#gpu-warmup-and-determinism)
- [Scene Cloning](#scene-cloning)
- [Remote USD Loading](#remote-usd-loading)
- [Scene Queries](#scene-queries)
- [PhysX Pointer Interop](#physx-pointer-interop)
- [Contact Data](#contact-data)
- [Running With Other Carbonite Users](#running-with-other-carbonite-users)
- [Logging](#logging)
- [OmniPVD Recording](#omnipvd-recording)

## Relationship and Consumption Model

ovphysx provides a stable C API and Python bindings for ovstage-driven PhysX simulation.
Use ovphysx when you need USD-based physics simulation outside of Omniverse Kit with a small native SDK.

- Python: `pip install ovphysx` and `from ovphysx import PhysX`
- C/C++: download the OVPhysX SDK and matching native OVStage archive as
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

> **Note:** In the source repository, samples live under `tests/c_samples/` and `tests/python_samples/`. In the SDK package, these are installed to `samples/c_samples/` and `samples/python_samples/` respectively.

**Python Samples** (`tests/python_samples/`):

| Sample | Tutorial | Feature |
|---|---|---|
| `hello_world.py` | [Hello World](tutorials/hello_world.md) | Load USD + step (minimal workflow) |
| `tensor_bindings.py` | [Tensor Bindings](tutorials/tensor_bindings.md) | Read/write simulation data through tensor bindings |
| `contact_binding.py` | [Contact Binding](tutorials/contact_binding.md) | Read contact forces through sensor/filter bindings |
| `tensor_bindings_views.py` | | Build lightweight view wrappers on TensorBindingsAPI (advanced) |
| `omnipvd_recording.py` | [OmniPVD Recording](tutorials/omnipvd_recording.md) | Record physics internals to .ovd files |
| `output_read.py` | [ovstage Integration](ovstage_integration.md) | Closed loop: author control into ovstage, drain it explicitly, step, read every output type back (ADR-0007) |

**Extra Python Samples** (`tests/python_samples_extra/`):

| Sample | Tutorial | Feature |
|---|---|---|
| `visual_rerunio_sample/visualize_rigid_bodies.py` | [Rendering Handoff](tutorials/render_handoff.md) | Visualize rigid body simulation with Rerun |

**C/C++ Samples** (`tests/c_samples/`):

| Sample | Tutorial | Feature |
|---|---|---|
| `hello_world_c/` | [Hello World](tutorials/hello_world.md) | Minimal C hello world |
| `tensor_bindings_c/` | [Tensor Bindings](tutorials/tensor_bindings.md) | CPU tensor read/write |
| `tensor_bindings_gpu_c/` | | GPU tensor read/write with CUDA |
| `contact_binding_c/` | [Contact Binding](tutorials/contact_binding.md) | Contact force reading |
| `omnipvd_recording_cpp/` | [OmniPVD Recording](tutorials/omnipvd_recording.md) | Record physics internals to .ovd files |
| `physx_interop_cpp/` | [PhysX Interop](tutorials/physx_interop.md) | Direct PhysX SDK pointer access |
| `output_read_c/` | [ovstage Integration](ovstage_integration.md) | Closed loop: author control into ovstage, drain it explicitly, step, read every output type back (ADR-0007) |

For SDK setup, refer to [SDK Quickstart](tutorials/quickstart.md).

For tensor binding shape/read/write semantics, refer to [Tensor Bindings](tutorials/tensor_bindings.md).
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
- synchronize before reading or modifying tensors on CPU/GPU if they're currently accessed by asynchronous operations inside ovphysx
- ensure correctness before external side-effects (logging, rendering, network I/O)

## Configuration

The SDK uses a typed config system for known settings, plus a raw Carbonite-setting override for arbitrary paths. Config entries are built
with convenience functions from `ovphysx_config.h` and passed at instance creation or set at runtime.

### C

```c
#include "ovphysx/ovphysx_config.h"

// Before instance creation
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
ovphysx_initialize();
ovphysx_create_instance(&args, &handle);

// At runtime
ovphysx_set_global_config(ovphysx_config_entry_num_threads(8));

// Typed getter
int32_t threads;
ovphysx_get_global_config_int32(OVPHYSX_CONFIG_NUM_THREADS, &threads);

ovphysx_destroy_instance(handle);
ovphysx_shutdown();
```

### Python

```python
from ovphysx import PhysX, PhysXConfig, ConfigBool, ConfigInt32

# At initialization
physx = PhysX(config=PhysXConfig(
    disable_contact_processing=True,
    num_threads=4,
    carbonite_overrides={"/physics/updateToUsd": False},
))

# Typed getter
threads = physx.get_config_int32(ConfigInt32.NUM_THREADS)
enabled = physx.get_config_bool(ConfigBool.DISABLE_CONTACT_PROCESSING)
```

The ``carbonite_overrides`` dict accepts arbitrary Carbonite setting paths for
settings not yet covered by the typed fields. Value types are auto-detected from the
string representation. Using a ``carbonite_overrides`` key that targets a path already
covered by a typed field raises ``ValueError``.

> **Note:** `carbonite_overrides` is write-only. There is no getter for arbitrary Carbonite paths; use the typed getters (`get_config_bool`, `get_config_int32`, etc.) for known settings.

Config is process-global (Carbonite-backed). All instances in the same process share the
same config state.

### Cooked-Collider Cache (UJITSO)

Collision "cooking" converts a mesh into the representation PhysX simulates (convex hull,
triangle mesh, SDF, or convex decomposition). It is CPU-expensive, so ovphysx caches the
cooked result on disk and reuses it on the next run through UJITSO, a local content-addressed
cache. The cache key includes the mesh geometry, the cooking parameters, the cooking type, and
a cooker/PhysX version token, so a changed mesh, changed parameters, or a different PhysX build
automatically re-cooks instead of returning a stale shape.

The cache directory is **application-provided**: ovphysx does not read environment variables
and does not choose a location on your behalf. Set it to enable cross-run persistence:

```python
import ovstage
from ovphysx import PhysX, PhysXConfig

physx = PhysX(config=PhysXConfig(cooked_collider_cache_dir="/path/to/your/cache"))
```

In C, set the `OVPHYSX_CONFIG_COOKED_COLLIDER_CACHE_DIRECTORY` string config key. The first run with
a given cache cooks and stores the result; later runs with an unchanged scene are served from the
cache without re-cooking.

Cooked colliders are **not** reused across runs when:

- No `cooked_collider_cache_dir` is configured: cooking still runs, but nothing is persisted, so it re-cooks every run.
- ovphysx cannot create or write the configured directory: cooked colliders are not persisted across runs (a warning is logged when the directory cannot be created).
- The mesh, the cooking parameters/type, or the PhysX build changed: the key differs, so the shape is re-cooked (this is correct, not a fault).
- Collision cooking has been disabled (it is enabled by default).

ovphysx uses UJITSO as a local, in-process cache. (UJITSO can also operate as a distributed cache;
ovphysx does not use that.) The cache is per-machine, shared only through the directory you
configure, and it grows without automatic eviction -- delete that directory to reclaim space. If
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
returned by `ovphysx_create_instance()`.

## Multi-Instance Support

The SDK exposes a per-instance handle (`ovphysx_handle_t`) so callers can manage lifetime, error queues, and tensor/contact bindings on a per-instance basis. **However, the underlying simulation state is process-global**: only one ovstage-backed USD scene can be attached at a time across all instances in a process. Attaching a stage on one instance detaches whatever stage was previously attached.

Concretely, this means:

- Multiple `ovphysx_handle_t` values can coexist, but they share one simulation backend and one attached stage.
- Config, the hard CPU-only policy, and the attached stage are process-global. Different instances cannot use different config values or stages in the same process; outside hard CPU-only mode, CPU/GPU dynamics are authored per scene.
- For isolated stages or parallel simulations, run each in a separate subprocess.

Multiple `UsdPhysicsScene` prims within the same loaded stage **are** supported. `ovphysx_step()` advances all scenes together; per-scene independent stepping is not currently exposed through the ovphysx C API.

In-process destroy/create cycles **are supported**: the Carbonite framework,
dependency plugins, static omni.physx runtime, and OmniClient remain resident
across destroy/create pairs and until process exit. Hard CPU-only mode is the
exception: as soon as `ovphysx_set_cpu_mode(true)` succeeds, it is sticky and
cannot be reverted in that process. Outside hard
CPU-only mode, CPU/GPU dynamics remain authored per scene in USD rather than
selected by `ovphysx_create_instance()`. Call `ovphysx_shutdown()` once when
the current process-lifecycle scope is done; it clears the
`ovphysx_initialize()` token but does not tear down the resident static runtime.

## Operation Indices and Polling

`op_index` values are single-use. A wait processes unconsumed indices in order
through the requested index: every completed or failed index it reaches is
consumed and must not be used again. A timeout leaves the first still-pending
index and later indices unconsumed. An index completed by internal stream
synchronization may be acknowledged once with `wait_op`; that acknowledgement
consumes it. Polling is supported by passing `timeout_ns = 0` to `wait_op`.

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

lock = threading.Lock()

def sim_thread(physx, dt):
    for _ in range(1000):
        with lock:
            physx.step_sync(dt)  # mutation completes before lock is released

def read_thread(physx, binding, buf):
    for _ in range(1000):
        with lock:
            binding.read(buf)    # no concurrent step() on another thread

t1 = threading.Thread(target=sim_thread, args=(physx, dt))
t2 = threading.Thread(target=read_thread, args=(physx, binding, buf))
t1.start()
t2.start()
```

When using asynchronous `step()`, the same rule applies: do not call
`binding.read()` / `binding.write()` concurrently with an in-flight step.
Either wait first (`wait_op()` / `wait_all()` / `step_sync()`), or keep all
step and tensor calls on one thread. In-stream tensor reads submitted after
`step()` on the same thread are stream-ordered and safe without an explicit
wait; cross-thread use always requires external serialization.

## Tensor Binding Diagnostics

- Pattern bindings can intentionally match zero prims, so expected TensorAPI
  no-match diagnostics are quieted on the simulation view used to create that
  binding.
- Explicit `prim_paths` keep the default error-level no-match diagnostics for
  typo detection.
- To detect partial misses programmatically, compare requested explicit paths
  with Python `binding.prim_paths` or C `ovphysx_tensor_binding_get_prim_paths()`
  after binding creation.

## Ownership and Lifetimes

- Tensor bindings, contact bindings, and attribute bindings own internal resources. They are automatically destroyed when the parent instance is destroyed through `ovphysx_destroy_instance()`. Explicit per-binding destruction (`ovphysx_destroy_tensor_binding`, `ovphysx_destroy_contact_binding`) is available for releasing resources earlier.
- In Python, use `with physx.create_tensor_binding("/World/robot*", tensor_type=TensorType.RIGID_BODY_POSE) as binding:`, `with physx.create_contact_binding(["/World/sensor*"]) as binding:`, or call `binding.destroy()` explicitly. If garbage collection cleans up a `TensorBinding` or `ContactBinding` first, ovphysx emits `ResourceWarning`. Python suppresses this warning category by default; run with `-W default` or set `PYTHONWARNINGS=default` to show it.
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
- **PhysX instance lifecycle (Python)**: use `with PhysX() as physx:` or
  call `physx.release()` explicitly to guarantee deterministic cleanup of
  native resources. If an instance is garbage-collected *during* the run
  (dropped mid-program rather than held to interpreter shutdown), `__del__`
  emits a `ResourceWarning` before calling `release()` itself. The warning is
  silent by default (filtered out unless `python -W default::ResourceWarning`
  or a test suite captures it) and mirrors Python file-object semantics. The
  Python API does not register a process-exit cleanup hook, so applications
  must not rely on interpreter shutdown to release live instances.

## Dependency Management

The native distribution consists of two separate package roots: the OVPhysX SDK
and the matching native OVStage archive. The Python distribution instead uses
the ovphysx wheel and its exact `ovstage` wheel dependency:

- OVPhysX runtime dependencies are loaded from its packaged plugins directory
  (`_install/plugins/` in a source install, `ovphysx/plugins/` in the wheel).
- OVStage supplies its own headers, shared library, CMake package, and runtime
  tree. Native users download its archive separately and keep the package roots
  separate.
- The ovphysx wheel includes the physics runtime plus the OVStage-provided
  OmniClient and connection libraries needed for PhysX-first startup.
- The exact `ovstage` wheel supplies OVStage, the resolver and its registry, and
  the namespaced USD runtime; the ovphysx wheel omits duplicate resolver/USD
  singleton binaries.
- Python exposes only TensorBindingsAPI; the legacy `ovphysx.tensors` compatibility layer
  is no longer shipped.
- `pip install ovphysx` resolves the OVStage wheel automatically. Native SDK
  users manually download both archives.
- Auto-detects library location through `getLibraryDirectory()`
- Pre-loads shared libraries with `RTLD_GLOBAL` for plugin symbol resolution
- Offline-capable once both native packages or both declared wheels are installed

For source builds, namespaced monolithic USD is the only supported layout. The
classic USD build switch is not present.

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

## GPU Warmup and Determinism

Reads from GPU-backed DirectGPU bindings require a warmup step that initializes DirectGPU buffers.
This is done automatically on the first tensor operation.
If deterministic initial state matters, call `ovphysx_warmup_gpu()` explicitly after USD load and before the first tensor read.

> **Ordering constraint:** all `clone()` calls must happen **before** GPU
> warmup. Calling `clone()` after `warmup_gpu()` or after the first `step()` /
> `step_sync()` / `step_n_sync()` would reallocate the DirectGPU buffers and silently corrupt
> already-initialized solver state, so the C runtime rejects this ordering with
> `OVPHYSX_API_INVALID_ARGUMENT`, surfaced in Python as `RuntimeError`. If you
> need to clone after warmup, call `reset_stage()` first to clear the warmup state.

GPU dynamics and DirectGPU TensorAPI are separate choices.

**Process-level CPU-only mode** must be set before creating any instance.
To prevent any CUDA driver contact (for example on a CPU-only deployment):

```python
from ovphysx import PhysX
PhysX.set_cpu_mode(True)   # before the first PhysX() for the no-CUDA-touch guarantee
physx = PhysX()
```

This forces all PhysX scenes in the process to use CPU dynamics, overriding
any `physxScene:enableGPUDynamics=true` in the USD stage. The mode is sticky
for the process lifetime as soon as enabling it succeeds. For per-scene CPU
control (mixed CPU/GPU process without this flag), author each scene explicitly:
`physxScene:enableGPUDynamics=false` and `physxScene:broadphaseType="MBP"`.

**GPU dynamics** (whether the PhysX simulation pipeline runs on GPU) is
otherwise controlled by authoring `physxScene:enableGPUDynamics=true` in the
USD stage -- ovphysx does not read or write this setting itself.

High-throughput CUDA TensorBinding and ContactBinding views additionally require
opting into DirectGPU before instance creation:

```python
from ovphysx import PhysX, PhysXConfig

physx = PhysX(
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

## Scene Cloning

Two paths duplicate a scene subtree for large-scale parallel simulation:

1. **Direct ovphysx API.** After an ovstage-backed scene has been attached and
   drained, callers may use `ovphysx_clone()` (C), `PhysX::clone()` (C++), or
   `PhysX.clone()` (Python). Pass a source path,
   a list of target paths, and an optional flat array of per-target parent
   transforms `(px, py, pz, qx, qy, qz, qw)` (position + imaginary-first
   quaternion; `NULL` co-locates every clone on the source). Each transform
   positions a copy's parent; every cloned body keeps its pose relative to the
   source's parent, so an at-origin source lands each body exactly at its
   transform. Replication executes inline. C and Python return an already-complete
   op index (`ovphysx_wait_op` remains valid and returns immediately), while the
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
   same-environment objects cloned by different calls would never collide. Refer to
   [`tests/python_samples/clone.py`](../tests/python_samples/clone.py) and
   [`tests/c_samples/clone_c/main.c`](../tests/c_samples/clone_c/main.c).

2. **Build the scene through ovstage.** Apps that own the ovstage `Stage` can
   duplicate the source subtree with `ovstage.Stage.clone()` (Python) or
   `ovstage_clone()` (C), then drain the clone delta with
   `update_from_ovstage()`. This creates the corresponding physics objects, but
   in ovphysx 0.5 TensorBindingsAPI does not discover objects created by an
   ovstage clone delta. Use this path only when tensor bindings do not need to
   include the clones. Complete TensorBindingsAPI support is scheduled for
   ovstage after 0.5.

For tensor-based multi-environment workloads in 0.5, populate and attach the
source Stage, drain any later committed source edits, and use the direct
`clone()` API. Complete direct cloning before `warmup_gpu()` or the first
simulation step. Direct cloning invalidates existing tensor and contact
bindings; destroy and recreate them before use. Do not also drain a duplicate
ovstage clone delta for the same target paths; the direct clones are runtime-only
physics copies and are the copies TensorBindingsAPI should target in this
workflow.

Neither cloning nor `step()` implicitly drains later ovstage edits. Applications
must seal those edit ordinals and call `ovphysx_update_from_ovstage()` before
cloning or stepping.


## Remote USD Loading

ovstage population accepts any URI that the Omniverse Client Library supports:

- **Local paths:** `/path/to/scene.usd` (as before)
- **Omniverse Nucleus:** `omniverse://server/path/scene.usd`
- **S3 (HTTPS):** `https://my-bucket.s3.us-east-1.amazonaws.com/path/scene.usd`
- **Azure Blob:** `https://account.blob.core.windows.net/container/scene.usd`

The client library is loaded automatically when an ovphysx instance is created. No additional setup is needed for Nucleus URIs when the server allows anonymous access.

> **Note:** Use HTTPS S3 URLs (virtual-hosted style), not `s3://` URIs. The OmniClient library resolves S3 assets through HTTPS and adds AWS authentication automatically when credentials are configured.

### Configuring credentials

Remote-asset credentials live on the **Omniverse Client Library (OmniClient)** —
the asset resolver ovstage population uses to fetch S3 / Azure assets — not on
ovphysx. The application owns population, so it configures credentials directly on
OmniClient before populating ovstage from a private URL. ovphysx does not wrap
this (it only consumes an already-populated Stage). The examples below use
placeholder strings; do not commit real credentials in source code.

- **Python:** use the `omni.client` S3 / Azure configuration API
  (for example `omni.client.set_s3_configuration(...)` / `set_azure_sas_token(...)`).
- **C / C++:** call OmniClient directly — `omniClientSetS3Configuration2(...)` /
  `omniClientSetAzureSASToken(...)`.

```python
import ovphysx
import ovstage
# Configure credentials on the asset layer your app uses, BEFORE population, e.g.:
#   import omni.client
# Configure the access key, secret, and region with omni.client's S3 API.

physx = ovphysx.PhysX()

stage = ovstage.Stage("remote-scene")
ovstage.population.open_usd(
    stage,
    "https://my-bucket.s3.us-east-1.amazonaws.com/scenes/robot.usd",
    ordinal=1,
    domains=ovstage.PopulationDomain.PHYSICS,
)
physx.attach_ovstage(stage, read_ordinal=1)
```

### Notes

- OmniClient credentials are **process-global** — configure them **before** ovstage
  population of the remote URL.
- The `domains` mask above is `PHYSICS`, which matches the shipped samples when
  the USD is known not to put physics under native scene-graph instances. For
  arbitrary content prefer `ALL` (equivalently `PHYSICS | RENDERING`) — refer to
  [Population domains](ovstage_integration.md#population-domains).
- OmniClient must be loaded in the process. Creating an ovphysx instance loads it
  from the exact matched OVStage runtime and pins it for process lifetime. A
  client already loaded by OVStage is reused only when its version matches;
  ovphysx fails closed if an incompatible foreign client is already present.
  An app that populates ovstage before/without ovphysx relies on OVStage's USD
  runtime to bring up the resolver. `carb.omniclient.plugin` is not part of the
  static ovphysx bootstrap path.

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

# Raycast downward
hits = physx.raycast(
    origin=[0, 10, 0],
    direction=[0, -1, 0],
    distance=100.0,
    mode=SceneQueryMode.CLOSEST,
)
if hits:
    print(f"Hit at distance {hits[0]['distance']}")

# Sweep a sphere
hits = physx.sweep(
    geometry_type=SceneQueryGeometryType.SPHERE,
    direction=[1, 0, 0],
    distance=50.0,
    radius=0.5,
    position=[0, 1, 0],
)

# Overlap test
hits = physx.overlap(
    geometry_type=SceneQueryGeometryType.BOX,
    half_extent=[1, 1, 1],
    position=[0, 0, 0],
    rotation=[0, 0, 0, 1],
)
```

### Path Encoding

Hit results contain `collision`, `rigid_body`, and `material` fields as uint64-encoded
SdfPaths matching ovphysx's internal path encoding. Consumers that need to compare
hit paths against known prims should use the same encoding.


## PhysX Pointer Interop

For advanced use cases that go beyond the TensorBindingsAPI -- such as custom joint
manipulation or direct body property access -- ovphysx can return raw PhysX SDK object
pointers by USD prim path.

```c
void* ptr = NULL;
ovphysx_result_t r = ovphysx_get_physx_ptr(
    handle, OVPHYSX_LITERAL("/World/physicsScene"), OVPHYSX_PHYSX_TYPE_SCENE, &ptr);
// Cast: physx::PxScene* scene = static_cast<physx::PxScene*>(ptr);
```

The `ovphysx_physx_type_t` enum specifies which PhysX type to look up
(scene, actor, articulation link, joint, shape, material, etc.).
The C API returns `void*`; the caller casts to the appropriate PhysX SDK type.

Casting requires the PhysX SDK C++ headers (for example `PxScene.h`,
`PxRigidDynamic.h`). The ovphysx SDK ships these headers under
`include/physx/`; `find_package(ovphysx)` sets `ovphysx_PHYSX_INCLUDE_DIR`
to point there. No PhysX library linking is needed.

Refer to the [PhysX Interop tutorial](tutorials/physx_interop.md) for a
complete sample that uses `setKinematicTarget()` on a `PxRigidDynamic*`.

The C++ experimental API provides a type-safe overload that deduces the
enum from the pointer type, preventing mismatches at compile time:

```cpp
physx::PxScene* scene = nullptr;
physx.getPhysXPtr("/World/physicsScene", scene);  // enum auto-deduced
```

PhysX pointer interop is exposed only through the C and C++ APIs. The Python
binding does not expose raw PhysX pointers; use the
[TensorBindingsAPI](tutorials/tensor_bindings.md) for Python workflows instead.

### Pointer Lifetime

Returned pointers are valid from acquisition until
`ovphysx_reset_stage()` or instance destruction. Calls to `ovphysx_step()` do
**not** invalidate existing pointers. Do not call
`release()` on returned pointers — ovphysx owns them.

### Thread Safety

PhysX APIs on returned pointers must only be called between simulation
steps — specifically after `wait_op()` completes for the preceding step
and before the next `ovphysx_step()` call. Calling PhysX APIs while a
step is in-flight is a data race.

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
- `on_all_objects_destroyed` — fires BEFORE a bulk teardown (e.g.
  `ovphysx_reset_stage()`). Flush the entire pointer cache; no per-object
  destruction events follow.

Subscriptions are **process-global**: a single subscription receives events
from every ovphysx instance in the process. There is no per-instance filter,
and callback payloads do not include a stage identifier — only `prim_path`.
Today ovphysx is single-attached-stage-per-process, so treat these callbacks
as one-stage-at-a-time and do not rely on them for cross-stage bookkeeping.

```c
ovphysx_object_change_callbacks_t cbs = {0};
cbs.on_object_destroyed     = my_on_destroyed;
cbs.on_all_objects_destroyed = my_on_all_destroyed;
cbs.user_data               = my_cache;

ovphysx_subscription_id_t sub = OVPHYSX_INVALID_SUBSCRIPTION_ID;
ovphysx_subscribe_object_changes(&cbs, &sub);
ovphysx_unsubscribe_object_changes(sub);
```

```cpp
auto sub = ovphysx::subscribeObjectChanges(ovphysx::ObjectChangeCallbacks{
    .onDestroyed   = [&cache](std::string_view path, ovphysx_physx_type_t) { cache.drop(path); },
    .onAllDestroyed = [&cache]() { cache.clear(); },
});
// sub destructor unsubscribes automatically.
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
| **Use when** | You need tensorized aggregate forces or flat detailed/raw contact data for fixed sensor bodies | You need variable-length event headers and contact-point records |
| **Data shape** | `[S, 3]` net forces, `[S, F, 3]` force matrices, or flat `[C, ...]` buffers indexed by count/start tensors (DLPack, GPU-compatible) | Variable-length arrays of event headers + contact points (raw buffers) |
| **API style** | Create a binding, then read tensors each step | Call once per step, receive pointers to internal buffers |
| **USD requirement** | Sensor prims must have `PhysxContactReportAPI` applied | Prims must have `PhysxContactReportAPI` applied |
| **Key functions** | `create_contact_binding()`, `read_contact_net_forces()`, `read_contact_force_matrix()`, `read_contact_data()`, `read_friction_data()`, `read_raw_contact_data()` | `get_contact_report()` |

### Contact Binding (Aggregate and Detailed Tensors)

Contact bindings deliver aggregate forces and flat detailed contact, friction,
and unfiltered raw-contact data as DLPack tensors that work on both CPU and
GPU. Raw reads also return other-actor IDs that the binding can resolve to
physics-object paths. Every authored USD prim named by `sensor_patterns` must
have `PhysxContactReportAPI` applied; filter prims need no extra schema, and
runtime-only clones inherit contact reporting from the source actor.

Refer to the [Contact Binding tutorial](tutorials/contact_binding.md) for a full
walkthrough. Key points:

- Create the binding **before** the first step whose contacts you want to observe.
- Net forces shape: `[S, 3]` — one force vector per matched sensor.
- Force matrix shape: `[S, F, 3]` — per (sensor, filter) pair.
- Detailed contact and friction reads use flat `[C, ...]` payload buffers
  indexed by `[S, F]` count/start tensors.
- Unfiltered raw-contact reads use flat `[C, ...]` payload buffers indexed by
  `[S]` count/start tensors.
- CUDA output tensors require DirectGPU TensorAPI; `physxScene:enableGPUDynamics=true` by itself
  only selects GPU dynamics. Refer to [GPU Warmup and Determinism](#gpu-warmup-and-determinism).
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
const ovphysx_contact_event_header_t* headers = NULL;
const ovphysx_contact_point_t* data = NULL;
uint32_t num_headers = 0, num_data = 0;

// Basic contact report (headers + contact points)
ovphysx_get_contact_report(handle, &headers, &num_headers, &data, &num_data,
                           NULL, NULL);
// Access fields directly: headers[0].actor0, data[0].position[1], etc.

// With friction anchors
const ovphysx_friction_anchor_t* anchors = NULL;
uint32_t num_anchors = 0;
ovphysx_get_contact_report(handle, &headers, &num_headers, &data, &num_data,
                           &anchors, &num_anchors);
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
report = physx.get_contact_report()
for i in range(report["num_headers"]):
    h = report["headers"][i]
    print(f"pair {i}: actor0={h.actor0:#x}, {h.numContactData} points")
for j in range(report["num_points"]):
    p = report["points"][j]
    print(f"  pos=({p.position[0]:.3f}, {p.position[1]:.3f}, {p.position[2]:.3f})")
# Must finish access before the next step — buffers will be reused.
```

In Python (safe across steps, `copy=True`):

```python
report = physx.get_contact_report(copy=True)
physx.step_sync(dt)  # buffers are reused, but report is owned
for h in report["headers"]:
    print(f"actor0={h['actor0']:#x}, {h['numContactData']} points")
for p in report["points"]:
    pos = p["position"]
    print(f"  pos=({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
```

## Running With Other Carbonite Users

ovphysx can share a process with other OV libraries that use Carbonite and the
same namespaced USD runtime. For example, if another library has already loaded
the namespaced USD monolith, ovphysx reuses that loaded USD library instead of
loading a second copy.

ovphysx does not reuse a PhysX runtime loaded by another Carbonite user.
`IPhysxSimulation` is an internal direct runtime table, not a Carbonite-acquired
interface. ovphysx still uses Carbonite to load plugin binaries and dependencies,
then starts its statically linked `OvruntimePhysX` runtime directly. A foreign Carbonite
framework is allowed by default, but `OVPHYSX_COEXIST_REFUSE=1` makes ovphysx fail
fast, and foundation-interface mismatches still fail with clear probe errors.
Compatible OV libraries can share Carbonite and namespaced USD without sharing the
PhysX runtime.

When another USD-aware subsystem, such as ovrtx, shares the same namespaced USD
runtime, publish every subsystem's schema/plugin paths before either subsystem
opens a USD stage or otherwise touches the USD schema registry:

```c
#include <stddef.h>
#include <ovphysx/ovphysx.h>
#include <ovrtx/ovrtx.h>

int main(void)
{
    ovphysx_register_schema_paths();
    ovrtx_register_schema_paths();

    ovphysx_initialize();
    ovphysx_create_args physx_args = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t physx = OVPHYSX_INVALID_HANDLE;
    ovphysx_create_instance(&physx_args, &physx);

    ovrtx_config_t rtx_config = { 0 };
    ovrtx_renderer_t* renderer = NULL;
    ovrtx_create_renderer(&rtx_config, &renderer);
    ovrtx_destroy_renderer(renderer);
    ovphysx_destroy_instance(physx);
    ovphysx_shutdown();
    return 0;
}
```

For ovphysx-only applications, this explicit call is optional:
`ovphysx_create_instance()` registers the same ovphysx schema path automatically.
The explicit call is for multi-subsystem processes where later initialization
order should not decide which USD schemas are visible.

Important process rules:

- Sharing an already-loaded namespaced USD library is supported.
- Call each subsystem's schema-path registration function before the first USD
  stage or schema-registry access. Late calls cannot repair an already-populated
  schema registry.
- Sharing a pre-loaded Carbonite PhysX runtime stack is not supported.
- Kit or Isaac Sim processes that already loaded their physics extensions are
  not supported ovphysx embedding targets.
- Device settings are process-global. If `/physics/cudaDevice` or
  `/physics/suppressReadback` is already set to a conflicting value,
  `ovphysx_create_instance()` fails instead of silently using the wrong mode.
- If another Carbonite user already set the app directory, ovphysx preserves it.

Schema-path registration governs coexistence in the *process*. If the subsystems
also share one ovstage Stage, that Stage must be populated with `ALL` (or
`PHYSICS | RENDERING`) — refer to
[Population domains](ovstage_integration.md#population-domains).

## Logging

ovphysx uses [Carbonite](https://docs.omniverse.nvidia.com/kit/docs/carbonite/) as its internal logging backend.
By default, the global log level is `LogLevel.WARNING` — only warnings and errors are emitted.

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

// Set before or after instance creation — applies globally to all outputs.
ovphysx_set_log_level(OVPHYSX_LOG_VERBOSE);

uint32_t current = ovphysx_get_log_level();
```

### Custom Log Callbacks (C)

Register one or more callbacks to receive log messages programmatically.
The caller must ensure the callback and any resources it references remain
valid until it is unregistered. If the callback and its resources naturally
outlive the process (for example a static function with no `user_data`), calling
`ovphysx_unregister_log_callback()` is not required. When called, it
guarantees the callback is not running on any thread and will never be
invoked again.

```c
void my_logger(uint32_t level, const char* message, void* user_data) {
    fprintf(stderr, "[%u] %s\n", level, message);
}

ovphysx_register_log_callback(my_logger, NULL);
// Run simulation here.
ovphysx_unregister_log_callback(my_logger, NULL);
// Safe to destroy any resources referenced by the callback here.
```

### Controlling Default Console Output

By default, Carbonite logs to the console. When a custom callback is registered
that also writes to the console, output may be doubled. Use
`ovphysx_enable_default_log_output()` to suppress the built-in console logger:

#### Python

```python
ovphysx.enable_python_logging()
ovphysx.enable_default_log_output(False)
```

#### C

```c
ovphysx_register_log_callback(my_logger, NULL);
ovphysx_enable_default_log_output(false);  // only my_logger receives messages now
```

### Python Logging Bridge

Route native log messages into Python's standard `logging` module:

```python
import logging
import ovphysx

# Route native messages to the "ovphysx" Python logger
ovphysx.enable_python_logging()

# Add a handler to see the output
logging.getLogger("ovphysx").addHandler(logging.StreamHandler())
logging.getLogger("ovphysx").setLevel(logging.DEBUG)

# Run simulation here; native messages appear in Python logging.

ovphysx.disable_python_logging()
```

## OmniPVD Recording

ovphysx supports recording PhysX simulation internals to `.ovd` files through OmniPVD. The resulting files can be opened in any Kit application with the OmniPVD extension for frame-by-frame inspection of shapes, contacts, and solver state.

Recording is controlled by two typed config fields, both set at instance creation:

| Python field | C builder | Description |
|---|---|---|
| `omnipvd_ovd_recording_directory` | `ovphysx_config_entry_omnipvd_ovd_recording_directory()` | Directory for `.ovd` output |
| `omnipvd_output_enabled` | `ovphysx_config_entry_omnipvd_output_enabled()` | Enable recording |

Both must be set **before** the PhysX instance is created. The recording directory is auto-created if it does not exist.

```python
from ovphysx import PhysX, PhysXConfig

physx = PhysX(
    config=PhysXConfig(
        omnipvd_ovd_recording_directory="/tmp/pvd_output",
        omnipvd_output_enabled=True,
    )
)

stage = ovstage.Stage("recorded-scene")
ovstage.population.open_usd(
    stage, "scene.usda", ordinal=1, domains=ovstage.PopulationDomain.PHYSICS
)
physx.attach_ovstage(stage, read_ordinal=1)
for i in range(100):
    physx.step_sync(1/60)

physx.detach_ovstage()
physx.release()  # finalizes recording → <timestamp>_rec.ovd
stage.destroy()
```

No OmniPVD-specific dependencies are needed beyond the matching OVPhysX and
OVStage packages; the writer is built into the packaged PhysX runtime.

For the full tutorial with C examples and Kit inspection instructions, refer to [OmniPVD Recording](tutorials/omnipvd_recording.md).

You now have the core integration rules for building, running, and operating ovphysx. For the full C API reference, refer to the [C API Reference](api.md). For Python, refer to the [Python API Reference](python_api.rst).
