<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Device residency and interop

This page covers where read/write data lives (CPU vs GPU), how DLPack tensors are owned, how to
synchronize with CUDA, and how the device-buffer pool and device mode behave.

## Where the data lives (CPU vs GPU)

A single read or write can hand back a mix of CPU and GPU columns, so **never infer a column's device
from its object type or attribute — check each tensor's device and match it.** What to expect:

**On a CPU scene**, every column is on CPU. The only catch: deformables and particles are
**GPU-only**, so a CPU scene has none.

**On a DirectGPU scene**, columns are on GPU, with these exceptions:

- **Not available** — whole vehicles and vehicle wheels (host vehicle simulation is unsupported on DirectGPU), and the contact events (`contactEvent*`:
  PhysX's GPU contact pairs carry no event type, so the read omits them and completes).
- **On CPU even here** — authored inputs the sim never rewrites:
  - rigid body / articulation link: `mass`, `inverseMass`, `inertia`, `inverseInertia`,
    `centerOfMassPosition`, `centerOfMassOrientation`, `disableGravity`, `disableSimulation`,
    `staticFriction`, `dynamicFriction`, `restitution`, `contactOffset`, `restOffset`, `shapeCount`,
    `kinematicTargetPosition`, `kinematicTargetOrientation` (CPU-integrated, so host even on DirectGPU);
  - whole articulation: `staticFriction`, `dynamicFriction`, `restitution`, `contactOffset`,
    `restOffset`, `shapeCount`;
  - joint DOF properties: `jointStiffness`, `jointDamping`, `jointLimit`, `jointMaxVelocity`,
    `jointMaxForce`, `jointArmature`, `jointStaticFriction`, `jointDynamicFriction`,
    `jointViscousFriction`, `jointSpeedEffortGradient`, `jointMaxActuatorVelocity`,
    `jointVelocityDependentResistance`, `jointDriveType`;
  - deformable: `restPoints`, `simElementIndices`, `collisionElementIndices`;
  - deformable materials: `deformableDynamicFriction`, `deformableYoungsModulus`,
    `deformablePoissonsRatio`, `deformableElasticityDamping`, `deformableBendingStiffness`,
    `deformableThickness`, `deformableBendingDamping`.
- **On CPU for sleep-state reads** — `isSleeping` on rigid bodies and articulation links.
  DirectGPU disables sleeping, so this column reports `0` from the scene's rule without
  reading stale host simulation state. Normal step-first and disabled-row omissions apply.
- **On CPU only when writing** — particle `points` / `velocities` (on GPU when read, staged through
  CPU on write).

Every other contact-sensor column is on GPU. The filtered contact detail and the raw contacts
synchronize the host once per read — the host reads their per-sensor counts back to size the arrays —
so request them only in the reads that need them.

## DLPack ownership and lifetime

Read and write groups are **producer-owned** — the caller never allocates a group. Groups are handed
over as borrowed DLPack tensors (`DLTensor`). On the read side there are two lifetimes:

- the **group struct and its `prims.list`** are valid until `ovphysx_release_group` or
  `ovphysx_release_read` is accepted, or the attachment is retired, whichever happens first;
- the **column and its numeric storage** — the `columns[0]` descriptor, the tensors, their data,
  index maps, mask, and the CUDA sync event — are owned by the **read session** and valid until `ovphysx_release_read` is accepted. Fetching further groups and
  an intervening `ovphysx_step` do not invalidate them.

**Release each step.** You re-read every step for fresh values, so release each read once you've
consumed it — its buffers then go back to its query for the next read (see
[Read buffers over a held query](#read-buffers-over-a-held-query)). Holding a read across steps is
legal but only gives a stale snapshot and keeps its buffers from the next read (unless it is captured;
see [Capturing a read in a CUDA graph](#capturing-a-read-in-a-cuda-graph)).

The destination is never an alias of live engine storage: the read gathers each column into a buffer
it owns, which a held query keeps for its next read. Python's `PhysX.read` returns an `Operation`; materialized
group tensors are `warp.array` values on the read's native device. Closing the result ends the read: its
tensors are invalid afterwards, so copy what you keep. A result dropped without a close stays alive
while its tensors are. On the write side a non-empty Python group tensor is a **writable alias of the mapped
memory** (invalid after commit/release), never a copy; an empty group tensor is a Warp-owned empty
array.

Calls on one instance run in submission order. A query, read, fetch, or commit
therefore sees the result of earlier steps. Do not call the same instance
concurrently from another thread. A commit's `write_done_sync` separately waits
for the application to finish filling its buffer. Neither kind of ordering
collects the earlier step's task result. A read snapshot remains valid across
steps until the read result is closed.

Native session calls return task receipts, including fetch and release. Query,
read, and write handles are available on acceptance. Wait before using other
outputs. Keep output slots and tensor data alive until the task finishes;
strings and selections may be released after the call returns.

Stop using a resource once its release is accepted. Retry a rejected release,
but never resubmit an accepted one, even if its task later fails. Finish GPU
consumers before release or teardown. Task completion does not replace the CUDA
stream and event handoffs below.

Detach automatically retires read group metadata and prim lists while the
stage dictionary is alive. Borrowed numeric read snapshots can remain alive across
detach until read release; copy any needed metadata before retirement.
Write sessions instead become invalid at detach because their mappings refer to
the live scene. Finish their GPU consumers before detaching.
Instance destruction ends every native session lifetime;
Python rejects explicit `destroy()` before teardown while a read array or downstream
view that still holds its read session remains alive. The instance and arrays stay usable; drop every alias and retry.
The library's process-exit cleanup is the final lifetime boundary and still tears
down the runtime. Exit handlers registered after `import ovphysx` run before that
cleanup and may read the retained arrays of a result not closed. Handlers registered before import must use
an owned CPU copy made before cleanup instead.

## CUDA synchronization

GPU data is handed over **before the work producing it has necessarily finished**, so you synchronize
through a CUDA event.

**Reading.** Each group carries a CUDA event (in `cuda_sync`) that fires when its data is ready.

- Consuming on the stream the read was given, when that stream is in the scene's CUDA context (for
  `{0, 0}`, the legacy default (null) stream of the producing CUDA context) → already ordered after
  the producer work; nothing to do.
- Consuming on **any other** stream, including the per-thread default stream, a stream of another
  context, or the read's own stream when it belongs to another context → make it wait on the event
  first with
  CUDA's `cuStreamWaitEvent` or `cudaStreamWaitEvent` and check the returned status.
  These enqueue a dependency without blocking the host. Skip the wait when `event == 0`.
- In Python, the first `wait()` or `result` access of a read fetch builds the group's `warp.array`
  values and orders the event onto the Warp stream current at that access, using
  the bundled native runtime to check the CUDA status without blocking the host or
  adding a Python CUDA package. A failed wait raises `RuntimeError` before the arrays
  are exposed. Host groups and zero events skip the wait.
  `PhysX.wait_all()` collects outcomes without building arrays. Order any other stream explicitly.

The event belongs to the read session — don't destroy it.

Pass your stream as `ovphysx_read`'s `cuda_sync` (Python: `cuda_stream=`) and a DirectGPU device
group of rigid bodies, links, whole articulations, joints, tendons, point instancers, deformables,
deformable skin meshes or particle sets is gathered on it, after the work already queued there and
with no host wait. A deformable skin-mesh read cannot be captured while a scene has skin meshes.
For CUDA graph capture requirements, refer to
[Capturing a read in a CUDA graph](#capturing-a-read-in-a-cuda-graph). A stream of another CUDA
context than the scene's is waited for on the host instead, as it is for every other group. The
read's PhysX copies run after the work already queued on your stream, and the next step waits for
those copies, so it also waits for that work.

**Writing.** You are the producer, so tell physics when your data is ready via the `write_done_sync`
you pass to `ovphysx_commit_group`:

- a nonzero event → physics waits for it before reading your buffer;
- a nonzero stream → physics reads your buffer only after the work queued on it;
- both nonzero → both are honored;
- `{0, 0}` → nothing outstanding (a host-resident group is *not* automatically `{0, 0}`).

How the dependency is honored depends on the group:

- **A DirectGPU device group** is committed on the provided `stream` after the `wait_event`, with no
  host synchronization, so the commit can be captured into a CUDA graph (see
  [Capturing a write step in a CUDA graph](#capturing-a-write-step-in-a-cuda-graph)). A `stream` of
  another CUDA context than the scene's is waited for on the host instead; Warp and PyTorch streams
  share the scene's context only when the instance is created with `active_cuda_gpus`. After a rigid
  body is enabled or disabled, a centre of mass changes or a point instancer moves, the next call that
  picks up the change waits on the host for the device commits still queued.
- **Every other group** is applied on the host after a host wait on `stream` and `wait_event`, so it
  cannot be committed on a stream being captured.

A context-relative stream value (`1` for the legacy default stream, `2` for the per-thread default
stream) refers to the CUDA context current on the calling thread, so make the producer's context
current first. Keep the stream and event valid until the commit task completes. A failure to
establish a dependency fails the commit task and spends the group; the SDK retains the group's
storage, and the session release reports that failure. Python's `commit(group, cuda_stream=...,
cuda_wait_event=..., event_owner=...)` keeps `event_owner` alive until the commit is collected.

A host-applied commit waits for `stream` and `wait_event` on the calling thread, and no other call of
the instance runs meanwhile. Producer work must not depend on a CUDA host callback or other code
that calls ovphysx on that instance, or that waits for this commit to finish: that cyclic dependency
can deadlock. The same holds for a DirectGPU device commit still queued when a rigid body is
enabled or disabled, a centre of mass changes or a point instancer moves, since the next call that
picks up the change waits for it on the host. This does not grant concurrent use of the public API.

## The read device-buffer pool

On the DirectGPU read path each output column allocates a device buffer plus a pinned host staging
buffer; these dominate per-read cost. To improve performance, ovphysx keeps released buffers in a per-CUDA-context pool
(best-fit reuse), freed only after the read's completion event. A read first uses the buffers its
query holds (see [Read buffers over a held query](#read-buffers-over-a-held-query)); the pool serves
the rest, and takes back the buffers a query drops.

The maximum memory allocated in this pool is bound by the process-wide
`OVPHYSX_GLOBAL_CONFIG_OVSTAGE_READ_POOL_MAX_MB` key (Python
`GlobalConfigKey.OVSTAGE_READ_POOL_MAX_MB`; Carbonite `/physics/ovstageReadPoolMaxMB`),
**default 256 MiB**. `0` or any negative value **disables** the pool (every column a held query
does not keep is freed and re-allocated).

Note that the budget bounds *retained* bytes per context, not the transient peak of a single
large read. If the pool does not have enough memory for the call, then memory will be allocated,
only performance will be affected and not the result of the call.

## Read buffers over a held query

A read's device columns belong to its query. When a read is released, its columns are kept for the
query's next read instead of being freed. The next read over the same query, with the same
attributes and rows, gathers into the **same device pointers**, after the gathers of the read before
it; a read of fewer attributes or rows reuses them too. To get this, hold the query across steps and
release each read before opening the next. The query keeps one set of columns per scene and kind of
object: a second read over it open at the same time gets its own, and only one of the two sets is kept
when both are released. The kept columns go back to the pool when the query is
released.

The next read writes those columns on the stream it is given, possibly while your own work still
reads them on another, and once the query drops them any read may take them. Before releasing a
read, finish the work that consumes its columns, or queue it on the stream later reads are given.
For the next read over the query with the same attributes and rows, passing it a `wait_event`
recorded after that work is enough.

In Python, physics runs in its own CUDA context unless the instance is created with
`active_cuda_gpus`, so a read without a `cuda_stream` is not ordered against Warp's or PyTorch's
streams. Pass the next read `cuda_wait_event`, recorded on the consumer's stream after it used the
columns: the read waits for it on the device, while passing that stream as `cuda_stream` waits for
it on the host.

## Write buffers over a held query

A write's device columns belong to its query. When a write is released cleanly, its columns are
kept for the query's next write instead of being freed. The next write over the same query, for the
same attribute on the same scene with the same row count, hands out the **same device pointer**.
To get this, hold the query across steps and release each write before opening the next: a query
opened and released every step allocates every time. `ovphysx_update_articulations_kinematic` over a
held query likewise reuses its internal device index list.

A reused DirectGPU column may still be in use by its previous commit. Before filling it, make your
stream wait on the group's `data.cuda_sync.wait_event`. Python does this on Warp's current stream when
it fetches the group; to fill on another stream, first make that stream wait for Warp's, for example
`wp.Stream(device, cuda_stream=handle).wait_stream(wp.get_stream(device))`.

A fill captured in a CUDA graph is valid only for the group it was captured from. Before every replay,
check that the group still matches what the graph captured, and capture again when anything differs:

- every tensor's data pointer, byte offset, shape and dtype;
- the group's prim list, so row *i* still names the same prim.

A matching pointer alone does not prove reuse: a freed column's address can come back for a new
allocation, possibly of a different size. These cases invalidate reuse, so the next write's column
must be checked again even if its pointer looks unchanged:

- The first write over the query.
- A different row count, for example after the selection changed.
- A write after the previous one's release failed, or after its scene changed while it was open: that
  write freed its columns instead of keeping them.
- A write opened while another write over the query is still open. It gets its own column, and when
  both are released only one of the two is kept for the next write; the other is freed, so a graph
  captured on it would replay into freed memory.

The kept columns are freed when the query is released, or when the last write still open on it is
released. Columns kept for a scene that no longer exists, for example after a reset, are freed at the
query's next write. A reused column holds whatever the previous write left in it: fill every row each
time, as with a new column, or commit with a row mask.

## Capturing a write step in a CUDA graph

A DirectGPU write step -- fill, commit (masked or not) and articulation kinematic refresh -- can be
captured into a CUDA graph and replayed every step. The graph is bound to the write session it was
captured from, so keep that session open:

1. Hold the query, open the write and fetch its groups. In C, make the capturing stream wait on a
   reused group's `data.cuda_sync.wait_event` before beginning the capture: a capture cannot wait on
   an event recorded outside it. Python waits at the fetch.
2. On the capturing stream, fill each group and commit it with `{stream, 0}`. After a root or joint
   state write, also call `ovphysx_update_articulations_kinematic` on that stream: a step does not
   refresh link poses for a replayed write. Wait for each task before ending the capture.
3. After any change listed below that invalidates the graph, release the session, open a new one
   and capture again.

Requirements and refusals:

- Capture needs an NVIDIA driver for CUDA 12.5 or newer. On an older driver, a commit on a capturing
  stream is refused at admission (`OVPHYSX_API_INVALID_ARGUMENT` in C, an error in Python), and the
  group stays live; a kinematic refresh on it fails its task, which `ovphysx_wait_task` reports.
  Either way nothing is queued and the capture stays valid.
- Before beginning the capture, wait for the tasks of every instance that uses its CUDA context:
  queued work may use the GPU on an instance's worker thread.
- Between beginning and ending the capture, only read, fill, commit and refresh on the capturing
  stream: do not step or call any other ovphysx operation.
- The capturing stream must belong to the scene's CUDA context. Create the instance on an explicit
  device (`active_cuda_gpus`, for example `PhysX(active_cuda_gpus="0")`) so physics shares the
  primary context with Warp and PyTorch. A commit or refresh on a capturing stream of another
  context fails.
- A host-applied group (host-resident, deformable `points` and `velocities`, particle) is refused on a
  capturing stream (`OVPHYSX_API_INVALID_ARGUMENT` in C, an error in Python) and stays live. A volume
  deformable's `kinematicTarget` is a device group and can be captured.
- Work on the legacy default stream is refused while a blocking stream of its context is being
  captured: stream `1`, `0` for DirectGPU work (Python's default), a stream of another context for
  DirectGPU work, which then runs on the scene's legacy default stream, and any host-applied group
  in device memory. Pass the capturing stream.
- A selected kinematic refresh must run once outside the capture first, to upload its articulation
  list.
- A `wait_event` passed during the capture must be recorded in the same capture.
- Refresh any root or joint state written outside the capture before starting it: a captured
  refresh takes over the refresh those writes wait for, which only a launch of the graph runs.
- Capture fails while the PhysX Visual Debugger is recording.
- A captured commit fails if any of the changes listed below, except a moved point instancer, was made
  after the write was opened.

The graph replays with the rows, GPU indices and buffers resolved when it was captured. These changes
invalidate it, so capture again after any of them:

- prims added or removed, physics APIs applied or removed, any edit that makes physics re-create an
  object, or the scene reset or replaced;
- any rigid body enabled or disabled (including the ones that are not touched by the graph);
- changing the centre of mass of a body/link whose wrench is written by the graph;
- moving a point instancer whose instances the graph writes, or any of its ancestors.

Note that failure to follow this requirement can lead to replays writing wrong values or writing to
the wrong bodies, with no error reported and no apparent failure.

Never:

- replay while a step is running, or step before a replay has finished;
- make one of the changes listed above before a replay has finished;
- replay after the write session is released;
- release the query passed to `ovphysx_update_articulations_kinematic` while the graph may be
  replayed.

## Capturing a read in a CUDA graph

A DirectGPU read of rigid bodies, links, whole articulations, joints, tendons, point instancers,
deformables or particle sets can be captured into a CUDA graph, with the write step or alone. A
deformable skin-mesh read cannot be captured while a scene has skin meshes. The graph is bound to the
read it was captured from, so keep that read open:

1. Hold the query, read it once outside the capture with the same attributes, and release that read:
   its columns and plan stay on the query. In Python, close it; its arrays are invalid afterwards.
2. On the capturing stream, read with `{stream, 0}` and fetch its groups. Wait for each task before
   ending the capture. The groups name no event: work captured after them on the stream is ordered
   after the gathers.
3. Each replay refreshes the same columns from the state current when it runs. Centre-of-mass and
   inverse-dynamics columns are the exception: they may use the link poses of the last step or
   kinematic refresh, and the gravity of the capture. After writing joint positions or root poses
   outside the graph, call `ovphysx_update_articulations_kinematic` outside any capture, on the stream
   the graph is replayed on, before the replay, and capture again after a gravity change.

A `wait_event` passed during the capture must be recorded in the same capture: a wait on an event
recorded outside it invalidates the capture.

A captured read fails before anything is queued, and the capture stays valid, when:

- the following requirement was not met: before the capture begins, call `ovphysx_read` once outside
  it, on the same query with the same attributes, and release that read. That call allocates the
  read's device columns and uploads its row list, which a captured read cannot do, and leaves them on
  the query for the captured read to reuse;
- since that read, an object was created or removed, a rigid body was enabled or disabled, or the
  ACTIVE set changed;
- another read over the same query is still open, in the capture or outside it: a graph captures one
  read per query;
- it holds a host-resident or contact group;
- it reads `DEFORMABLE_SKIN_MESH` and a scene has skin meshes: the read waits on the skinning of each
  step through events recorded outside the capture;
- its stream belongs to another CUDA context, or it would use the legacy default stream while a
  blocking stream of its context is being captured;
- the NVIDIA driver is older than CUDA 12.5, which capture needs.

Capture again after any change listed for a captured write, after a deformable body or particle set
moves, or after a kinematic-target write is opened over a deformable body a captured kinematic-target read
did not list: a replay does not detect
them, and once a read outside the graph has rebuilt the views it gathers through, a replay may read
freed memory. For a point-instancer read, creating or removing any object, a material included,
rebuilds them. An ACTIVE-scope read replays the membership it was captured with.

Never:

- replay while a step is running, or after the read is released;
- make one of the changes listed above before a replay has finished: physics rebuilds or rewrites the
  tables a replay reads without waiting for it;
- destroy the graph before its replays have finished: destroying it can block the host until they end.

Release reads and queries outside any capture: a release may wait on the host and free device memory,
which invalidates a capture. In Python, call `wait_all()` right before beginning a capture, so no
deferred release runs inside it. Releasing a captured read does not wait, unless another read over the
query was released while it was open: it then waits on the host for the last replay. The query's next
read waits for the last replay's gathers, not for the rest of the graph: before a read on another
stream than the replay's, finish the replay or pass a wait event recorded after the launch.

`tests/python_samples/rl_step_capture.py` captures an IsaacLab-style control step this way: a read, a
Warp policy and a joint-target write in one graph, replayed between physics steps.

## Device mode and threading

Hard CPU-only mode is process-wide: set
`OVPHYSX_GLOBAL_CONFIG_CPU_ONLY` to `true` before the first instance, or use the
`OVPHYSX_DISABLE_GPU` environment variable (env overrides the setter). CUDA
selection is per instance: `ovphysx_set_config(..., OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS, ...)`
after create and before attach: one ordinal, or a supported comma-separated ordinal list
that distributes scenes round-robin across GPUs. A single ovphysx instance is not thread-safe — serialize
access. Different instances may be attached at the same time and may step their own attachments
concurrently. Hard CPU-only mode is process-wide; outside that mode, CPU/GPU dynamics are authored
per scene.
