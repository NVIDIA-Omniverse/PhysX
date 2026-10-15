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
    `restOffset`, `shapeCount`, `jacobianShape`;
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
- the **numeric storage** — the tensors, their data, index maps, mask, and the CUDA sync event — is
  owned by the **read session** and valid until `ovphysx_release_read` is accepted. Fetching further groups and
  an intervening `ovphysx_step` do not invalidate it.

**Release each step.** You re-read every step for fresh values, so release each read once you've
consumed it — its buffers then return to the pool for the next read. Holding a read across steps is
legal but only gives a stale snapshot and keeps its buffers out of the pool.

The destination is always **session-owned** memory, not an alias of live engine storage (the read
gathers each column into its own buffer). Python's `PhysX.read` returns an `Operation`; materialized
group tensors are `warp.array` values on the read's native device. On the write side a non-empty Python group tensor is a **writable alias of the mapped
memory** (invalid after commit/release), never a copy; an empty group tensor is a Warp-owned empty
array.

Query construction synchronizes with preceding simulation work before inspecting the
scene. Keep step and session calls externally serialized. If an asynchronous step
starts mid-session, complete it before fetching or reading new scene-dependent data.
The write runtime also orders commit against an in-flight step; its `write_done_sync`
separately orders the application's buffer fill. Neither synchronization collects
the earlier step's task outcome. Retained read snapshots remain valid across steps.

Native session calls return task receipts, including fetch and release. Their
current implementation executes inline, but outputs may be inspected only after a
successful wait. Keep input descriptors, strings, output slots and referenced
storage alive and stable until completion. A completed host task can leave CUDA
work outstanding; keep the event and stream dependencies below.

Release admission ends caller use and transfers remaining cleanup to the SDK.
Rejected releases retain caller ownership and may be retried. An accepted release
that later fails must not be issued again; collect its failure and let the SDK own
any remaining cleanup. Finish external consumers before release or instance
teardown: a producer-ready event does not prove an application's later GPU reads
have completed.

Detach automatically retires read group metadata and prim lists while the
stage dictionary is alive. Borrowed numeric read snapshots can remain alive across
detach until read release; copy any needed metadata before retirement.
Write sessions instead become invalid at detach because their mappings refer to
the live scene. Finish their GPU consumers before detaching.
Instance destruction ends every native session lifetime;
Python rejects explicit `destroy()` before teardown while a read array or downstream
view remains alive. The instance and arrays stay usable; drop every alias and retry.
Normal interpreter exit is the final lifetime boundary and still tears down the runtime.

## CUDA synchronization

GPU data is handed over **before the work producing it has necessarily finished**, so you synchronize
through a CUDA event.

**Reading.** Each group carries a CUDA event (in `cuda_sync`) that fires when its data is ready.

- Consuming on the legacy default (null) stream of the producing CUDA context → already ordered
  after the producer work; nothing to do.
- Consuming on **any other** stream, including the per-thread default stream or a stream of
  another context → make it wait on the event first with
  CUDA's `cuStreamWaitEvent` or `cudaStreamWaitEvent` and check the returned status.
  These enqueue a dependency without blocking the host. Skip the wait when `event == 0`.
- In Python, the first `wait()` or `result` access of a read fetch builds the group's `warp.array`
  values and orders the event onto the Warp stream current at that access, using
  the bundled native runtime to check the CUDA status without blocking the host or
  adding a Python CUDA package. A failed wait raises `RuntimeError` before the arrays
  are exposed. Host groups and zero events skip the wait.
  `PhysX.wait_all()` collects outcomes without building arrays. Order any other stream explicitly.

The event belongs to the read session — don't destroy it.

**Writing.** You are the producer, so tell physics when your data is ready via the `write_done_sync`
you pass to `ovphysx_commit_group`:

- a nonzero event → physics waits for it before reading your buffer;
- a nonzero stream → physics drains that stream before reading your buffer;
- both nonzero → both are honored;
- `{0, 0}` → nothing outstanding (a host-resident group is *not* automatically `{0, 0}`).

The stream and event name your PRODUCER work; they do not select the stream physics consumes on.
A context-relative stream value (`1` for the legacy default stream, `2` for the per-thread default
stream) refers to the CUDA context current on the calling thread, so make the producer's context
current first. Keep the stream and event valid until the commit task completes. The current
implementation may block the committing call while it completes these dependencies; an
`Operation` returned without collection does not imply a nonblocking host call. A failure to
establish a dependency fails the commit task and spends the group; the SDK retains the group's
storage, and the session release reports that failure. Python's `commit(group, cuda_stream=...,
cuda_wait_event=..., event_owner=...)` keeps `event_owner` alive until the commit is collected.

During this wait, the commit remains an active runtime operation. The public ovphysx API rejects
overlapping query, read, write, fetch, commit and release submissions with
`OVPHYSX_API_INVALID_STATE` instead of waiting for the commit. Producer work must not depend on a
CUDA host callback or other code that needs such a call to succeed, or that waits for this commit
to finish: that cyclic dependency can deadlock. This is a limitation of the current blocking
implementation and does not grant concurrent use of the public API.

## The read device-buffer pool

On the DirectGPU read path each output column allocates a device buffer plus a pinned host staging
buffer; these dominate per-read cost. To improve performance, ovphysx keeps released buffers in a per-CUDA-context pool
(best-fit reuse), freed only after the read's completion event.

The maximum memory allocated in this pool is bound by the process-wide
`OVPHYSX_GLOBAL_CONFIG_OVSTAGE_READ_POOL_MAX_MB` key (Python
`GlobalConfigKey.OVSTAGE_READ_POOL_MAX_MB`; Carbonite `/physics/ovstageReadPoolMaxMB`),
**default 256 MiB**. `0` or any negative value **disables** the pool (every column freed and
re-allocated).

Note that the budget bounds *retained* bytes per context, not the transient peak of a single
large read. If the pool does not have enough memory for the call, then memory will be allocated,
only performance will be affected and not the result of the call.

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
