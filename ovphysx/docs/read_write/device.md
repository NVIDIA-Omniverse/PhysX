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

- **Not available** — vehicle wheels (vehicles are CPU-only).
- **On CPU even here** — authored inputs the sim never rewrites:
  - rigid body / articulation link: `mass`, `inverseMass`, `inertia`, `inverseInertia`,
    `centerOfMassPosition`, `centerOfMassOrientation`, `disableGravity`, `disableSimulation`,
    `staticFriction`, `dynamicFriction`, `restitution`, `contactOffset`, `restOffset`, `shapeCount`;
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
- **On CPU only when writing** — particle `points` / `velocities` (on GPU when read, staged through
  CPU on write).

## DLPack ownership and lifetime

Read and write groups are **producer-owned** — the caller never allocates a group. Groups are handed
over as borrowed DLPack tensors (`DLTensor`). On the read side there are two lifetimes:

- the **group struct and its `prims.list`** are valid until `ovphysx_release_group` or
  `ovphysx_release_read`;
- the **numeric storage** — the tensors, their data, index maps, mask, and the CUDA sync event — is
  owned by the **read session** and valid until `ovphysx_release_read`. Fetching further groups and
  an intervening `ovphysx_step` do not invalidate it.

**Release each step.** You re-read every step for fresh values, so release each read once you've
consumed it — its buffers then return to the pool for the next read. Holding a read across steps is
legal but only gives a stale snapshot and keeps its buffers out of the pool.

The destination is always **session-owned** memory, not an alias of live engine storage (the read
gathers each column into its own buffer). In Python a read returns a `warp.array` on the read's
native device. On the write side a non-empty Python group tensor is a **writable alias of the mapped
memory** (invalid after commit/release), never a copy; an empty group tensor is a Warp-owned empty
array.

The synchronous calls (`ovphysx_read` / `ovphysx_write` and their fetch/release) drain pending
simulation first. Any buffer passed to an **async** op (e.g. `ovphysx_step`) must stay valid and
unmodified until `ovphysx_wait_op` reports completion.

## CUDA synchronization

GPU data is handed over **before the work producing it has necessarily finished**, so you synchronize
through a CUDA event.

**Reading.** Each group carries a CUDA event (in `cuda_sync`) that fires when its data is ready.

- Consuming on the default (null) stream → already correct; nothing to do.
- Consuming on **your own** stream → make it wait on the event first with
  `ovphysx_cuda_stream_wait_event(stream, event)` (async; `event == 0` is a no-op).

The event belongs to the read session — don't destroy it.

**Writing.** You are the producer, so tell physics when your data is ready via the `write_done_sync`
you pass to `ovphysx_commit_group`:

- a CUDA event → physics waits on it before reading your buffer;
- a stream but no event → physics drains that stream;
- `{0, 0}` → nothing outstanding (a host-resident group is *not* automatically `{0, 0}`).

## The read device-buffer pool

On the DirectGPU read path each output column allocates a device buffer plus a pinned host staging
buffer; these dominate per-read cost. To improve performance, ovphysx keeps released buffers in a per-CUDA-context pool
(best-fit reuse), freed only after the read's completion event.

The maximum memory allocated in this pool is bound by the process-wide config
`OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB` (builder
`ovphysx_config_entry_ovstage_read_pool_max_mb`; Carbonite `/physics/ovstageReadPoolMaxMB`),
**default 256 MiB**. `0` or any negative value **disables** the pool (every column freed and
re-allocated).

Note that the budget bounds *retained* bytes per context, not the transient peak of a single
large read. If the pool does not have enough memory for the call, then memory will be allocated,
only performance will be affected and not the result of the call.

## Device mode and threading

Hard CPU-only mode is process-wide: `ovphysx_set_cpu_mode(true)` before the first instance, or the
`OVPHYSX_DISABLE_GPU` environment variable. The active CUDA ordinal is a single value on
`ovphysx_create_args`, applied at attach. A single ovphysx instance is not thread-safe — serialize
access; multiple instances across threads are fine, and all instances in a process share the same
(CPU or GPU) device mode.
