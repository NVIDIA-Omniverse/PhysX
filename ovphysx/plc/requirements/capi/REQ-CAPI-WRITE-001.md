<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-WRITE-001
title: App-to-Physics Write API (public C + Python surface)
status: implemented
owner: ovphysx
---

## Description

ovphysx shall expose an **app → physics** write API shaped as the mirror of the
output read API, replacing the write-oriented tensor-binding surface
(`ovphysx_write_tensor_binding` / `ovphysx_write_tensor_binding_masked`) as the
supported way for an application to push simulation state in.

The write reuses the read's query verbatim and reaches exactly what the read reaches: a
session covers the set `ovphysx_query` matched for `(object_type, scope)`, for **one
attribute**. Everything else a caller has to learn — session lifecycle, storage ownership,
synchronization handoff, iteration termination, error reporting — is the read's contract
applied in the opposite direction, so a group fetched from one direction feeds the other
with no repack.

The [session overview](../../../docs/read_write/index.md),
[writable data](../../../docs/read_write/writable.md), and
[device contract](../../../docs/read_write/device.md) describe this public surface.
The runtime scatter is implemented in
`ovphysx/ovruntime/source/omni.physx/plugins/OvxPhysicsWrite.cpp`; public C and Python
write tests verify its observable behavior through this surface.

## Acceptance Criteria

- AC-1: **Session surface, one attribute per session.** ovphysx exposes
  `ovphysx_write(handle, query, attribute, out_write)`,
  `ovphysx_fetch_write_next(handle, write, out_group)`,
  `ovphysx_commit_group(handle, write, group, write_done_sync)`, and
  `ovphysx_release_write(handle, write)`. A session carries exactly one
  `ovx_string_or_token_t` attribute (string name or interned token); writing several
  attributes over one prim set means several sessions. It adds **no query surface**:
  `ovphysx_query`, `ovphysx_fetch_query_result`, `ovphysx_query_shared_dictionary`, and
  `ovphysx_release_query` are reused unchanged, and a session covers exactly the set its
  query matched.

- AC-1a: **The write's attribute vocabulary is its own.** A name accepted for write need
  not be one `ovphysx_read` emits: write-only control inputs such as forces and wrenches
  have no read counterpart. Where a name *is* shared with the read, the requested-name vs
  emitted-token distinction documented on `ovphysx_read` applies to it unchanged.

- AC-2: **Group type is `ovstage_map_group_t`, reused verbatim.** No ovphysx mirror
  struct and no translation layer. `ovphysx_fetch_write_next` yields a **producer-owned
  `const` borrow** (`const ovstage_map_group_t**`) that the caller never allocates:
  the group is a descriptor the caller reads, describing buffers the caller fills
  through `data.tensors[i].data`. No field of the struct is the caller's to assign.
  `prims.list` / `prims.offset` / `prims.count` / `prims.index_map` carry the read's
  meanings, `data` holds the writable tensors, and `meta.layout_generation` bumps on
  structural change so a writer can cache its own setup while it is stable. Tensor
  shape, dtype, and device are dictated by the implementation, not chosen by the
  caller. Because the session carries one attribute, the group needs no attribute field
  to be unambiguous.

- AC-3: **Commit takes the group pointer, and addresses are never recycled.** Because
  `ovstage_map_group_t` carries no `write_group_id` (the read's `read_group_id` has no
  twin on it) and ovstage's own `unmap_group` commits by pointer, `ovphysx_commit_group`
  takes the borrowed `const ovstage_map_group_t*`. Group addresses are therefore unique
  for the life of the session and never reused between groups, since the pointer is the
  commit identity and reuse would make a committed group indistinguishable from a live
  one. This asymmetry against `ovphysx_release_group` is documented at both entry
  points.

- AC-4: **Storage, ownership, and discard on release.** Group storage is session-owned
  and stable until that group is committed, independent of further fetches — the
  guarantee `ovphysx_fetch_read_next` gives. Ownership of the mapped data transfers to
  physics on commit. Committing one group does not invalidate another. The session is
  valid until `ovphysx_release_write`, which **discards** every group that was never
  committed rather than publishing it, and therefore takes no sync token. After commit
  the mapped pointers belong to physics; the header states that dereferencing them is
  undefined behavior rather than a checked error, since a raw `data.tensors[i].data`
  access makes no API call the runtime could reject.

- AC-5: **Synchronization.** `ovphysx_commit_group` takes
  `ovstage_cuda_sync_t write_done_sync` `{stream, wait_event}` — the type ovstage's
  `unmap_group` / `unmap_attribute` take and the mirror of the read's
  `data.cuda_sync`. Physics waits on a supplied `wait_event` before consuming the
  data. `{stream, 0}` — queued work with no event — still requires draining that stream;
  only `{0, 0}` asserts nothing is outstanding. There is no bare `cuda_event` parameter
  anywhere in this contract. The runtime additionally orders the write against an
  in-flight step, draining pending simulation before scattering.

- AC-5a: **A first write is never auto-warmed; refusal is DirectGPU-only.** The ovstage
  write never calls `ovphysx_warmup_if_needed` (the tensor-binding write does — see
  ADR-0012's 2026-08-27 amendment). Device modes differ on a scene that has not yet
  stepped:
  - **CPU** and **GPU with readback** (no `/physics/suppressReadback`): a pre-step write
    **commits and is applied**. The host scatter has somewhere to land.
  - **DirectGPU** (`eENABLE_DIRECT_GPU_API` / `suppressReadback`): commit is **refused**.
    The DirectGPU superset view it scatters into does not exist until the first step
    (ADR-0008 Decision 10's DirectGPU sizing). The runtime log names
    `setVelocityColumnOvStage` (and siblings) as requiring a prior step.
  The write never advances simulation time on the caller's behalf in any mode. A caller
  that wants one recipe for all three modes calls `ovphysx_step` / `ovphysx_warmup()`
  first; that is portability, not a CPU/GPU refusal. The header states the split at the
  write entry points. (NVBugs 6763272: earlier text treated the DirectGPU refusal as
  universal.)

- AC-6: **Completeness, and what a failed session leaves behind.** The caller fills
  every mapped entry of a group before committing it. There is no signal for "which
  entries did I fill", so a partially filled group that is committed publishes whatever
  its unfilled entries contain. A group that is never committed is discarded by AC-4 and
  publishes nothing, so a caller that fails or throws mid-fill cannot leak uninitialized
  data. Committed groups are not rolled back: a session that fails after some commits
  leaves those applied, and restoring a known state is the caller's responsibility. The
  header states both rules explicitly.

- AC-7: **Iteration and errors.** `ovphysx_fetch_write_next` returns
  `OVPHYSX_API_END_OF_ITERATION` — not an error — once all groups are consumed, with
  `*out_group` NULL; that is the only non-error exit, and any other non-`SUCCESS`
  status is a failure that must not be read as exhaustion. Null out-params and null
  required arguments return `OVPHYSX_API_INVALID_ARGUMENT`. Unknown-handle behavior is
  scoped by endpoint, and the split follows whether the call mutates:
  `ovphysx_write`, `ovphysx_fetch_write_next`, and `ovphysx_commit_group` **return an
  error** for a handle or group pointer the runtime does not recognise as live — commit
  is the mutation itself, so reporting success would tell the caller state was published
  when nothing was. Only `ovphysx_release_write` is **idempotent success** for an
  already-released or unknown handle, matching `ovphysx_release_read`, so teardown is
  always safe. No case terminates the process.

- AC-8: **Validation is C-first.** Argument, handle, and lifecycle validation live in
  the C API so every frontend inherits it. The Python layer stays thin and adds no
  checks the C API does not already make.

- AC-9: **Python surface, filling in place.** `PhysX.write()` returns a context manager
  whose `__exit__` releases the session and discards uncommitted groups per AC-4.
  An exception does not roll back earlier commits or a partially applied failed commit.
  The caller commits each group explicitly.

  Every Python group tensor is a `warp.array` on the tensor's native CPU or CUDA
  device. A non-empty tensor is a **writable alias of the mapped memory, never a
  copy**; an empty tensor is a Warp-owned empty array because it has no native
  pointer to alias. A non-empty alias is invalid after its group is committed or
  its session is released, per AC-4. The Python surface keeps the explicit
  `ovstage_cuda_sync_t` handoff: a caller with queued CUDA work passes its stream
  and event to `WriteSession.commit()` rather than relying on `__exit__` timing.
  No automatic stream or event inference is added. C-API errors surface as Python
  exceptions.

- AC-10: **Threading follows the shipped contract, unchanged.** `ovphysx.h` states that
  instances share the underlying omni.physx runtime and attached stage, and that
  simulation, stage mutation and binding creation must be **serialized across
  instances**; operations on a single instance are not thread-safe. A write session is
  subject to both rules: it must not be driven from two threads, and it must not run
  concurrently with work on another instance. This requirement introduces no per-instance
  isolation and must not be read as granting any.

- AC-11: **Deprecation of the whole tensor-binding surface.** The named binding entry
  points — `ovphysx_create_tensor_binding`, `ovphysx_destroy_tensor_binding`,
  `ovphysx_get_tensor_binding_spec`, `ovphysx_read_tensor_binding`,
  `ovphysx_write_tensor_binding`, `ovphysx_write_tensor_binding_masked`,
  `ovphysx_tensor_binding_get_prim_paths` — **and** the helpers that take a required
  `ovphysx_tensor_binding_handle_t` and are unreachable without one —
  `ovphysx_get_articulation_metadata`, `ovphysx_articulation_get_{dof,body,joint}_names`,
  `ovphysx_articulation_update_kinematic`, `ovphysx_rigid_body_view_wake_up`,
  `ovphysx_rigid_body_view_sleep`, plus the `ovphysx_articulation_metadata_t` type — are
  marked deprecated **consistently** across the C header, the Python bindings, the generated
  documentation, the changelog, and the samples — not the header alone. Each points at its
  successor where one exists: `ovphysx_read` for the read half, this API for the write half,
  and `ovphysx_update_articulations_kinematic` for the binding FK update. The metadata/names
  and wake/sleep helpers have **no non-binding successor yet**; that gap is a
  **removal-blocker** (a read-API topology/names path and a session wake/sleep control),
  detailed in AC-14 — it is not a reason to leave them looking supported. A
  helper being unreachable without the deprecated binding makes it part of the surface that
  is removed with it. Compatibility behavior is retained for the approved deprecation period;
  **removal is not required in ovphysx 0.6** unless the approved compatibility policy permits it.

- AC-12: **The documented loop is a runnable sample, not prose.** A C sample under
  `ovphysx/tests/c_samples/` and a Python sample under `ovphysx/tests/python_samples/`
  drive the full session — query, write, fetch, fill, commit, release — including the
  `ovstage_cuda_sync_t` handoff on the device path and error handling at each step. Both
  are built and run by the sample suites, and the published documentation includes them
  by `{literalinclude}` rather than restating them, matching the `output_read_c` sample
  the read API already ships. A documented lifecycle nobody compiles is a documented
  lifecycle nobody verified.

- AC-13: **A failed commit reports WHICH failure it was.** `ovphysx_commit_group` separates a
  group that was never live — unknown session, or a group unknown, foreign or already
  committed — from a live group whose publish failed. The first is rejected before any
  publish, so it truthfully reports that nothing was written. A live group is spent even
  when a preflight check rejects it before publishing. Its error preserves the runtime's
  cause and any known write outcome; otherwise it reports that partial application is
  possible, because a device scatter can fail after writing some rows. Neither case may
  be described with the other's wording. The Python exception preserves the same cause
  before releasing query or session handles, which may replace the last-error string. A failure
  the runtime does not classify (the write sidecar faulted or is absent) is reported as
  unknown rather than as either. The distinction is carried out of the runtime by
  `ovxCommitGroup`'s optional `OvxCommitFailure` out-param, since the `bool` cannot express it.

- AC-14: **Removal is gated on read-API capabilities that do not exist in 0.6.** The whole
  tensor-binding surface can be *removed* only once its consumers have a non-binding
  replacement. Three gaps block that: (a) **path/pattern selection on the read** — `ovphysx_query`
  selects all objects of a type, with no prim/pattern argument, so a binding read of a specific
  prim subset has no 1:1 session equivalent (it must read-all-then-filter through the ovx
  path-dictionary API); and (b) the **binding-coupled helpers with no successor** —
  `ovphysx_get_articulation_metadata`, `ovphysx_articulation_get_{dof,body,joint}_names`, and
  `ovphysx_rigid_body_view_wake_up` / `_sleep` (AC-11) expose topology, names and sleep control
  the read/query API does not, so removing the create/destroy lifecycle they require means the
  read API must grow those first; and (c) the **binding-coupled kinematic update has only a
  partial successor** — `ovphysx_articulation_update_kinematic` is scoped to a binding's
  articulation subset and honors POSITION / VELOCITY flags, but the non-binding
  `ovphysx_update_articulations_kinematic` updates every articulation in the instance, ignores
  those flags, and is a no-op on CPU (`CpuSimulationView::updateArticulationsKinematic` is empty),
  so subset, flag-selective, and CPU callers have no equivalent yet. Removal must not proceed until
  all three gaps are closed.

## Test References

- [TEST-CAPI-WRITE-001](../../tests/capi/TEST-CAPI-WRITE-001.md) — the write-session entry points are implemented and
  covered by the write-session tests. AC-11's deprecation markers are landed on
  the tensor-binding surface (C header `OVPHYSX_DEPRECATED_MSG`, the C++
  `ovphysx::TensorBinding` wrapper, and the `PhysX.create_tensor_binding`
  `DeprecationWarning`). The per-language compat smoke test is landed:
  `ovphysx/tests/python_tests/test_tensor_binding_deprecation.py` asserts the deprecated
  factory still works AND emits its `DeprecationWarning`, and the retained
  `ovphysx/tests/c_unittests/test_tensor_binding.cpp` is the C/C++ compat coverage.

## Code References

- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_write`, `ovphysx_fetch_write_next`, `ovphysx_commit_group`, `ovphysx_release_write`; deprecation markers on the superseded entry points)
- ovphysx/include/ovphysx/ovphysx_types.h (`ovphysx_write_handle_t`; writability classification)
- ovphysx/src/ovphysx/ (write session implementation + C-first validation)
- ovphysx/python/ovphysx/api.py (`PhysX.write` context manager)
- ovphysx/python/ovphysx/_bindings.py (write entry-point bindings)
- ovphysx/docs/changelog.md (user-facing behavior change + deprecation notice)
- ovphysx/tests/c_samples/ (still outstanding — runnable write sample, sibling of output_read_c)
- ovphysx/tests/python_samples/session_write.py (runnable Python write sample)
- ovphysx/docs/tutorials/tensor_bindings.md (literalinclude of session_write.py; the C side, tests/c_samples/, remains outstanding)

## Dependencies

- ovphysx/ovruntime/include/omni/physx/IOvxPhysicsWrite.h (runtime write and commit contract)
- ovphysx/ovruntime/source/omni.physx/plugins/OvxPhysicsWrite.cpp (scatter, type coverage and device residency)
- ovphysx/ovruntime/include/omni/physx/RuntimeError.h (shared synchronous runtime cause capture)
