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
session covers the set its query matched, for a **nonempty list of attributes** whose
fetched groups name their original request. Everything else a caller has to learn — session lifecycle, storage ownership,
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

- AC-1: **Session surface, several attributes per session.** ovphysx exposes
  `ovphysx_write(handle, query, attributes, attribute_count, out_write, out_summary)`,
  `ovphysx_fetch_write_next(handle, write, out_group)`,
  `ovphysx_commit_group(handle, write, group, write_done_sync)`, and
  `ovphysx_release_write(handle, write)`. One session plans every requested
  `ovx_string_or_token_t` attribute (string name or interned token); request validation,
  the optional summary and group identity are
  [REQ-CAPI-SESSION-001](REQ-CAPI-SESSION-001.md). It adds **no query surface**:
  `ovphysx_query`, `ovphysx_query_selection` and `ovphysx_release_query` are reused, and
  a session covers exactly the set its query matched. Every operation here returns a
  task receipt under [REQ-CAPI-ASYNC-002](REQ-CAPI-ASYNC-002.md).

- AC-1a: **The write's attribute vocabulary is its own.** A name accepted for write need
  not be one `ovphysx_read` emits: write-only control inputs such as forces and wrenches
  have no read counterpart. Where a name *is* shared with the read, the requested-name vs
  emitted-token distinction documented on `ovphysx_read` applies to it unchanged.

- AC-2: **Group type is `ovstage_map_group_t`, reused verbatim.** No ovphysx mirror
  struct and no translation layer. `ovphysx_fetch_write_next` fills
  `ovphysx_fetched_write_group_t`, whose `group` is a **producer-owned `const` borrow**
  (`const ovstage_map_group_t*`) that the caller never allocates: the group is a
  descriptor the caller reads, describing buffers the caller fills through
  `data.tensors[i].data`. No field of the struct is the caller's to assign.
  `prims.list` / `prims.offset` / `prims.count` / `prims.index_map` carry the read's
  meanings and `data` holds the writable tensors. `meta.layout_generation` is currently
  zero and is not an invalidation signal; rebuild queries and cached setup after a known
  structural change. Tensor shape, dtype, and device are dictated by the implementation,
  not chosen by the caller. The map group has no attribute field, so the fetched
  `attribute_index` is what distinguishes groups of same-shaped attributes.

- AC-3: **Commit takes the group pointer, and addresses are never recycled.** Because
  `ovstage_map_group_t` carries no `write_group_id` (the read's `read_group_id` has no
  twin on it) and ovstage's own `unmap_group` commits by pointer, `ovphysx_commit_group`
  takes the fetched `group` pointer itself, not a copy of the descriptor. Group addresses are therefore unique
  for the life of the session and never reused between groups, since the pointer is the
  commit identity and reuse would make a committed group indistinguishable from a live
  one. This asymmetry against `ovphysx_release_group` is documented at both entry
  points.

- AC-4: **Storage, ownership, and discard on release.** Group storage is
  session-owned and stable across fetches until that live group is accepted for
  publication or the session is released. A live group is spent even when
  publication fails; an invalid or foreign pointer does not consume another
  group's storage. Committing one group does not invalidate another. Accepted
  session release transfers cleanup to the SDK and discards uncommitted groups.
  Detach retires write sessions before their scene or context disappears.
  Callers finish external CPU/CUDA work using uncommitted storage before release
  or detach, which takes no producer completion token. Dereferencing spent or
  released mapped pointers is undefined behavior, not a checked error.

- AC-5: **Synchronization.** `ovphysx_commit_group` takes
  `ovstage_cuda_sync_t write_done_sync` `{stream, wait_event}` — the type ovstage's
  `unmap_group` / `unmap_attribute` take and the mirror of the read's
  `data.cuda_sync`. Physics waits on a supplied `wait_event` before consuming the
  data. `{stream, 0}` — queued work with no event — still requires draining that stream;
  only `{0, 0}` asserts nothing is outstanding. There is no bare `cuda_event` parameter
  anywhere in this contract. Before admission, the C API drains earlier simulation
  on the instance, including a step started after the write session was opened.

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
  The query a write opens over runs the attachment's initial scene parse, a zero-dt update
  with no simulation step, so the articulations parsed at attach are already in their scene.
  Articulation root, link, joint and tendon writes follow the same split as rigid bodies: a
  pre-step write applies on CPU and GPU with readback, and its DirectGPU commit is refused.
  (NVBug 6895491: articulations entered their scene only at the first step, so a pre-step
  articulation write planned no group.)

- AC-6: **Completeness, and what a failed session leaves behind.** The caller fills
  every mapped entry of a group before committing it. There is no signal for "which
  entries did I fill", so a partially filled group that is committed publishes whatever
  its unfilled entries contain. A group that is never committed is discarded by AC-4 and
  publishes nothing, so a caller that fails or throws mid-fill cannot leak uninitialized
  data. Opening a session publishes nothing. Committed groups are not rolled back: each
  accepted commit spends only its own group, a session that fails after some commits
  leaves those applied, and restoring a known state is the caller's responsibility.
  Commits execute in the caller's order, whatever the fetch order or attribute kind. The
  header states these rules explicitly.

- AC-7: **Iteration and errors.** Successful fetch completion with
  `out_group->group == NULL` and `out_group->attribute_index == SIZE_MAX` means
  exhaustion. Rejection and failed completion are not exhaustion.
  Null required arguments, invalid instances, and absent or foreign resource
  handles reject admission. With a valid owned write session, an invalid, foreign,
  or spent non-null group pointer produces an accepted failed commit task and publishes
  nothing. Release of an absent handle on a valid instance produces a successful
  task; release of a live foreign handle rejects admission. No case terminates
  the process. Execution errors are retrieved from the task wait result, not
  inferred from the admission status or the thread's last-error string.

- AC-8: **Validation is C-first.** Argument, handle, and lifecycle validation live in
  the C API so every frontend inherits it. The Python layer stays thin and adds no
  checks the C API does not already make.

- AC-9: **Python surface, filling in place.** `PhysX.write(query, attributes, *,
  metadata=False)` takes a completed `Query` and a sequence of names or tokens and returns
  an `Operation` whose result is a `WriteSession`; opening neither fetches nor waits.
  `WriteSession.fetch_next()` returns an `Operation` whose result is a `WriteGroup`
  carrying `attribute_index` and the canonical `attribute` token, or `None` at
  exhaustion. `WriteSession.commit(group, cuda_stream=0, cuda_wait_event=0, *,
  event_owner=None)` submits exactly that group, spends it on admission, and returns an
  `Operation`; a second commit of the same group raises. `close()` and context exit
  submit release without waiting and discard uncommitted groups per AC-4. `commit_sync`,
  `close_sync` and `PhysX.write_sync(object_type, attributes, ...)` are the explicit
  waiting adapters; `write_sync` owns its internal query, fetches every group, and waits
  for release on context exit. An exception does not roll back earlier commits or a
  partially applied failed commit. The caller commits each group explicitly. Operation
  semantics are REQ-PYTHON-TASK-001.

  Every Python group tensor is a `warp.array` on the tensor's native CPU or CUDA
  device. A non-empty tensor is a **writable alias of the mapped memory, never a
  copy**; an empty tensor is a Warp-owned empty array because it has no native
  pointer to alias. A non-empty alias is invalid once its group's commit is accepted or
  its session release is accepted, per AC-4. The Python surface keeps the explicit
  `ovstage_cuda_sync_t` handoff: a caller with queued CUDA work passes its stream
  and event to `WriteSession.commit()`, keeping any event owner alive through
  collection, rather than relying on `__exit__` timing. No automatic stream or event
  inference is added. C-API errors surface as Python exceptions.

- AC-10: **Threading follows the shipped contract.** `ovphysx.h` states that a single
  instance is not thread-safe, and that different instances may step their own
  attachments concurrently. Each instance holds at most one ovstage attach; several
  may be attached at once. A write session must not be driven from two threads. It
  may run while another instance steps its own attachment. Process-wide limits still
  apply: one hard CPU-only mode, one SDK tolerance scale, and a recording transition
  that needs every attached instance idle.

- AC-11: **Deprecation of the whole tensor-binding surface.** The named binding entry
  points — `ovphysx_create_tensor_binding`, `ovphysx_destroy_tensor_binding`,
  `ovphysx_get_tensor_binding_spec`, `ovphysx_read_tensor_binding`,
  `ovphysx_write_tensor_binding`, `ovphysx_write_tensor_binding_masked`,
  `ovphysx_tensor_binding_get_prim_paths` — **and** the helpers that take a required
  `ovphysx_tensor_binding_handle_t` and are unreachable without one —
  `ovphysx_get_articulation_metadata`, `ovphysx_articulation_get_{dof,body,joint}_names`,
  `ovphysx_rigid_body_view_wake_up`, `ovphysx_rigid_body_view_sleep`, plus the
  `ovphysx_articulation_metadata_t` type — are
  marked deprecated **consistently** across the C header, the Python bindings, the generated
  documentation, the changelog, and the samples — not the header alone. Each points at its
  successor where one exists: `ovphysx_read` for the read half, this API for the write half,
  the whole-articulation
  topology-metadata read attributes on `ovphysx_read` (`dofCount`, `linkCount`, `jointCount`,
  `fixedTendonCount`, `spatialTendonCount`, `isFixedBase`) for `ovphysx_get_articulation_metadata`, and
  the `wake` write attribute on `ovphysx_write` for `ovphysx_rigid_body_view_wake_up` / `_sleep`, and
  **prim-path identity** for `ovphysx_articulation_get_{dof,body,joint}_names` — link and joint names are
  the leaf name of the `kOvxArticulationLink` / `kOvxArticulationJoint` prim served by `ovphysx_read`,
  and per-DOF identity is the whole-articulation `dofJointPath` / `dofAxisIndex` read attributes, which
  additionally make an inverse-dynamics matrix column joinable to its joint and axis. The joint-name
  successor covers only DOF-bearing joints: a fully-locked (zero-DOF) joint has no joint read group or
  descriptor entry, so its name — which the deprecated getter returned — is not yet recoverable through
  the read API, a residual noted against blocker 1 in the deprecation plan. Every other deprecated
  helper points at a landed successor. A
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

- AC-13: **A failed commit reports which failure occurred.** An invalid,
  absent, or foreign write session rejects admission. For a valid owned session,
  an unknown, foreign, or already-committed group fails its task before publication;
  nothing is written and no other session's group is consumed. A live group is
  spent when accepted for publication, including when a preflight check or
  publication fails. Its task error preserves the runtime cause and any known
  write outcome; otherwise it reports that partial application is possible.
  A sidecar failure with no runtime classification reports an unknown outcome.
  Python preserves the same cause before cleanup can replace thread-local errors.
  The runtime's optional `OvxCommitFailure` output carries the distinction that
  its boolean result cannot express.

- AC-14: **Removal blockers are closed, except the deferred masked write.** The whole
  tensor-binding surface can be *removed* only once its consumers have a non-binding
  replacement. **Path/pattern selection on the read is no longer a gap** (formerly (a)):
  `ovphysx_query_selection` adds an explicit prim-handle-list / glob selection to the query that
  narrows both read and write ([REQ-CAPI-SELECT-001](REQ-CAPI-SELECT-001.md), ADR-0025), the 1:1
  successor to a tensor binding's prim subset — the static path-set fully subsumes the indexed write;
  only the deprecated masked write's data-derived per-row mask is deferred as a separate item. The
  **binding-coupled kinematic update is also no longer a gap**: the primary
  `ovphysx_update_articulations_kinematic` now uses query membership, accepts POSITION / VELOCITY flags,
  and executes on CPU and GPU (with DirectGPU's native all-component refresh), so the former
  binding-coupled FK blocker is closed. The binding-coupled **names** helpers
  `ovphysx_articulation_get_{dof,body,joint}_names`, `ovphysx_get_articulation_metadata`'s topology
  metadata, and `ovphysx_rigid_body_view_wake_up` / `_sleep`'s sleep control are also no longer gaps:
  they now have successors (respectively, prim-path identity plus the `dofJointPath` / `dofAxisIndex`
  DOF-column descriptor, for DOF-bearing joints — a fully-locked zero-DOF joint's name is a documented
  limitation (AC-11); the whole-articulation `dofCount` / `linkCount` / `jointCount` /
  `fixedTendonCount` / `spatialTendonCount` / `isFixedBase` read attributes; and the `wake` write
  attribute — AC-11). With selection, kinematic update, names, metadata, and sleep all served by
  non-binding successors, the removal blockers this criterion enumerated are closed; only the deferred
  data-derived masked write remains as a separate follow-up.

- AC-15: **Character controllers use the generic surface.**
  `OVPHYSX_QUERY_TARGET_CHARACTER_CONTROLLER` / `SimObjectType.CHARACTER_CONTROLLER` is value 12.
  Value 11 is the contact-sensor type.
  The only character-controller attribute is `move`, classified `CONDITIONAL` because a
  DirectGPU scene refuses it. Position and orientation are not character-controller
  attributes; the controller prim's pose is the rigid-body position and orientation.
  `ovphysx_query` and `ovphysx_query_selection` both admit the type, including selection
  by `prim_paths` and by pattern. No CCT-specific public entry point is added. Public C
  and Python enum values and the C-layer writability snapshot stay aligned with the
  runtime tables. Movement happens at the write, so the parse-only `simulate(0)` before
  the first public step does not move a controller.

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
- ovphysx/include/ovphysx/ovphysx_types.h (`ovphysx_write_handle_t`, `ovphysx_fetched_write_group_t`; writability classification)
- ovphysx/src/ovphysx/ (write session implementation + C-first validation)
- ovphysx/python/ovphysx/api.py (`PhysX.write`, `PhysX.write_sync`, `WriteSession`, `WriteGroup`)
- ovphysx/python/ovphysx/_operations.py (`Operation`, the typed result of every Python write step)
- ovphysx/python/ovphysx/_bindings.py (write entry-point bindings and the fetched-group layout)
- ovphysx/docs/changelog.md (user-facing behavior change + deprecation notice)
- ovphysx/tests/c_samples/ (still outstanding — runnable write sample, sibling of output_read_c)
- ovphysx/tests/python_samples/session_write.py (runnable Python write sample)
- ovphysx/docs/tutorials/tensor_bindings.md (literalinclude of session_write.py; the C side, tests/c_samples/, remains outstanding)

## Dependencies

- [REQ-CAPI-SESSION-001](REQ-CAPI-SESSION-001.md) (request validation, summaries and fetched-group identity)
- ovphysx/ovruntime/include/omni/physx/IOvxPhysicsWrite.h (runtime write and commit contract)
- ovphysx/ovruntime/source/omni.physx/plugins/OvxPhysicsWrite.cpp (scatter, type coverage and device residency)
- ovphysx/ovruntime/include/omni/physx/RuntimeError.h (shared synchronous runtime cause capture)
