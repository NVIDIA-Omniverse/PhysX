<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-WRITE-001
maps_to: REQ-CAPI-WRITE-001
type: integration
---

## Scenario

The public write surface round-trips against the read over the same query, and holds the
read's lifecycle and error contracts in the write direction. The central case is the one
the API exists for: **read a group → modify → write it back through a write session on
the same query → read again and see the modified values**. Around it sit the lifecycle
and misuse cases, which must be rejected rather than fatal, and the Python context
manager, whose exit is the only thing standing between a caller and a forgotten commit.

Exercised through the C API first (the C-first validation rule of REQ AC-8 means the
Python cases must not be able to reach a check the C cases cannot), then repeated
through the Python surface. The public implementations are
`ovphysx/tests/c_unittests/test_write_api.cpp`,
`ovphysx/tests/python_tests/test_write_api.py`, and, for the default Python operation
surface, `ovphysx/tests/python_tests/cpu_tests/test_session_async.py`. CCT coverage also uses
`ovphysx/tests/c_unittests/test_writability.cpp`,
`ovphysx/tests/python_tests/test_writability.py`, and
`ovphysx/tests/python_tests/cpu_tests/test_character_controller_read_write.py`.
Additional runtime-only regression cases are not included in the public source distribution.

`ovphysx/tests/c_unittests/test_session_physics.cpp` adds further **AC-1** coverage of
the write-session surface (`ovphysx_write` / `ovphysx_fetch_write_next` /
`ovphysx_commit_group` / `ovphysx_release_write`, one attribute per session over the
read's query): the session-API successor to the physics-fact tests the deprecated
tensor-binding file checked — disable gravity/simulation, wake, forces and wrenches, DOF
drive type, projected joint force, and selected writes — each writing a column and reading
it back through the same query on CPU, ordinary GPU dynamics and DirectGPU.

The Python write surface (**AC-9**, `PhysX.write()` filling group tensors in place and committing each
group) is exercised by the session suites `cpu_tests/test_session_properties.py`,
`cpu_tests/test_session_shape.py`, `cpu_tests/test_session_rw_semantics.py`,
`cpu_tests/test_session_warp.py`, `test_session_api_behavior.py` and, on DirectGPU,
`test_session_warp_gpu.py`, `test_session_rigid_body_gpu.py`, `test_session_deformable_gpu.py` and
`test_session_properties_gpu.py` — fill-in-place write round-trips confirmed by read-back, with read-only
attributes refusing a write as a Python exception.

The pre-step cases of **AC-5a** live in `ovphysx/tests/python_tests/cpu_tests/test_write_prestep.py` (CPU),
`ovphysx/tests/python_tests/test_gpu_lifecycle_advanced.py` (GPU with readback, one subprocess per case) and
`ovphysx/tests/python_tests/test_write_api.py` (DirectGPU), for rigid bodies and for articulation roots,
links, joints and tendons.

## Given

- A stepped ovphysx instance with dynamic rigid bodies and an articulation, run on
  CPU, on GPU with readback, and on GPU under DirectGPU (`--forceGpu --directGpu` /
  Python `suppressReadback`). The round-trip cases need a stepped scene; AC-5a is the
  unstepped exception below. The write and the legacy binding branch on
  `PxSceneFlag::eENABLE_DIRECT_GPU_API`.
- A separate plain `PhysX()` subprocess running GPU dynamics with default Carbonite
  settings and no `suppressReadback` override. It warms up and steps once before
  writing `disableGravity`.
- A read session over `OVPHYSX_QUERY_TARGET_RIGID_BODY` / `OVPHYSX_SCOPE_ALL` for
  `OVPHYSX_ATTR_POSITION` and `OVPHYSX_ATTR_ORIENTATION`, supplying the baseline values.
  The write half here opens one single-attribute session per attribute; identity across
  several attributes in one session is TEST-CAPI-SESSION-001.
- A scene in which some bodies are asleep, so an `OVPHYSX_SCOPE_ACTIVE` query matches a
  proper subset of the `OVPHYSX_SCOPE_ALL` set.
- The attached Stage's path dictionary, to resolve interned handles when a case needs to
  name prims.
- A CPU scene containing standalone dynamic and kinematic rigid bodies, with both
  velocity columns captured before writing.
- A CPU scene containing two capsule character controllers, queried through
  `OVPHYSX_QUERY_TARGET_CHARACTER_CONTROLLER` and read back as rigid-body poses.

## When

- **Round trip.** A read group's poses are captured; a write session is opened over the
  same query for `OVPHYSX_ATTR_POSITION`, every mapped entry of every fetched group is
  filled with a modified pose, each group is committed and the session released; a second
  session repeats it for `OVPHYSX_ATTR_ORIENTATION`; both attributes are then read back.
- **Query reach.** A write session is opened on an `OVPHYSX_SCOPE_ACTIVE` query, filled,
  and committed; the full `OVPHYSX_SCOPE_ALL` set is then read back.
- **Lifecycle.** A group is committed twice. A committed group's mapped pointers are
  used again. A session is released with groups still uncommitted. A session is
  released twice. A second group is fetched while an earlier one is still uncommitted,
  and the earlier one is then committed.
- **Iteration.** `ovphysx_fetch_write_next` is driven past the last group.
- **Failure partway.** A session with several groups is abandoned after committing some
  but not all of them, with the uncommitted group partially filled with a recognisable
  sentinel, and a fresh session is then opened over the same query.
- **Discard on release.** A session is opened, a group is fetched and filled, and the
  session is released **without** committing it.
- **Errors.** A null `out_group`, a null out-param on `ovphysx_write`, a zero attribute
  count, an attribute name the queried type does not accept, and a read-only attribute
  name are each submitted.
- **Diagnostic preservation.** Unknown and read-only rigid-body names, and an attribute
  readable only on another object type, are rejected through C and Python. Python cleanup
  is also exercised with a query release that clears the native last-error string.
  An application log callback also attempts `ovphysx_set_log_level()` while an
  unknown attribute is being rejected; the callback-time call is itself rejected.
- **Mixed velocity group.** Through Python, the dynamic/kinematic group is filled and
  committed once for `linearVelocity` and once for `angularVelocity`, then read without
  stepping.
- **First-write, no auto-warm.** On a freshly attached scene that has not been warmed up
  or stepped, a write session is opened and one group committed. The case is run on CPU,
  on GPU with readback, and on DirectGPU (`--forceGpu --directGpu` / Python
  `suppressReadback`), because AC-5a is mode-specific.
- **Articulation first-write.** The same case with the first operation after attach being an
  articulation root (`rootPosition`), link (`mass`) or joint (`jointPosition`) write over
  `mixed_base_articulations.usda`, with no read, lookup, step or warmup before it, and on
  DirectGPU also a fixed-tendon `tendonDamping` write. On CPU, each of the 34 writable
  root, link and joint columns is also opened that way on a fresh attachment.
- **Unknown handles.** A handle that was never issued is passed to each of
  `ovphysx_fetch_write_next`, `ovphysx_commit_group`, and `ovphysx_release_write`, and a
  group pointer that was already committed is passed to `ovphysx_commit_group` again.
- **Character-controller classification.** The C and Python writability helpers are
  queried for CCT position, orientation, and `move`; `move` is conditional, position
  and orientation are unclassified, enum value 12 is accepted, and `OVPHYSX_QUERY_TARGET_COUNT` is rejected.
- **Character-controller behavior.** Public path and pattern selections each write
  `move` on one of two controllers. The other controller's rigid-body pose is unchanged.
  Two writes before one step move by the sum; the next step does not replay it; the
  rigid-body pose stays at the pre-write value until that step.
- **Character-controller summaries.** Python default operations request query, read
  and write metadata for all controllers, one selected controller, and an empty
  selection. The public C selection case also checks query and write summaries.
- **Python.** The same round trip runs through the `PhysX.write_sync()` context manager,
  once exiting normally and once exiting via an exception raised inside the block.
- **Python sample.** Run `tests/python_samples/session_write.py`, including its
  unknown-attribute request, through the sample suite.
- **Python default operations.** A completed query opens one `PhysX.write()` session for
  `linearVelocity` and `angularVelocity` without metadata; the query is closed; every group
  is fetched, filled by its `attribute_index` and committed through operations; both
  attributes are read back.
- **Python ordinary GPU dynamics.** After bodies are moving under gravity,
  `PhysX.write_sync()` changes only `disableGravity` before the next step. Disabled bodies'
  vertical velocities are compared with gravity-enabled controls, then gravity is
  re-enabled and the simulation steps again.
- **Python fill on both residencies.** Non-empty host and device tensors are filled
  through the `warp.array` objects Python hands back. The shared converter is also
  exercised with a zero-element host DLTensor. The device fill is committed with the
  caller's current Warp stream. Both non-empty tensors are read back natively; the
  forced-race event case is not included in the public test suite.

## Then

- The read-back values equal what was written, for every prim in the written set, on
  both devices (REQ AC-1, AC-2).
- A write session covers exactly the set its query matched: after the `ACTIVE`-scope
  write, every prim in the `ALL` set that the `ACTIVE` query did not match reads back
  bit-identical to its pre-write value (REQ AC-1). The write exposes no addressing
  parameter of its own; its reach is determined by the query alone.
- `ovphysx_fetch_write_next` fills `ovphysx_fetched_write_group_t` whose `group` is a
  `const ovstage_map_group_t*` the caller never allocated, and the emitted group is the
  ovstage type itself — no ovphysx mirror of the map group appears in the signature. Every
  group of a single-attribute session names request 0 (REQ AC-2). Filling a group requires
  no cast: the caller writes through `data.tensors[i].data`, and the fetch and commit
  signatures agree on the qualifier.
- `ovphysx_commit_group` accepts the fetched group pointer; no `write_group_id` is
  required or available on the struct (REQ AC-3).
- Fetching a later group does not invalidate an earlier uncommitted one: the earlier
  group's storage is intact and commits correctly afterwards (REQ AC-4).
- Both release tasks complete successfully (REQ AC-7). The **double commit does not**: the
  second call returns an accepted task that fails because the group is no longer
  live. Its address was never recycled to another group (REQ AC-3, AC-7).
- Releasing a session with an uncommitted group publishes nothing: the prims that group
  covered read back bit-identical to their pre-write values, and the sentinel written into
  the partially filled group never appears in the simulation (REQ AC-4, AC-6).
- No case terminates the process (REQ AC-7). Not asserted: that dereferencing a
  committed group's mapped pointers is *rejected* — a raw `data.tensors[i].data` access
  makes no API call, so there is nothing to reject. REQ AC-4 states it as undefined
  behavior, and it is enforced by documentation or a sanitizer run, not a return code.
- The abandoned session reports its failure, the process survives, and a fresh session
  over the same query afterwards fetches, fills, commits and reads back correctly. Its
  committed groups stay applied — REQ AC-6 promises no rollback — while its uncommitted,
  partially filled group published nothing.
- Iterating past the last group returns an accepted task; its successful wait
  leaves `group == NULL` and `attribute_index == SIZE_MAX`, and a failed fetch names no
  request either. Exhaustion never reports a task error (REQ AC-7).
- Null out-params, null required arguments and a zero attribute count reject admission
  with `OVPHYSX_API_INVALID_ARGUMENT`. A name the type does not accept, and a read-only
  name, each produce an accepted failed task with a message naming the attribute (REQ AC-7, and REQ-INPUT-COVERAGE-001 AC-5).
- A pre-step write does **not** auto-warm in any mode (simulation time is unchanged by
  the commit itself). On CPU and on GPU with readback the commit **succeeds** and a
  subsequent read (CPU can read before the first step) or the next step shows the written
  values. On DirectGPU the commit **fails** and the write does not land; a later step
  without a new write shows only the solver's own motion (REQ AC-5a).
- An articulation first-write behaves the same way. On CPU and on GPU with readback the
  session plans one group, the commit succeeds and an immediate read returns the written
  value. On CPU the root case also reads zero root velocity, so nothing stepped, and every one
  of the 34 columns plans one group covering every matched prim. On DirectGPU the session plans one group and its
  commit fails with "requires at least one simulation step"; the same write applies after one
  step (REQ AC-5a).
- Unknown write handles reject fetch/commit admission. Releasing an absent write
  on a valid instance returns a successful task. An invalid or already-committed
  group pointer on a valid owned session instead returns an accepted failed task
  that reports no publication (REQ AC-7, AC-13).
- Admission rejections and execution failures are produced by the C API. Python's
  waiting adapters (`write_sync`, `commit_sync`, `close_sync`) collect their tasks and
  raise the same C-level errors (REQ AC-8, AC-9).
- The Python sample catches `TaskError` for its unknown-attribute request; an unrelated
  `RuntimeError` fails the sample instead of satisfying that check (REQ AC-7, AC-9, AC-12).
- `ovphysx_writability` classifies character-controller `move` as conditional and
  position and orientation as unclassified. `ovphysx_query_selection` admits type 12
  by prim path and by pattern. A selected `move` changes only that controller.
  Two commits before one step sum; the next step does not replay them. The
  rigid-body pose is unchanged until the step that integrates the kinematic target.
  Attach's `simulate(0)` does not move a controller that has not been written
  (REQ AC-15).
- Character-controller summaries report the matched prim count. Query metadata has
  no readable tokens; Python reads of `position` and `move` emit no groups and report
  invalid tokens with unsupported-attribute omissions. A `move` write emits one
  group at request index 0 when nonempty, or no groups for an empty match; fetches
  then exhaust normally (REQ AC-1, AC-15;
  `test_character_controller_async_session_metadata`).
- Failed write tasks retain the requested attribute and runtime reason in the C
  task error and Python exception, including after Python cleanup. The
  DirectGPU pre-step rigid-body velocity failure names the first-step requirement and
  `warmup()` or stepping as the remedy (REQ AC-7, AC-9, AC-13).
- Callback-time rejection does not replace the enclosing write's attribute error
  (REQ AC-7; `WriteApiTest.RejectedAttributeSurvivesLogCallbackError`). This checks
  public error preservation, not the runtime's internal nested-scope isolation.
- Each mixed CPU velocity commit raises with the SDK setter's kinematic-body reason.
  Dynamic rows before and after the rejected body read back the requested velocity;
  the kinematic row stays unchanged (REQ AC-13;
  `test_mixed_kinematic_cpu_velocity_reports_partial_write`).
  A second commit of the failed group also raises (REQ AC-13).
- The default-operation session reports `metadata is None` and no prefetched groups; each
  fetched group's `attribute` equals the token of its `attribute_index`; both indices
  appear; the read-back values follow the index each group was filled for
  (REQ AC-9; `test_multi_attribute_write_keeps_request_identity_without_metadata`).
- The `write_sync` context manager releases on both the normal and the exception path without
  leaking the session. On the normal path the explicitly committed groups read back; on
  the exception-before-commit path the filled group is discarded (REQ AC-9, AC-4).
- In the plain `PhysX()` subprocess, disabled bodies coast at roughly constant vertical
  velocity on the first step after the isolated `disableGravity` write while enabled
  controls accelerate downward. After the flag is cleared, all bodies accelerate
  downward, proving the Python write reaches the simulation on ordinary GPU dynamics
  rather than only changing readable host state (REQ AC-1, AC-9).
- Every Python write tensor is a `warp.array` on the native CPU or CUDA device. The
  zero-element result is Warp-owned rather than presented as an alias of a null native
  pointer (REQ AC-9).
- The non-empty host and device arrays read back the values assigned through them. A
  conversion that copied would pass a naive "did the array change" check and fail this
  one (REQ AC-9).
- The device group filled from Python reads back the queued Warp fill when the caller's
  stream is passed explicitly to commit. No automatic stream or event handoff is asserted
  (REQ AC-9, AC-5).
- Every entry point of the tensor-binding surface — both directions, plus the lifecycle
  and introspection calls — still functions during the deprecation period and emits its
  deprecation marker, each naming its successor; the changelog records the new surface
  and the deprecation (REQ AC-11). `ovphysx_rigid_body_view_wake_up` / `_sleep` name the
  `wake` write attribute, `ovphysx_get_articulation_metadata` the six topology-metadata read
  attributes, and `ovphysx_articulation_get_{dof,body,joint}_names` prim-path identity plus the
  `dofJointPath` / `dofAxisIndex` DOF-column descriptor — every binding-coupled helper now names a
  landed successor. Path/pattern selection and query-based kinematic update are available;
  the data-derived per-row masked write remains deferred, and the names successor retains
  the documented zero-DOF joint limitation (REQ AC-11, AC-14). The per-language compat smoke test
  `tests/python_tests/test_tensor_binding_deprecation.py` pins the Python side (the deprecated
  factory works AND raises `DeprecationWarning` naming the successor); the retained
  `tests/c_unittests/test_tensor_binding.cpp` is the C/C++ compat coverage. The names successor's C-ABI
  reachability is exercised by `tests/c_unittests/test_output_read.cpp`'s `ArticulationDofColumnDescriptor`:
  it reads the `dofJointPath` / `dofAxisIndex` descriptor through `ovphysx_read`, resolves each DOF
  column's handle to its joint prim path through the attached Stage's dictionary, and confirms each of the four DOFs
  (two revolute joints on each of two articulations) names a distinct joint prim at axis 0 — the (joint
  prim, axis) identity that replaces `ovphysx_articulation_get_dof_names` (REQ AC-14).
- `MultiInstance.OutputCompletesWhilePeerRuntimeCallbackIsBlocked` in
  `tests/c_unittests/test_multi_instance.cpp` holds a peer's real teardown callback
  on another thread. Query creation and cached-query validation, write/fetch/commit,
  read/fetch/group release and session/query release on the idle owner must all
  finish before that callback returns. Committed CPU masses read back exactly;
  output calls made from the callback itself remain rejected (REQ AC-10).
- `lifecycle_tests/test_multi_instance_output.py::test_output_while_peer_steps`
  runs 300 `write_sync`/`read_sync` round trips while a peer steps, verifies every
  written mass and unchanged peer masses, and repeats with a shared lock as a
  positive control (REQ AC-10).
- Not asserted here: that a second thread driving the same session is rejected. REQ AC-10
  carries `ovphysx.h`'s single-instance serialization rule forward as a caller obligation,
  enforced by documentation, not by a runtime check.
