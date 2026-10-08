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
through the Python surface.

## Given

- A stepped ovphysx instance with dynamic rigid bodies and an articulation, run once on
  CPU and once on GPU under DirectGPU (`--forceGpu --directGpu --hidden`), since both
  the write and the legacy binding branch on `PxSceneFlag::eENABLE_DIRECT_GPU_API`.
- A read session over `OVPHYSX_OBJECT_RIGID_BODY` / `OVPHYSX_SCOPE_ALL` for
  `OVPHYSX_ATTR_POSITION` and `OVPHYSX_ATTR_ORIENTATION`, supplying the baseline values.
  The write half needs one session per attribute.
- A scene in which some bodies are asleep, so an `OVPHYSX_SCOPE_ACTIVE` query matches a
  proper subset of the `OVPHYSX_SCOPE_ALL` set.
- The shared dictionary from `ovphysx_query_shared_dictionary`, to resolve interned
  handles when a case needs to name prims.

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
- **Errors.** A null `out_group`, a null out-param on `ovphysx_write`, an attribute name
  the queried type does not accept, and a read-only attribute name are each submitted.
- **First-write warmup.** On a freshly attached scene that has not been warmed up or
  stepped, a write session is opened and one group committed, with simulation time
  sampled before and after.
- **Unknown handles.** A handle that was never issued is passed to each of
  `ovphysx_fetch_write_next`, `ovphysx_commit_group`, and `ovphysx_release_write`, and a
  group pointer that was already committed is passed to `ovphysx_commit_group` again.
- **Python.** The same round trip runs through `PhysX.write()` as a context manager,
  once exiting normally and once exiting via an exception raised inside the block.
- **Python fill on both residencies.** Non-empty host and device tensors are filled
  through the `warp.array` objects Python hands back. The shared converter is also
  exercised with a zero-element host DLTensor. The device fill is committed with the
  caller's current Warp stream. Both non-empty tensors are read back natively; the
  forced-race event coverage is owned by
  [TEST-INPUT-DEVICE-001](../../../ovruntime/plc/tests/input/TEST-INPUT-DEVICE-001.md).

## Then

- The read-back values equal what was written, for every prim in the written set, on
  both devices (REQ AC-1, AC-2).
- A write session covers exactly the set its query matched: after the `ACTIVE`-scope
  write, every prim in the `ALL` set that the `ACTIVE` query did not match reads back
  bit-identical to its pre-write value (REQ AC-1). The write exposes no addressing
  parameter of its own; its reach is determined by the query alone.
- `ovphysx_fetch_write_next` yields a `const ovstage_map_group_t*` the caller never
  allocated, and the emitted group is the ovstage type itself — no ovphysx mirror struct
  appears in the signature (REQ AC-2). Filling a group requires no cast: the caller
  writes through `data.tensors[i].data`, and the fetch and commit signatures agree on
  the qualifier.
- `ovphysx_commit_group` accepts the borrowed group pointer; no `write_group_id` is
  required or available on the struct (REQ AC-3).
- Fetching a later group does not invalidate an earlier uncommitted one: the earlier
  group's storage is intact and commits correctly afterwards (REQ AC-4).
- The double release returns success (REQ AC-7). The **double commit does not**: the
  second call is rejected because the group is no longer live, and the group's address was
  never recycled to another group, so the runtime can tell the two apart (REQ AC-3, AC-7).
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
- Iterating past the last group returns `OVPHYSX_API_END_OF_ITERATION` with
  `*out_group == NULL`, and that status is distinguishable from every error status; no
  error status is reachable by exhausting the iteration (REQ AC-7).
- Null out-params and null required arguments return `OVPHYSX_API_INVALID_ARGUMENT`. A
  name the type does not accept, and a read-only name, are each rejected with a message
  naming the attribute (REQ AC-7, and REQ-INPUT-COVERAGE-001 AC-5).
- A write to a scene that has not been stepped is **refused**, not silently warmed: the
  DirectGPU superset view it scatters into does not exist until the first step, so the
  session fails rather than advancing simulation on the caller's behalf. This is symmetric
  with the read's pre-step omission; a caller steps or calls `ovphysx_warmup()` itself
  first (REQ AC-5a).
- Unknown handles split by whether the call mutates, exactly as REQ AC-7 scopes them:
  `ovphysx_fetch_write_next` and `ovphysx_commit_group` return an error, while
  `ovphysx_release_write` returns success. The commit case matters most — a success there
  would tell the caller state was published when nothing was (REQ AC-7).
- Every rejection above is produced by the C API: the Python cases raise exceptions
  carrying the same C-level messages, and no check exists only in Python (REQ AC-8,
  AC-9).
- The Python context manager releases on both the normal and the exception path without
  leaking the session. On the normal path the explicitly committed groups read back; on
  the exception path nothing the block had filled is published (REQ AC-9, AC-4).
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
  and the deprecation (REQ AC-11). The per-language compat smoke test
  `tests/python_tests/test_tensor_binding_deprecation.py` pins the Python side (the deprecated
  factory works AND raises `DeprecationWarning` naming the successor); the retained
  `tests/c_unittests/test_tensor_binding.cpp` is the C/C++ compat coverage.
- Not asserted here: that a second thread driving the same session, or concurrent work on
  another instance, is rejected. REQ AC-10 carries `ovphysx.h`'s serialization rules
  forward as caller obligations — including the cross-instance one, since instances share
  the runtime and attached stage — and they are enforced by documentation, not by a
  runtime check.
