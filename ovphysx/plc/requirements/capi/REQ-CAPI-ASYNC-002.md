<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-ASYNC-002
title: Task receipts for session and articulation operations
status: implemented
owner: ovphysx
---

## Description

Query, read, write and articulation operations use the task completion contract.
They initially execute inline; accepted work can therefore already be finished
when its receipt is returned. Submission does not promise nonblocking execution.
Session creation results are outputs of the creating task
([REQ-CAPI-SESSION-001](REQ-CAPI-SESSION-001.md)).

## Acceptance Criteria

- **AC-1: Surface.** `ovphysx_query`, `ovphysx_query_selection`,
  `ovphysx_release_query`, `ovphysx_read`, `ovphysx_fetch_read_next`,
  `ovphysx_release_group`, `ovphysx_release_read`, `ovphysx_write`,
  `ovphysx_fetch_write_next`, `ovphysx_commit_group`, `ovphysx_release_write`,
  `ovphysx_update_articulations_kinematic` and `ovphysx_evaluate_sdf_shapes` return
  `ovphysx_enqueue_result_t`. The articulation update takes a live whole-articulation
  query and POSITION/VELOCITY flags; the SDF evaluation takes a live SDF_SHAPE query,
  caller-owned tensors and their producer handoff (REQ-CAPI-SDF-001). There is no separate query-metadata or
  query-dictionary accessor; optional creation results are task outputs.
  `ovphysx_writability` remains an instance-free immediate helper outside this contract.
- **AC-2: Admission and storage.** Validate instance, arguments, known resource
  ownership and required interfaces before admission. Reserve identity and
  failure storage before side effects, including lazy attach and warmup.
  Clear output slots, including optional result slots, before validation. Callers keep
  input arrays/strings and output slots alive through task completion and inspect
  outputs after waiting. A creator that fails leaves zero outputs, so dependent work
  cannot be submitted against an unpublished handle.
- **AC-3: Errors.** Rejection returns zero identity and an immediate error;
  accepted execution failures retain successful admission and are reported by
  waits. Every accepted operation, including an idempotent release, has a real
  collectable identity. Internal C++ consumers wait and collect their hidden tasks;
  Python exposes its receipts and waits only in explicit adapters and teardown
  (REQ-PYTHON-TASK-001). Destructor failures are logged without throwing.
- **AC-4: Iteration.** Successful fetch completion with a null group means normal
  exhaustion; a write fetch also reports `attribute_index == SIZE_MAX`. A backend
  fetch failure is a task error. No iteration status is returned as either admission
  status or generic wait status.
- **AC-5: Ownership.** Query/read/write handles belong to their creating instance;
  another live instance cannot use or release them. Already-absent release is
  idempotent on a valid instance. Accepted release transfers cleanup to the SDK,
  even on execution failure; callers stop using the resource and never retry it.
  Rejected release retains caller ownership. Detach releases write sessions
  before their scene/context disappears; pinned read snapshots retain their
  existing lifetime across detach. Instance teardown releases all remaining sessions.
- **AC-6: Ordering.** Query/read/write creation, commit, articulation update and
  SDF evaluation finish required prior simulation before admission. A mid-session step is
  finished before the next live-scene mutation or snapshot. Internal drains
  preserve earlier uncollected failures without rejecting otherwise-valid work.
  Fetch keeps its same-instance pending-simulation drain; neither fetch nor release
  imposes a global simulation barrier or drains another instance.
  Output session operations and cached-query validation from runtime or application
  log callbacks return `INVALID_STATE` before draining simulation or entering the
  output backend. Log-callback rejection is local to the calling thread; other
  client threads can continue using idle instances.
- **AC-7: Data lifetime.** Fetches return the original borrowed OVStage group;
  commit preserves pointer identity. Read numeric data and synchronization
  objects remain session-owned across steps and group release. Group metadata
  ends at group/session release or terminal attachment retirement. Callers finish external GPU consumers before
  releasing their read session or destroying the instance. Host task completion
  preserves the existing
  CUDA ready-event and producer-done handoffs; it is not a device-wide barrier.
  Commit spends a live group belonging to the session once accepted for
  publication, even if publication fails. Unknown, foreign or already-spent
  pointers publish nothing. Session cleanup waits for queued consumers before
  freeing their storage. Stage-derived prim identities require their original
  live dictionary; only numeric snapshots retain their lifetime across detach.
- **AC-8: Articulation update.** The articulation operation accepts only a query
  owned by the instance, evaluates its membership during execution, applies the
  requested POSITION and/or VELOCITY update without a simulation step, and
  reports stale, wrong-type, unsupported-flag, and backend failures through the
  admission/task split. Non-DirectGPU scenes honor the component mask;
  DirectGPU's native refresh may update both components for either supported
  nonzero mask. Zero flags are a successful no-op after query validation.

## Known limitations

Submission currently executes inline. CPU and existing blocking GPU paths keep
their blocking behavior; this requirement introduces no worker queue or scheduler.

## Test References

- [TEST-CAPI-ASYNC-002](../../tests/capi/TEST-CAPI-ASYNC-002.md)

## Code References

- `ovphysx/include/ovphysx/ovphysx.h`: public signatures and lifetime contract.
- `ovphysx/src/include/internal/sdk/TaskContext.hpp`: task creation and outcome recording.
- `ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp`: session producers, ownership,
  and query-scoped articulation update.
- `ovphysx/src/ovphysx/ovphysxTensorBinding.cpp`: warmup.
- `ovphysx/src/ovphysx/ovphysx.cpp`: pending simulation and instance teardown.
- `ovphysx/src/ovphysx/OvStageOutput.cpp`: internal receipt collection.
- `ovphysx/src/ovphysxInternal/ovphysxInternalInterop.cpp`: runtime result capture.
- `ovphysx/tests/c_unittests/test_async_api.cpp`: session task regressions.

## Dependencies

- [REQ-CAPI-TASK-001](REQ-CAPI-TASK-001.md)
- [REQ-CAPI-WRITE-001](REQ-CAPI-WRITE-001.md)
- [REQ-CAPI-SESSION-001](REQ-CAPI-SESSION-001.md)
