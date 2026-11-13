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
Short calls execute on the caller when the instance is idle and otherwise queue
behind its earlier work. Calls using caller CUDA streams stay on the caller.
Accepted work may finish before return or later; the receipt reports admission.
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
- **AC-2: Admission and storage.** Validate instance, arguments and known handle
  ownership before admission. Reserve identity and failure storage before side
  effects, including lazy attach and warmup. Clear output slots before validation.
  Accepted creators publish their handle before returning; summaries and fetched
  groups are outputs of successful execution. Callers keep those output slots
  alive through completion. Copied inputs follow AC-10; failed creators follow AC-13.
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
- **AC-6: Ordering.** Session operations execute after earlier work on their
  instance, including simulation finalization, without collecting its outcomes.
  Queued submissions do not wait for that work; calling-thread operations acquire
  the instance lane first. A mid-session step finishes before the next live-scene
  mutation or snapshot. Neither fetch nor release imposes a global barrier or
  drains another instance.
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

- **AC-9: Short-call scheduling.** Query creation, read creation with an empty
  CUDA handoff, write creation, group fetches, host releases and eligible host
  commits run on the caller when idle; otherwise they queue in instance order
  without waiting for earlier work. A query that performs initial scene parsing
  follows the same rule.
- **AC-10: Input copies and outputs.** Queued selected/contact queries copy their
  selectors; queued reads/writes copy attribute arrays and string bytes before
  returning, and queued reads copy the CUDA handoff by value. A non-NULL string
  takes precedence and is copied; an entry with a NULL string keeps its token
  (ovstage's `ovx_token_or_string_t` contract). Fetch and summary output slots remain
  borrowed until completion. A queued host commit retains the original group
  pointer; the caller keeps its data alive until completion.
- **AC-11: Stable errors.** Invalid arguments, invalid/foreign/released handles,
  callback reentry, a stopped executor and exhausted admission resources reject
  without a task. State determined by preceding work instead fails the accepted
  task: missing attachment/backend, failed input producer, closed write session
  or failed lifecycle generation. Inline and queued execution use the same
  API/task-specific failure text.
- **AC-12: Caller-stream operations.** A read with a nonempty CUDA handoff,
  articulation update, SDF evaluation, masked commits, commits with a producer
  stream/event, and commits for sessions that exposed device groups execute on
  the caller after earlier instance work. Reads with an empty handoff and
  unmasked host commits with an empty handoff are eligible for queueing. Runtime
  refusal of a caller-thread commit (including a bad mask or capture misuse)
  rejects before spending its group. Write releases, including those for
  sessions that exposed device groups, follow AC-9 scheduling.
- **AC-13: Handles at submission.** Accepted creators return a nonzero,
  non-reused opaque handle immediately. Later same-instance calls may use it
  without first waiting for creation. A failed creator leaves a failed handle;
  consumers fail their own tasks naming the producer, and release safely retires
  it. Foreign handles are rejected even while creation is pending. Accepted
  release prevents later use immediately; earlier admitted calls retain their
  place before that release. Exhausted handle space rejects creation.
- **AC-14: Attachment generations.** Calls bind to the attachment generation
  selected by prior accepted attach/detach submissions. If a lifecycle task
  fails, dependent calls fail until that failure is collected and new calls
  can bind to the surviving attachment. Status queries do not recover a failed
  generation, and collecting a failure does not retarget already-submitted work.
- **AC-15: Closed writes.** Detach retires backend write sessions opened before
  it while retaining their known handles. Consumers of such handles fail their
  tasks consistently whether detach already ran or is queued. Release retires
  the closed handle; writes submitted later are not prematurely retired.
- **AC-16: Execution-mode coverage.** Private test hooks can force short calls
  through the queue even when idle, and can create workerless instances. These
  modes preserve public results, ordering and failure ownership. Test runners
  select the mode; the library does not read the test environment variable.

## Known limitations

Short calls on idle instances and caller-stream operations can block. Callers
collect pending tasks before beginning CUDA graph capture. Host task completion
is not CUDA device completion.

## Test References

- [TEST-CAPI-ASYNC-002](../../tests/capi/TEST-CAPI-ASYNC-002.md)
- [TEST-CAPI-ASYNC-006](../../tests/capi/TEST-CAPI-ASYNC-006.md)
- [TEST-CAPI-ASYNC-007](../../tests/capi/TEST-CAPI-ASYNC-007.md)

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
