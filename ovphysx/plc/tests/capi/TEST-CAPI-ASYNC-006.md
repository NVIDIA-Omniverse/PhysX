<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-ASYNC-006
maps_to: REQ-CAPI-ASYNC-002
type: integration
---

## Scenario

Short calls preserve results and errors when moving between caller and queue.

## Given

- `WorkerShortCalls` in `tests/c_unittests/test_short_call_queue.cpp`, with a
  CPU scene, live sessions and a deterministic worker hold.
- GPU counterparts in `WorkerShortCallsGpuTest` when CUDA is available.

## When

- Execute the same query/read/write/fetch/release sequence idle and behind a held
  worker; submit a whole chain without waits.
- Overwrite selector and attribute inputs after submission, then release the hold.
- Submit invalid arguments, failed-input calls and calls on detached write sessions.
- Commit a host group without a handoff, then with a handoff or mask; repeat for
  device groups. Attempt a refused device or masked commit and retry validly.
- Open a device read with an empty handoff, then with a producer event.
- Run the CPU suite with its private always-queue and no-worker modes.

## Then

- Idle short calls finish on the caller; busy short calls return pending, write
  summaries/groups at completion and retain submission order (REQ AC-6, AC-9).
- Input mutation after return cannot change queued selections or attributes;
  returned data matches the idle path (REQ AC-10).
- Rejected calls create no task. Execution-state failures have the same error
  text in both modes; detached writes remain identifiable until release
  (REQ AC-11, AC-15).
- Masked, producer-handoff and device commits run on the caller after prior
  work; a refusal leaves the group available for retry (REQ AC-12).
- A read with an empty handoff may queue; a read with a producer event waits for
  earlier work and runs on the caller (REQ AC-9, AC-12).
- Mode-specific cases skip only when their required mode is absent; common
  behavior and ownership assertions run in every selected mode (REQ AC-16).

## Automated coverage

- `WorkerShortCalls.IdleInstanceRunsShortCallsInline`,
  `QueuedCallsReturnPendingAndWriteOutputsWhenTheyRun`, `QueuedCallsKeepSubmissionOrder`,
  `QueryThatParsesTheSceneRunsInlineOnAnIdleInstance`, `InputsAreCopiedAtSubmission`.
- `WorkerShortCalls.StateChecksFailTheTaskWithOneTextOnBothPaths`,
  `ClosedWriteSessionFailsItsCallsWithOneTextOnBothPaths`,
  `MaskedCommitRunsOnTheCallingThreadAfterEarlierWork`, `CommitVenueFollowsItsHandoff`.
- `WorkerShortCallsGpuTest.QueuedDeviceReadMatchesTheCallingThreadRead`,
  `CommitVenueFollowsDeviceMemoryAndHandoff`, `RefusedDeviceCommitIsRejectedWithoutATask`.
- `ReadCaptureGpuTest.CapturedReadReplaysTheCurrentPoses` in the forced-queue
  GPU pass.
- `scripts/test_cpp.cmake` and `tests/c_unittests/test_main.cpp`: execution-mode passes.
