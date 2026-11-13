<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-LANE-001
maps_to: REQ-CAPI-LANE-001
type: integration
---

## Scenario

Caller-thread operations observe prior instance work while independent instances
and recording-status reads remain available.

## Given

- `Lanes` in `tests/c_unittests/test_instance_lanes.cpp`, two CPU instances and
  deterministic holds on instance jobs.

## When

- Call articulation update behind a held worker; submit output work while it runs.
- Submit lifecycle work while a peer's lifecycle job is held.
- Change live contact processing while a peer is busy; query recording status.
- Wait through a synchronous call after an uncollected step.
- Open, fetch, commit and release debug-render sessions on an unattached instance
  while a peer is held inside detach.
- Open a debug-render read snapshot without an attachment, attach its instance,
  then fetch/release the snapshot while a peer detach is held.

## Then

- Caller execution waits for preceding work without rejecting it as runtime busy;
  output submissions queue as specified by REQ-CAPI-ASYNC-002 (REQ AC-1, AC-2).
- Independent-owner submission makes progress (REQ AC-1).
- Unattached debug-render sessions complete before the peer detach is released.
  A read snapshot opened unattached remains usable after its instance attaches,
  without waiting for the peer lifecycle operation (REQ AC-1).
- The synchronous drain preserves task outcomes for collection (REQ AC-3).
- Process-wide mutation waits for the peer, while recording-status reads do not
  (REQ AC-4).

## Automated coverage

- `Lanes.ShortCallWaitsBehindWorkerJob`, `OutputNotRejectedWhileWorkerRuns`,
  `LifecycleSubmissionWaitsForNoPeerLifecycleJob`, `ProcessWideWaitsForPeerWorker`,
  `RecordingStatusWaitsForNoInstanceWork`,
  `RecordingTransitionsWaitingForWorkRefuseNoOtherTransition`.
- `Lanes.UnattachedInstanceOutputRunsWhileAPeerDetaches` and
  `UnattachedReadSnapshotProgressesAfterAttach`: unattached debug-render sessions
  and retained host snapshots progress during peer detach.
- `WorkerStep.SyncDrainDoesNotCollect` in `tests/c_unittests/test_worker_step.cpp`.
