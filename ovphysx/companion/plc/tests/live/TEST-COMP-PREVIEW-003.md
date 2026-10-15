<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-PREVIEW-003
maps_to: REQ-COMP-PREVIEW-001
type: integration
---

## Coverage status

Native CPU integration executed in [test_preview.py](../../../tests_native/test_preview.py). Covers
[REQ-COMP-PREVIEW-001](../../requirements/live/REQ-COMP-PREVIEW-001.md)
AC-1 and AC-7.

## Scenario

Retire mixed core-playback and companion-preview owners in both orders while
retaining peer work, views and a shared recording session.

## Given

- One public ovphysx instance owns playback A and the companion bridge owns
  native preview B, with distinct ovstage sources and shared runtime services.
- Per-owner views/subscriptions, a pending job and a process recording session
  controlled by the host coordinator; source lifetime and SDK-release counters.
- A core peer C with a third distinct ovstage source for checking public attachment coexistence.

## When

- Reject C's attach to B's already-owned source, then attach C to its own
  source while A and B live. Retire C and continue A and B.
- Retire B while A has pending work, then fetch/step/read A and query recording.
  Repeat with A retired first, explicitly coordinating its facade recording
  finalization policy before transition, and continue B afterward.
- Request recording transitions while an owner is pending and retry after
  completion. Request process shutdown through the coordinator with owners or
  external leases remaining, then finish their orderly retirement.

## Then

- C attaches its distinct source without displacing A or B. Duplicate-source
  acquisition remains rejected and retiring C preserves A and B. (REQ AC-1)
- Retiring B does not stop A's capture or release A's SDK resources; retiring A
  follows the explicit recording policy without invalidating B. Both survivors
  can step/read. Per-owner teardown drains callbacks/jobs and invalidates only
  its own views; source buffers survive through detach. (REQ AC-7)
- Shared recording transitions return Busy while required owners are pending.
  Coordinated final shutdown rejects premature release or drains all users;
  process services outlive both owners, candidates and leases, then release
  exactly once. No claim of an automatic existing core shutdown guard is used
  to satisfy this test. (REQ AC-7)

## Remaining coverage

GPU/device execution is not qualified or advertised by the CPU host adapter.
Owner-local queries/events/views and recording lifetimes are covered by the
inspection, events, debug, OVD and interaction suites. Customer-USD output and
Play/cancel transitions are covered by TEST-COMP-PREVIEW-001.

## Executable coverage

- [Executed subset cases](../../../tests_native/test_preview.py). Remaining scenario obligations are listed above.

## Review regressions

The fresh-process inspector sample destroys an unused unattached core while the
preview is pending, including when playback owns recording. Parallel independent
preview construction and queue resolution exercise attachment publication; this
is an integration stress test, not a claimed sanitizer reproduction.

With scoped OVD capability enabled, a core owner attaches and remains dormant.
A preview attachment does not start recording. After the core explicitly starts
and stops a session, another preview attachment does not restart it. The core
can start and finalize another FILE session
([test_ovd.py](../../../tests_native/test_ovd.py),
`test_preview_attach_does_not_start_or_restart_recording`). (REQ AC-7)

A preview and its Stage left in an unreachable cycle without explicit close are
collected with a fake native preview table: the native preview closes before the
Stage finalizer runs, so the pinned Stage outlives native close
([test_preview.py](../../../tests_native/test_preview.py),
`test_cyclic_gc_closes_preview_before_its_stage_is_finalized`). (REQ AC-7)
