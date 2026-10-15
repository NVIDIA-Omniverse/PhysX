<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-RT-FAULT-001
title: Fail closed without repeating or leaking partial simulation work
status: proposed
owner: physics-runtime
---

## Description

Fail closed without repeating or leaking partial simulation work. This is a runtime-extension adapter obligation for the single-scene ovphysx/ovstage integration, derived from the reviewed ADRs. The adapter migration is not implemented by these records.

## Acceptance Criteria

- AC-1: **Failed Play.** A failed resolve/capture/create/attach/initialization shall destroy all safely releasable acquired play resources and return to Stopped only after cleanup/restoration completes; otherwise retain only cleanup-owned resources in Faulted. It shall never step a partial session or displace another owner.
- AC-2: **Failed step/publication.** After a step or publication error, mark Faulted, block later simulation/output and never retry the partly applied interval. Count only confirmed completed steps; emit no provider-success/presentation-ready notification merely because timeline intent or backend completion was broadcast.
- AC-3: **Failed Stop/close.** Teardown/restoration failure shall block Play, retain only resources needed to finish cleanup/restoration and identify the failed phase; it shall not report Stopped/Detached success, free in-use host data or recreate simulation as a recovery shortcut.

## Test References

Specification only — the mapped adapter tests are not implemented yet; legacy/API cases below are inspiration, not claimed adapter coverage.

- [TEST-RT-FAULT-001](../../tests/simulation/TEST-RT-FAULT-001.md) — Play failure at each initialization boundary leaves no hidden session.
- [TEST-RT-FAULT-002](../../tests/simulation/TEST-RT-FAULT-002.md) — A failed step or writeback cannot replay a partially applied interval.
- [TEST-RT-FAULT-003](../../tests/simulation/TEST-RT-FAULT-003.md) — Failed teardown or restoration retains only recoverable ownership.

## Code References

- [Current ovphysx lifecycle API](../../../../../../include/ovphysx/ovphysx.h) — playback implementation substrate, not completed Kit adapter coverage.
- [ovstage integration](../../../../../../docs/ovstage_integration.md) — input/output ownership and synchronization.


Existing integration boundaries and source behavior to adapt; these references do not claim the new adapter already implements this requirement. New implementation paths and source annotations must be recorded when that code lands.

- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — Current attach/update/detach/step/read/query API and ownership contracts (current API reference).
- [test_step_and_write_to_ovstage.py](../../../../../../tests/python_tests/test_step_and_write_to_ovstage.py) — Utility tests for world matrices, instancer holes, shadow arrays, write failures and cache invalidation (current API reference).
- [test_lifecycle_failure_injection.py](../../../../../../tests/python_tests/test_lifecycle_failure_injection.py) — Native failure, retained ownership and terminal lifecycle cases (current API reference).

## Dependencies

- [ADR-0001](../../adr/ADR-0001-ovphysx-stageupdate-lifecycle.md).
- [ADR-0004](../../adr/ADR-0004-start-resume-state-capture.md).
- [ADR-0005](../../adr/ADR-0005-simulation-update-and-substepping.md).
- [ADR-0007](../../adr/ADR-0007-stop-and-state-restoration.md).
- [ADR-0008](../../adr/ADR-0008-ovstage-detach.md).
- [ADR-0010](../../adr/ADR-0010-ovstage-simulation-output.md).
- [REQ-RT-DRIVER-001](REQ-RT-DRIVER-001.md).
- [REQ-RT-PLAY-001](REQ-RT-PLAY-001.md).
- [REQ-RT-STOP-001](REQ-RT-STOP-001.md).
- [REQ-RT-DETACH-001](REQ-RT-DETACH-001.md).
