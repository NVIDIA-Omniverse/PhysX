<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-RT-PLAY-001
title: Load the current live ovstage on Play
status: proposed
owner: physics-runtime
---

## Description

Load the current live ovstage on Play. This is a runtime-extension adapter obligation for the single-scene ovphysx/ovstage integration, derived from the reviewed ADRs. The adapter migration is not implemented by these records.

## Acceptance Criteria

- AC-1: **Fresh loading.** Start/onResume from Stopped shall resolve the remembered live stage, capture its current initial state, create one fresh ovphysx instance and call ovphysx_attach_ovstage once with a nonzero sealed input ordinal containing edits made before Play; no filename reload is permitted.
- AC-2: **Ownership.** Play shall retain the host ovstage throughout the play session, respect the process-wide attached-source owner, and enter Running only after capture, timestep validation, attach and initialization succeed; the initial ordinal shall not be drained again.
- AC-3: **No initial step.** Play shall create a fresh simulation generation with zero completed steps and zero remainder, without a dynamics/warm-up step or output publication; enabled updates alone advance simulation.
- AC-4: **Entry conditions.** Start while Running shall change nothing; start from Paused shall resume the existing session without reload or recapture; start with no remembered or resolvable live stage shall fail without loading another stage.

## Test References

Specification only — the mapped adapter tests are not implemented yet; legacy/API cases below are inspiration, not claimed adapter coverage.

- [TEST-RT-PLAY-001](../../tests/simulation/TEST-RT-PLAY-001.md) — First Play loads the latest sealed contents and does not step.
- [TEST-RT-PLAY-002](../../tests/simulation/TEST-RT-PLAY-002.md) — Play rejects missing stage identity without fallback loading.
- [TEST-RT-PLAY-003](../../tests/simulation/TEST-RT-PLAY-003.md) — The play epoch retains the borrowed ovstage until teardown.

- [TEST-RT-PLAY-004](../../tests/simulation/TEST-RT-PLAY-004.md) — Play cannot displace the existing backend owner.

## Code References

- [Current ovphysx lifecycle API](../../../../../../include/ovphysx/ovphysx.h) — playback implementation substrate, not completed Kit adapter coverage.
- [ovstage integration](../../../../../../docs/ovstage_integration.md) — input/output ownership and synchronization.


Existing integration boundaries and source behavior to adapt; these references do not claim the new adapter already implements this requirement. New implementation paths and source annotations must be recorded when that code lands.

- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — Current attach/update/detach/step/read/query API and ownership contracts (current API reference).
- [test_usd_loading.cpp](../../../../../../tests/c_unittests/test_usd_loading.cpp) — AttachOvstageTwiceRejected, UnsealedAttachFailsAndSealedRetrySucceeds, AttachOvstageRejectsZeroReadOrdinal, DetachWithoutStepDropsPendingArticulations (current API reference).

## Dependencies

- [ADR-0004](../../adr/ADR-0004-start-resume-state-capture.md).
- [REQ-RT-STAGE-001](REQ-RT-STAGE-001.md).
- [REQ-RT-SNAPSHOT-001](REQ-RT-SNAPSHOT-001.md).
- [REQ-RT-STEP-001](REQ-RT-STEP-001.md).
