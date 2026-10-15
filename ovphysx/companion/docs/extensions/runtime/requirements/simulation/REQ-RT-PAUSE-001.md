<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-RT-PAUSE-001
title: Preserve the play session while paused
status: proposed
owner: physics-runtime
---

## Description

Preserve the play session while paused. This is a runtime-extension adapter obligation for the single-scene ovphysx/ovstage integration, derived from the reviewed ADRs. The adapter migration is not implemented by these records.

## Acceptance Criteria

- AC-1: **Completion boundary.** Pause shall stop new substeps, finish the current submitted step/publication and cancel interaction with temporary-property cleanup before entering Paused; a reentrant pause shall take effect at a safe completed-substep boundary.
- AC-2: **Preservation.** Paused state shall retain the same instance, backend attachment, original snapshot, solver state/time, remainder and valid output resources; paused updates shall not step, drain editor changes or publish new simulation output.
- AC-3: **Resume.** Resume shall continue the same epoch with only resumed elapsed time; no new instance, attach, snapshot or catch-up for paused wall time is permitted, and canceled mouse input requires a fresh press.
- AC-4: **Idempotence.** Repeated pause shall have no further effect; pause from Stopped/Detached shall allocate nothing and shall not create an epoch.

## Test References

Specification only — the mapped adapter tests are not implemented yet; legacy/API cases below are inspiration, not claimed adapter coverage.

- [TEST-RT-PAUSE-001](../../tests/simulation/TEST-RT-PAUSE-001.md) — Pause and resume preserve instance, baseline and fractional time.

## Code References

- [Current ovphysx lifecycle API](../../../../../../include/ovphysx/ovphysx.h) — playback implementation substrate, not completed Kit adapter coverage.
- [ovstage integration](../../../../../../docs/ovstage_integration.md) — input/output ownership and synchronization.


Existing integration boundaries and source behavior to adapt; these references do not claim the new adapter already implements this requirement. New implementation paths and source annotations must be recorded when that code lands.


## Dependencies

- [ADR-0006](../../adr/ADR-0006-simulation-pause.md).
- [REQ-RT-PLAY-001](REQ-RT-PLAY-001.md).
- [REQ-RT-STEP-001](REQ-RT-STEP-001.md).
