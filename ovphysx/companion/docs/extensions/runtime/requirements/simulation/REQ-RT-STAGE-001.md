<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-RT-STAGE-001
title: Remember a stage without loading physics
status: proposed
owner: physics-runtime
---

## Description

Remember a stage without loading physics. This is a runtime-extension adapter obligation for the single-scene ovphysx/ovstage integration, derived from the reviewed ADRs. The adapter migration is not implemented by these records.

## Acceptance Criteria

- AC-1: **Stage open.** onAttach shall enter Stopped with only host-stage identity/reference and lifecycle bookkeeping; instance creation, backend attach, parsing/cooking, snapshot capture, input drain and output allocation counts shall remain zero.
- AC-2: **Identity.** Duplicate attach for the same host-stage generation shall be a no-op, including while Running/Paused; replacement shall finish the old stage teardown before remembering the new stage in Stopped.
- AC-3: **Stopped lifetime.** Closing a never-played or already-stopped stage shall clear its identity/reference without creating a backend instance or calling backend detach; repeated close shall be harmless.

## Test References

Specification only — the mapped adapter tests are not implemented yet; legacy/API cases below are inspiration, not claimed adapter coverage.

- [TEST-RT-STAGE-001](../../tests/simulation/TEST-RT-STAGE-001.md) — Open and close an unplayed stage without allocating physics.
- [TEST-RT-STAGE-002](../../tests/simulation/TEST-RT-STAGE-002.md) — Stage replacement drains the old session before remembering the new stage.

## Code References

- [Current ovphysx lifecycle API](../../../../../../include/ovphysx/ovphysx.h) — playback implementation substrate, not completed Kit adapter coverage.
- [ovstage integration](../../../../../../docs/ovstage_integration.md) — input/output ownership and synchronization.


Existing integration boundaries and source behavior to adapt; these references do not claim the new adapter already implements this requirement. New implementation paths and source annotations must be recorded when that code lands.


## Dependencies

- [ADR-0003](../../adr/ADR-0003-ovstage-attach.md).
- [REQ-RT-DRIVER-001](REQ-RT-DRIVER-001.md).
