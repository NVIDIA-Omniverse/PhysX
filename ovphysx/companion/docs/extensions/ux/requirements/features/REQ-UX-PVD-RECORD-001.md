<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-PVD-RECORD-001
title: Configure recording through its owning PhysX service
status: proposed
owner: physics-ux
---

## Description

Configure recording through its owning PhysX service. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Recording lifecycle.** OmniPVD Enabled/Directory shall configure a supported owning service without creating simulation on panel open; recorder start/failure shall be distinguished from requested enablement, with destination creation/error feedback.
- AC-2: **Finalization.** Stop/detach shall flush and close any active recorder before its simulation resources disappear; active changes shall follow the service's declared boundary and failures shall not overwrite or destroy source recordings.
- AC-3: **Legacy PVD.** Legacy PVD connection/network/file controls shall be separately capability-gated, report connection/reconnection failures and never imply OmniPVD compatibility or start unsupported live services.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-PVD-RECORD-001](../../tests/features/TEST-UX-PVD-RECORD-001.md) — Configure then finalize a supported recording with the session.
- [TEST-UX-PVD-RECORD-002](../../tests/features/TEST-UX-PVD-RECORD-002.md) — Keep legacy PVD separate from OmniPVD.

## Code References

- [ovd](../../../../../python/ovphysx_companion/ovd.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0008](../../adr/ADR-0008-debug-window.md).
- [ADR-0016](../../adr/ADR-0016-omnipvd.md).
- [REQ-RT-PLAY-001](../../../runtime/requirements/simulation/REQ-RT-PLAY-001.md).
- [REQ-RT-STOP-001](../../../runtime/requirements/simulation/REQ-RT-STOP-001.md).
- [REQ-RT-DETACH-001](../../../runtime/requirements/simulation/REQ-RT-DETACH-001.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
