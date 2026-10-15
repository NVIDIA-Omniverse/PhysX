<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-UX-DEFORM-WIDGET-001
title: Author deformable data without implicit computation
status: proposed
owner: physics-ux
---

## Description

Author deformable data without implicit computation. This is a proposed UX integration obligation derived from the ADRs below, aligned with the reviewed single-scene runtime and separate authoring lifecycle. It does not claim the migrated implementation exists.

## Companion integration

Use the public companion modules named under Code References. Customer USD edits stay on the host Python thread under EditSession; C++ receives source-neutral data only. The supported CPU service subset does not establish complete legacy feature parity. See [workflow coverage](../../WORKFLOW-COVERAGE.md) for retained behavior, missing providers and host work.

## Acceptance Criteria

- AC-1: **Schema editing.** Body, surface/volume material, simulation-mesh and attachment fields shall obey schema applicability/stage units, author through the active edit target and undo values/relationships exactly.
- AC-2: **Explicit computation.** Mesh generation/attachment computation/element-filter actions shall require an advertised authoring service and remain bound to initiating stage/prim; panel inspection shall start no cooking and missing service shall not hide authored fields.
- AC-3: **Completion.** A completed supported edit shall refresh its preview once; failed/stale completion shall report/discard appropriately and never edit a replacement stage.

## Test References

Specification only — these migrated UX scenarios are not implemented. Legacy cases are starting points, not claimed coverage.

- [TEST-UX-DEFORM-WIDGET-001](../../tests/features/TEST-UX-DEFORM-WIDGET-001.md) — Edit deformable properties without implicit cooking.
- [TEST-UX-DEFORM-WIDGET-002](../../tests/features/TEST-UX-DEFORM-WIDGET-002.md) — Refresh once after explicit computation and discard stale jobs.

## Code References

- [usd/deformables](../../../../../python/ovphysx_companion/usd/deformables.py) — existing companion substrate; host integration remains proposed.
- [usd/attachments](../../../../../python/ovphysx_companion/usd/attachments.py) — existing companion substrate; host integration remains proposed.
- [deformable_cooking](../../../../../python/ovphysx_companion/deformable_cooking.py) — existing companion substrate; host integration remains proposed.

Inspected release sources to adapt; current ovphysx APIs are separately pinned where used. These anchors do not imply the proposed provider/hook or automated case exists. Add implementation locations and source annotations when it lands.


## Dependencies

- [ADR-0021](../../adr/ADR-0021-deformable-properties.md).
- [REQ-UX-HOST-001](REQ-UX-HOST-001.md).
- [REQ-UX-PROGRESS-001](REQ-UX-PROGRESS-001.md).
