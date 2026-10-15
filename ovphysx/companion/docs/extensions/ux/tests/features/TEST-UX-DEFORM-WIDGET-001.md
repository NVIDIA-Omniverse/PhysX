<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-DEFORM-WIDGET-001
maps_to: REQ-UX-DEFORM-WIDGET-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Edit deformable properties without implicit cooking. Verifies REQ-UX-DEFORM-WIDGET-001 AC-1, AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Valid body/material/simulation-mesh/attachment prims, explicit stage units and no optional compute service.

## When

- Open/rebuild widgets, edit schema values/relationships and undo/redo; invoke absent compute actions.

## Then

- Only applicable fields are authored in the active target with correct units and exact undo. (REQ-UX-DEFORM-WIDGET-001 AC-1).
- Inspection starts no cooking, absent computation is unavailable and authored fields stay visible. (REQ-UX-DEFORM-WIDGET-001 AC-2).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-DEFORM-WIDGET-001](../../requirements/features/REQ-UX-DEFORM-WIDGET-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.


