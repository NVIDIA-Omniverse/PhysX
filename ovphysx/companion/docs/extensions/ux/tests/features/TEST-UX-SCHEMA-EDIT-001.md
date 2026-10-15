<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-SCHEMA-EDIT-001
maps_to: REQ-UX-SCHEMA-EDIT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Apply or clear an explicitly scoped physics preset with exact undo. Verifies REQ-UX-SCHEMA-EDIT-001 AC-1, AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Selected subtree with referenced physics APIs, multiple-apply tendon instances, geometry, another engine API and an unselected sibling.

## When

- Parameterize preset apply/remove, generic Clear Physics and PhysxSchema-only removal; inspect scope, execute, undo/redo; repeat against a read-only target.

## Then

- Preset changes only applicable intended APIs and preserves the other engine API. (REQ-UX-SCHEMA-EDIT-001 AC-1).
- Clear/removal matches the displayed scope; PhysxSchema-only removal retains generic PhysicsSchema data and geometry; the sibling remains unchanged. (REQ-UX-SCHEMA-EDIT-001 AC-2).
- One undo restores exact composed authored opinions and every multiple-apply instance; read-only rejection produces no partial edits. (REQ-UX-SCHEMA-EDIT-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-SCHEMA-EDIT-001](../../requirements/features/REQ-UX-SCHEMA-EDIT-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.


