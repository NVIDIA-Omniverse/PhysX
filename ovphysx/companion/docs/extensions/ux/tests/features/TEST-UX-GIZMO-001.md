<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-GIZMO-001
maps_to: REQ-UX-GIZMO-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Author a stopped transform drag in the correct local frame. Verifies REQ-UX-GIZMO-001 AC-1.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A stopped rigid prim under a rotated/nonuniformly scaled parent with a known world pivot and one locked translation axis.

## When

- Drag the gizmo along a known world delta, finish, undo/redo.

## Then

- Composed world transform matches the allowed delta/pivot within fixture tolerance, the locked axis is unchanged, one undo restores the original xform opinions and no physics session is created. (REQ-UX-GIZMO-001 AC-1).

- **Rotation and locks.** Retained rotational/translation gizmos shall implement their declared world pivot, snapping and axis-lock behavior under positive motion/orientation tests; unsupported rotational targets shall remain an explicit gap rather than silently using point dragging. (REQ AC-4).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-GIZMO-001](../../requirements/features/REQ-UX-GIZMO-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

