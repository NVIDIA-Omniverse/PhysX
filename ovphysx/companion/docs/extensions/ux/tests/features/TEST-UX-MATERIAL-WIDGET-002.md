<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-MATERIAL-WIDGET-002
maps_to: REQ-UX-MATERIAL-WIDGET-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Undo zero-stiffness dependency edits atomically. Verifies REQ-UX-MATERIAL-WIDGET-001 AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A writable material with stiffness=100, damping=5 and acceleration-spring=true; sentinel restitution/friction.

## When

- Set stiffness to zero deliberately, undo/redo, then attempt nonfinite/schema-invalid inputs.

## Then

- The single edit stores stiffness=0, damping=0 and acceleration-spring=false; one undo restores 100/5/true. (REQ-UX-MATERIAL-WIDGET-001 AC-2).
- Invalid input writes nothing; unrelated restitution/friction remain unchanged and dependent enablement matches the effective stiffness. (REQ-UX-MATERIAL-WIDGET-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-MATERIAL-WIDGET-001](../../requirements/features/REQ-UX-MATERIAL-WIDGET-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

