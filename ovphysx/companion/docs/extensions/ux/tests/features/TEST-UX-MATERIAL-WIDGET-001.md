<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-MATERIAL-WIDGET-001
maps_to: REQ-UX-MATERIAL-WIDGET-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Inspect inconsistent compliant data without rewriting it. Verifies REQ-UX-MATERIAL-WIDGET-001 AC-1, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A material with stiffness=0, damping=5, acceleration-spring=true and shared USD friction values, plus an inactive-engine variant.

## When

- Open/rebuild the widget twice and change selection.

## Then

- All authored values and shared material fields remain exact. (REQ-UX-MATERIAL-WIDGET-001 AC-1).
- Dependent controls reflect zero stiffness while retaining stored damping/spring values until an explicit edit. (REQ-UX-MATERIAL-WIDGET-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-MATERIAL-WIDGET-001](../../requirements/features/REQ-UX-MATERIAL-WIDGET-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

