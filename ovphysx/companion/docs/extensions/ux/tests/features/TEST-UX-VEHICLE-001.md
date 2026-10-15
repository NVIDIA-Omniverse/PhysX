<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-VEHICLE-001
maps_to: REQ-UX-VEHICLE-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Keep vehicle authoring usable without live vehicle service. Verifies REQ-UX-VEHICLE-001 AC-1, AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A writable vehicle schema fixture, valid friction materials and no live vehicle capability.

## When

- Create vehicle/table through their authoring tools, edit wheel/tire fields, undo/redo, reload the property contributor and attempt drive controls.

## Then

- Authoring validates targets, undoes exactly and registers each widget once. (REQ-UX-VEHICLE-001 AC-1).
- Live controls show unavailable without attaching simulation or installing another schedule. (REQ-UX-VEHICLE-001 AC-2).

- **Complete authoring.** Retain wizard geometry/axes/unit choices, wheel/suspension/tire/friction-table authoring, drive modes, nonlinear response tables and multi-wheel targeting. Suspension data helpers alone do not satisfy the wizard; every committed group must undo to exact prior opinions. (REQ AC-4).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-VEHICLE-001](../../requirements/features/REQ-UX-VEHICLE-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.
