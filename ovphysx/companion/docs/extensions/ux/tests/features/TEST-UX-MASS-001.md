<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-MASS-001
maps_to: REQ-UX-MASS-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Show computed mass separately and undo authored overrides. Verifies REQ-UX-MASS-001 AC-1, AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A body under a rotated parent with explicit stage units, authored MassAPI overrides and a supported query returning deliberately different computed values.

## When

- Select it, complete its query, edit mass/COM/inertia/principal axes in one drag, undo/redo; attempt NaN and schema-invalid mass/inertia.

## Then

- Authored and computed values are labeled distinctly and use independently calculated display-frame/unit conversions. (REQ-UX-MASS-001 AC-1).
- One undo restores all edited opinions and preserves unrelated data; nonfinite/invalid edits add no writes/history. (REQ-UX-MASS-001 AC-2).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-MASS-001](../../requirements/features/REQ-UX-MASS-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.


