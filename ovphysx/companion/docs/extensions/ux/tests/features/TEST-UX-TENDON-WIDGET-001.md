<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-TENDON-WIDGET-001
maps_to: REQ-UX-TENDON-WIDGET-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Edit one tendon instance and release only its helpers. Verifies REQ-UX-TENDON-WIDGET-001 AC-1, AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A prim with two fixed/spatial multiple-apply tendon instances and inherited schema data, with and without visualization support.

## When

- Select one instance, edit stiffness/damping/limits/attachments, undo/redo, deselect/unload and deliver a stale helper callback.

## Then

- Only the selected instance changes and undo restores exact identity/data; inherited and other-instance values remain intact. (REQ-UX-TENDON-WIDGET-001 AC-1).
- Missing visualization disables helpers only; schema inspection/editing works without runtime loading. (REQ-UX-TENDON-WIDGET-001 AC-2).
- Teardown removes only owned helpers/filters and ignores the late callback. (REQ-UX-TENDON-WIDGET-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-TENDON-WIDGET-001](../../requirements/features/REQ-UX-TENDON-WIDGET-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

