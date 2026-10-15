<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-VIEWPORT-001
maps_to: REQ-UX-VIEWPORT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Share display state across menu and provider reloads. Verifies REQ-UX-VIEWPORT-001 AC-1, AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Two engine display contributors, generic joint visibility and a writable shared display model.

## When

- Open/reopen the viewport menu, change None/Selected/All through another control, reopen, reload one contributor and unload it.

## Then

- There is one Physics category and one entry per engine/feature key; generic joints still work without PhysX. (REQ-UX-VIEWPORT-001 AC-1).
- The menu reflects each externally changed effective value and edits the same model. (REQ-UX-VIEWPORT-001 AC-2).
- Unloaded provider listeners/windows disappear while the other contributor remains. (REQ-UX-VIEWPORT-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-VIEWPORT-001](../../requirements/features/REQ-UX-VIEWPORT-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

