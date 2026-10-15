<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-MENU-MATERIAL-001
maps_to: REQ-UX-MENU-MATERIAL-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Commit only selected material APIs. Verifies REQ-UX-MENU-MATERIAL-001 AC-1, AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A writable stage, a pre-existing material and a provider advertising a subset of rigid/deformable/surface/PBD material capabilities.

## When

- Parameterize each supported type and supported combination: confirm once, undo/redo, then cancel another dialog; attempt an unsupported combination.

## Then

- Supported choices match capabilities; unsupported combinations explain the missing support before any write. (REQ-UX-MENU-MATERIAL-001 AC-1).
- Confirm authors exactly the chosen APIs at a unique path in one undo group; cancel and rejection leave layers and history unchanged, including the existing material. (REQ-UX-MENU-MATERIAL-001 AC-2).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-MENU-MATERIAL-001](../../requirements/features/REQ-UX-MENU-MATERIAL-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

