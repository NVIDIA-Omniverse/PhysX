<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-MENU-BASIC-002
maps_to: REQ-UX-MENU-BASIC-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Reject creation without a writable stage. Verifies REQ-UX-MENU-BASIC-001 AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- No stage, a read-only edit target and an inapplicable request as separate fixtures.

## When

- Invoke basic creation commands.

## Then

- No-stage/read-only/inapplicable requests produce no prim, opinion or undo entry and explain why. (REQ-UX-MENU-BASIC-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-MENU-BASIC-001](../../requirements/features/REQ-UX-MENU-BASIC-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

