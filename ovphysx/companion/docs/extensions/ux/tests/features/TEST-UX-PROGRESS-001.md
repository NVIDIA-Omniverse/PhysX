<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-PROGRESS-001
maps_to: REQ-UX-PROGRESS-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Track concurrent operation progress and terminal states. Verifies REQ-UX-PROGRESS-001 AC-1, AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Two operations A/B with distinct IDs in the same stage and one other-provider operation C.

## When

- Interleave progress including -0.2, 1.4 and unknown/nonfinite values, finish A, fail B with known text, cancel C; repeat with stage close/provider unload before late progress.

## Then

- Each operation retains its own identity/label/state and never overwrites another. (REQ-UX-PROGRESS-001 AC-1).
- Finite fractions clamp to 0/1, unknown/nonfinite progress is indeterminate and the exact failure text is exposed. (REQ-UX-PROGRESS-001 AC-2).
- Only the matching terminal overlay is removed, failures remain observable, and stale progress after teardown cannot recreate it. (REQ-UX-PROGRESS-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-PROGRESS-001](../../requirements/features/REQ-UX-PROGRESS-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.
