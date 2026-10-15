<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-MOUSE-002
maps_to: REQ-UX-MOUSE-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Cancel interaction on interruption without starting stopped physics. Verifies REQ-UX-MOUSE-001 AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A live owned drag with recorded transient resources and a pending feedback result.

## When

- Parameterize focus loss, input disable, Pause, Stop, selection replacement, stage replacement and provider removal; deliver the old result and attempt another stopped drag.

## Then

- Old-generation markers do not reappear. (REQ-UX-MOUSE-001 AC-2).
- Each interruption sends one cancel before releasing UI ownership and removes transient resources; stopped drag attempts make zero create/attach/query calls. (REQ-UX-MOUSE-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-MOUSE-001](../../requirements/features/REQ-UX-MOUSE-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.
