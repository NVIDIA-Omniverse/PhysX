<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-DEMOS-002
maps_to: REQ-UX-DEMOS-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Restore demo-owned state after close or launch failure. Verifies REQ-UX-DEMOS-001 AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A demo with temporary settings/subscriptions and unrelated user/provider sentinels.

## When

- Parameterize normal close and failure during launch/Play, then deliver a late demo callback.

## Then

- Only demo-owned settings/subscriptions are restored/removed, live resources follow Stop/detach and no hidden step task or late mutation remains. (REQ-UX-DEMOS-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-DEMOS-001](../../requirements/features/REQ-UX-DEMOS-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

