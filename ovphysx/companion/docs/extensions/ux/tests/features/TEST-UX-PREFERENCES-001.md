<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-PREFERENCES-001
maps_to: REQ-UX-PREFERENCES-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Build and reset only the audited preference model. Verifies REQ-UX-PREFERENCES-001 AC-1, AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Stored supported overrides, all retired legacy keys including reset-on-stop=false, unrelated provider preferences and stage metadata.

## When

- Open/rebuild preferences, reset, close/reopen, then Play/Stop with nonzero initial velocities.

## Then

- Only the reviewed mapped or UX-owned controls appear; retired values are not emitted as Carbonite config. (REQ-UX-PREFERENCES-001 AC-1).
- Reset clears supported owned overrides/pending requests while preserving unrelated and stage data; bootstrapped effective values remain restart-pending when appropriate. (REQ-UX-PREFERENCES-001 AC-2).
- There is no reset-on-stop toggle and legacy false has no effect: Stop restores initial pose/velocities. (REQ-UX-PREFERENCES-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-PREFERENCES-001](../../requirements/features/REQ-UX-PREFERENCES-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

