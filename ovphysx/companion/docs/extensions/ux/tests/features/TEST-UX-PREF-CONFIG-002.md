<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-PREF-CONFIG-002
maps_to: REQ-UX-PREF-CONFIG-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Keep authoring preferences local and retire default simulator selection. Verifies REQ-UX-PREF-CONFIG-001 AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Two registered engines, authoring toggles and scan limits, plus a stored legacy default-simulator=None.

## When

- Exercise scan limit 0/uint32 max and invalid -1/overflow; toggle authoring policies and reload the authoring provider; reopen preferences.

## Then

- Only valid limits persist; caps skip provider scans without blocking generic commands; authoring policies preserve schemas and follow their documented reload boundary. No setting is sent to ovphysx. (REQ-UX-PREF-CONFIG-001 AC-2).
- No legacy default-simulator control appears and its stored key does not alter Umbrella activation. (REQ-UX-PREF-CONFIG-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-PREF-CONFIG-001](../../requirements/features/REQ-UX-PREF-CONFIG-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

