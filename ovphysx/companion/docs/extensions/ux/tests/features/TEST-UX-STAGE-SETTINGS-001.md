<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-STAGE-SETTINGS-001
maps_to: REQ-UX-STAGE-SETTINGS-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Round-trip stage overrides without leaking them to the next stage. Verifies REQ-UX-STAGE-SETTINGS-001 AC-1, AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Stages A and B have different physics metadata; A has a writable edit target and B contains absent overrides with known defaults.

## When

- Edit A stage settings, save/reopen A, reset one override, then switch to B and reopen the panel.

## Then

- Only A stage metadata stores A overrides and save/reopen recovers requested/effective values. (REQ-UX-STAGE-SETTINGS-001 AC-1).
- Reset removes the intended override and reveals its fallback without resetting global preferences or other metadata. (REQ-UX-STAGE-SETTINGS-001 AC-2).
- B displays its own authored values/defaults and receives no A write or stale UI value. (REQ-UX-STAGE-SETTINGS-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-STAGE-SETTINGS-001](../../requirements/features/REQ-UX-STAGE-SETTINGS-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.


