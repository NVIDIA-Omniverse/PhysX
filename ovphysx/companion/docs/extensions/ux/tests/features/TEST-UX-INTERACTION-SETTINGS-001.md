<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-INTERACTION-SETTINGS-001
maps_to: REQ-UX-INTERACTION-SETTINGS-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Use validated shared grab and visualization settings. Verifies REQ-UX-INTERACTION-SETTINGS-001 AC-1, AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A running drag and a shared settings model used by mouse/debug/viewport controls.

## When

- Change interaction enablement, exercise grab coefficients 0/10 and invalid -1/11, push 0/1000 and invalid -1/1001, each advertised mode, and visualization distance 1/10000 with invalid 0/10001.

## Then

- Disabling interaction cancels the owned drag once and blocks new drags. (REQ-UX-INTERACTION-SETTINGS-001 AC-1).
- Valid grab/push choices reach the shared provider model; invalid/nonfinite values leave prior values unchanged; unsupported modes show unavailable. (REQ-UX-INTERACTION-SETTINGS-001 AC-2).
- Distance controls agree on the bounded effective value and affect the same overlay threshold. (REQ-UX-INTERACTION-SETTINGS-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-INTERACTION-SETTINGS-001](../../requirements/features/REQ-UX-INTERACTION-SETTINGS-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

