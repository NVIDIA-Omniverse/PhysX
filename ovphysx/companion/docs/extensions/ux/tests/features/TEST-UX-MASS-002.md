<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-MASS-002
maps_to: REQ-UX-MASS-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Retain authoring when computed data is unavailable or stale. Verifies REQ-UX-MASS-001 AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A body with authored MassAPI, no query capability in one fixture and a delayed query in another.

## When

- Open mass UI; for delayed query replace selection/stage/provider or delete target before completion.

## Then

- Authored values remain editable with an unavailable computed-data state; no force load occurs and stale completion cannot draw helpers or update a new body. (REQ-UX-MASS-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-MASS-001](../../requirements/features/REQ-UX-MASS-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

