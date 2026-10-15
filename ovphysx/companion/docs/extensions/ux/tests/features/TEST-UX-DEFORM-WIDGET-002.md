<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-DEFORM-WIDGET-002
maps_to: REQ-UX-DEFORM-WIDGET-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Refresh once after explicit computation and discard stale jobs. Verifies REQ-UX-DEFORM-WIDGET-001 AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- An advertised mesh/attachment/filter computation service with a delayed operation scoped to a selected prim.

## When

- Request each supported computation in separate fixtures; complete successfully, fail, or replace stage/selection before completion.

## Then

- Only deliberate requests start the scoped operation. (REQ-UX-DEFORM-WIDGET-001 AC-2).
- Success refreshes once; failure reports its error; stale completion cannot author or refresh the replacement target. (REQ-UX-DEFORM-WIDGET-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-DEFORM-WIDGET-001](../../requirements/features/REQ-UX-DEFORM-WIDGET-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

