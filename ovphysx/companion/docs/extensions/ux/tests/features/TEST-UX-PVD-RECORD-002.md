<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-PVD-RECORD-002
maps_to: REQ-UX-PVD-RECORD-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Keep legacy PVD separate from OmniPVD. Verifies REQ-UX-PVD-RECORD-001 AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- OmniPVD available but legacy PVD absent, plus an explicit legacy-service fixture with failing network connection.

## When

- Open network/file controls, attempt connection/reconnection and change recording settings during a live session.

## Then

- Absent legacy capability is unavailable rather than routed to OmniPVD; connection failure is visible. (REQ-UX-PVD-RECORD-001 AC-3).
- Live setting changes remain deferred or rejected at the declared boundary and do not silently restart/destroy a recorder. (REQ-UX-PVD-RECORD-001 AC-2).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-PVD-RECORD-001](../../requirements/features/REQ-UX-PVD-RECORD-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

