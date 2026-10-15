<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-PVD-RECORD-001
maps_to: REQ-UX-PVD-RECORD-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Configure then finalize a supported recording with the session. Verifies REQ-UX-PVD-RECORD-001 AC-1, AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A stopped stage, explicit recorder service and writable nonexistent nested destination; known recorder/session resource counters.

## When

- Open settings, enable recording, Play/update then Stop; repeat detach as the termination and inject directory/writer/finalization failures in separate fixture cases.

## Then

- Opening/configuring does not create simulation; start acknowledges a created destination or exposes failure separately from requested enablement. (REQ-UX-PVD-RECORD-001 AC-1).
- Active recording flush/close precedes session destruction; failure is surfaced and existing source recordings are preserved. (REQ-UX-PVD-RECORD-001 AC-2).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-PVD-RECORD-001](../../requirements/features/REQ-UX-PVD-RECORD-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

