<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-PVD-VIEW-001
maps_to: REQ-UX-PVD-VIEW-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Seek recorded data with no live physics backend. Verifies REQ-UX-PVD-VIEW-001 AC-1, AC-2, AC-3.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A recording with two frames containing distinct contact/COM/joint/velocity values, another recording with overlapping object IDs, and no live session.

## When

- Import, select a recorded prim, seek both frames, switch recording, then close the viewer.

## Then

- Source bytes remain unchanged, recording/frame identity is independent of live handles and backend create/step counts stay zero. (REQ-UX-PVD-VIEW-001 AC-1).
- Properties and all four visualization modes match each selected frame; old selection data clears on recording change. (REQ-UX-PVD-VIEW-001 AC-2).
- Close removes owned property/viewport subscriptions without affecting another viewer or provider. (REQ-UX-PVD-VIEW-001 AC-3).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-PVD-VIEW-001](../../requirements/features/REQ-UX-PVD-VIEW-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

