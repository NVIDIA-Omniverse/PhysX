<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-STAGE-POLICY-002
maps_to: REQ-UX-STAGE-POLICY-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Freeze adapter catch-up for one play epoch. Verifies REQ-UX-STAGE-POLICY-001 AC-2.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- A single 120-Hz scene, adapter min-frame-rate=30 and deterministic update driver.

## When

- Play, request 60 while running/paused, then Stop/Play; separately test 0/1/1000 and invalid -1/1001.

## Then

- The first epoch retains its 30 policy and the next uses 60; 0 is uncapped, valid bounds persist, invalid bounds do not. No presumed ovphysx min-frame-rate config entry is emitted. (REQ-UX-STAGE-POLICY-001 AC-2).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-STAGE-POLICY-001](../../requirements/features/REQ-UX-STAGE-POLICY-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

