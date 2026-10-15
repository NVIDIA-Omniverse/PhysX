<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-UX-STAGE-POLICY-001
maps_to: REQ-UX-STAGE-POLICY-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Retire unsupported stage controls without losing metadata. Verifies REQ-UX-STAGE-POLICY-001 AC-1, AC-4.

## Given

- The [UX fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; runtime fixtures use exactly one physics scene.
- Stage metadata contains all legacy USD-output, joint tolerance/extended-angle and USDRT keys plus unrelated sentinels.

## When

- Open/rebuild stage settings, save/reopen and Play through the ovphysx provider.

## Then

- No USD/local-velocity/particle-output control appears and no legacy writer key is applied; output uses the reviewed ovstage path. (REQ-UX-STAGE-POLICY-001 AC-1).
- Joint tolerance, extended-angle and USDRT controls are absent; all retired metadata and sentinels survive unchanged without private config replay. (REQ-UX-STAGE-POLICY-001 AC-4).

## Why not implemented

The proposed UX providers, configuration bridge and independent authoring lifecycle are not implemented in the companion Kit integration. Existing legacy tests do not exercise this migration. Implement this scenario with the corresponding feature; do not replace its assertions with a passing legacy case.

Mapped requirement: [REQ-UX-STAGE-POLICY-001](../../requirements/features/REQ-UX-STAGE-POLICY-001.md). The [source comparison](../../README.md#legacy-behavior-and-current-api-evidence) identifies baseline behavior and intentional differences.

