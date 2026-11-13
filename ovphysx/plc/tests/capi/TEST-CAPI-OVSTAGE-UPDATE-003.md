<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OVSTAGE-UPDATE-003
maps_to: REQ-CAPI-OVSTAGE-UPDATE-001
type: integration
---

## Scenario

A failed incremental scan reports a task error and remains retryable after sealing.

## Given

- A caller-owned OVStage with two articulations, fully sealed and attached at ordinal 1.
- A third articulation added at ordinal 2 with everything except `physics:body0`
  and `physics:body1` sealed.

## When

- Submit and wait for two updates of range [2, 2].
- Fully seal ordinal 2, retry that range, and replay it once more.

## Then

- Both unsealed updates are accepted but their tasks fail. The first task's error
  identifies a relationship read at ordinal 2 (AC-3, AC-4).
- There are still two articulations after each failure (AC-4).
- The sealed retry succeeds and exposes three articulations (AC-4).
- Replay succeeds and the count remains three (AC-1, AC-4).

## Test locations

- `ovphysx/tests/c_unittests/test_usd_loading.cpp`:
  `PartialSealIncrementalScanCanRetryAfterSealingJointRelationships`.
