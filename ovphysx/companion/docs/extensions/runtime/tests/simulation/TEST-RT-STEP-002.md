<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-STEP-002
maps_to: REQ-RT-STEP-001
type: unit
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Fractional time and capped backlog reproduce the legacy formula. Verifies REQ-RT-STEP-001 AC-2, AC-5, AC-6.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Scheduler fixture with one 60 Hz scene, deterministic double-precision oracle, no backend dynamics dependency. Start each parameter row with r=0.

## When

- For m=0, deliver two dt=1/120 updates; independently deliver dt=0.1 with m=30, m=0, and m=120.
- For the m=30 row, follow with dt=h; probe arithmetic planning with dt=1e12 seconds and m=0 without executing that many solver iterations.

- From the m=30 row with r=h, pause/resume without elapsed input, then Stop/start a new epoch and inspect the remainder at each boundary.

## Then

- Half-step updates yield n=0,r=h/2 then n=1,r=0. dt=0.1 yields (n=2,r=2h) for m=30, (n=6,r=0) for m=0 and (n=1,r=h) for m=120. The m=30 follow-up yields n=2,r=h; compare remainders within 1e-9 seconds. (REQ-RT-STEP-001 AC-2).
- The first m=30 update accounts for exactly 2h of solver time, not the 0.1 seconds of supplied wall time; retained or discarded time does not count as completed physics. (REQ-RT-STEP-001 AC-5).
- Uncapped arithmetic preserves the computed count without an artificial cap or integer wrap; pause/resume preserves r and Stop/new epoch resets it to zero. (REQ-RT-STEP-001 AC-6).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:


Mapped requirement: [REQ-RT-STEP-001](../../requirements/simulation/REQ-RT-STEP-001.md).
