<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-STEP-004
maps_to: REQ-RT-STEP-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Each substep has one callback/force/step sequence and a distinct sample time. Verifies REQ-RT-STEP-001 AC-4, AC-5.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Running scene at 60 Hz, m=0, input sampler spy, one held ray command and pre/post/presentation event recorder.

## When

- Deliver dt=3h at currentTime=1.0; then request a timeline rewind.

## Then

- There are three ordered control/pre-step/ray(h)/step(h)/wait/output sequences and exactly one pre/post notification per subscription per step. Presentation is released only after its output fence. (REQ-RT-STEP-001 AC-4).
- Input sample times are 1-2h,1-h,1; completed solver time increases by 3h. Rewind requires Stop/new Play and does not reverse or replay the running solver. (REQ-RT-STEP-001 AC-5).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [simulation.py](../../../../../../python/ovphysx/utils/simulation.py) — step_and_write_to_ovstage and OvStageOutputCache (current API reference).

Mapped requirement: [REQ-RT-STEP-001](../../requirements/simulation/REQ-RT-STEP-001.md).
