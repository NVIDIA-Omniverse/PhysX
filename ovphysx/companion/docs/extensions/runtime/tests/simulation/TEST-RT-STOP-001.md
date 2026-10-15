<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-STOP-001
maps_to: REQ-RT-STOP-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Stop restores rigid state and destroys the entire session. Verifies REQ-RT-STOP-001 AC-1, AC-2, AC-3, AC-4.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- One scene with the nested/nonzero-velocity and absent-property bodies from the snapshot fixture; one body has a render descendant and preexisting sim:* data.
- Run enough enabled substeps to alter pose/output; make an unrelated material edit; test reset-on-stop preference both true and false.

## When

- Stop from Running and, in a fresh parameter row, from Paused; inspect state only after the restoration completion fence.

## Then

- Caches/views/subscriptions close before backend detach/instance destruction; all session resource counters become zero and the snapshot is freed after restoration. (REQ-RT-STOP-001 AC-1).
- Source transforms, operation order/presence, initial linear/angular velocities and absent properties match the pre-Play copies exactly; no live actor setter is required. (REQ-RT-STOP-001 AC-2).
- Original shadows and descendant/world presentation are restored, newly created shadows removed, and unrelated material edits preserved. (REQ-RT-STOP-001 AC-3).
- Stop does not step, reattach, reparse or recreate resources; both preference values restore the same state and leave only host-stage bookkeeping. (REQ-RT-STOP-001 AC-4).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:


Mapped requirement: [REQ-RT-STOP-001](../../requirements/simulation/REQ-RT-STOP-001.md).
