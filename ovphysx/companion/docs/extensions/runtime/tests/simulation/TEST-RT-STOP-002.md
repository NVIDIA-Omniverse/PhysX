<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-STOP-002
maps_to: REQ-RT-STOP-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Stop restores dynamic instancer slots without overwriting authoring holes. Verifies REQ-RT-STOP-001 AC-2, AC-3.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Point-instancer snapshot fixture with simulated slots 0/2, unsimulated slot 1, authored trailing slot and nonzero velocities.
- After output changes dynamic slots, edit the unsimulated slot through the host and remove a separate simulated prim.

## When

- Stop and read native arrays, existence flags, scales and prototype indices from ovstage.

## Then

- Dynamic positions/orientations/linear-angular velocities return to their copied initial values and array representation. (REQ-RT-STOP-001 AC-2).
- Unsimulated edits and trailing data remain, scale/prototype data is unchanged and removed prims are not recreated. (REQ-RT-STOP-001 AC-3).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [test_step_and_write_to_ovstage.py](../../../../../../tests/python_tests/test_step_and_write_to_ovstage.py) — Utility tests for world matrices, instancer holes, shadow arrays, write failures and cache invalidation (current API reference).

Mapped requirement: [REQ-RT-STOP-001](../../requirements/simulation/REQ-RT-STOP-001.md).
