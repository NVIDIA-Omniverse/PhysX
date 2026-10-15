<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-INPUT-001
maps_to: REQ-RT-INPUT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Interleaved control and output ordinals never feed physics back into itself. Verifies REQ-RT-INPUT-001 AC-1, AC-2.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- A running attachment initialized at sealed ordinal 10; new controls at 11 and 13; physics output at 12 and 14; pending unsealed control at 15.
- Input adapter spy records ranges; use a velocity control whose application count is observable independently of output.

## When

- Drain/update the pending controls, request an overlapping control range again, and exercise an endpoint-only selection. Seal 15 only afterwards and submit a further update.

## Then

- No update_from_ovstage range includes 12 or 14 or reconsumes attach ordinal 10; controls 11/13 precede their affected steps. (REQ-RT-INPUT-001 AC-1).
- Each control is applied once, repeated/overlapping consumption does not duplicate effects, endpoint-only selection consumes only its endpoint, and 15 is not passed while unsealed. (REQ-RT-INPUT-001 AC-2).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — Current attach/update/detach/step/read/query API and ownership contracts (current API reference).
- [test_ovstage_drain.py](../../../../../../tests/python_tests/test_ovstage_drain.py) — test_drain_rigid_velocity and test_drain_rigid_velocity_mixed_kinematic (current API reference).

Mapped requirement: [REQ-RT-INPUT-001](../../requirements/simulation/REQ-RT-INPUT-001.md).
