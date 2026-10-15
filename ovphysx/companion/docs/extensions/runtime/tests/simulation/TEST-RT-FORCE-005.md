<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-FORCE-005
maps_to: REQ-RT-FORCE-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Deformable grab preserves barycentric attachment behavior. Verifies REQ-RT-FORCE-001 AC-6.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Declared-supported deformable picking path, known hit triangle and barycentric weights, and an offset rigid object as a negative control.

## When

- Begin and move a deformable drag, then release; repeat by closing the stage during the drag.

## Then

- The target uses the selected barycentric material point and radius 10 in stage units, with the intended attachment/filter rather than a rigid-hit substitute. Both termination paths remove owned attachment/filter/target state before backend teardown. (REQ-RT-FORCE-001 AC-6).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:


Mapped requirement: [REQ-RT-FORCE-001](../../requirements/simulation/REQ-RT-FORCE-001.md).
