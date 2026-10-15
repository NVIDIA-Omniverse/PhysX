<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-OUTPUT-004
maps_to: REQ-RT-OUTPUT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Shadow values and render descendants are ready at publication completion. Verifies REQ-RT-OUTPUT-001 AC-4.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- One scene with admitted articulation root/joint data, rigid angular velocity, a render child under a moving body and an admitted deformable/particle visible-output consumer.

## When

- Publish selected non-pose values and complete host descendant/geometry consumption.

## Then

- Non-pose data appears in sim:<name> with documented frames/units; rigid angular velocity remains radians/s. Source physics attributes are not implicitly overwritten; child transforms and visible points reflect the completed output before presentation-ready is reported. (REQ-RT-OUTPUT-001 AC-4).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [test_step_and_write_to_ovstage.py](../../../../../../tests/python_tests/test_step_and_write_to_ovstage.py) — Utility tests for world matrices, instancer holes, shadow arrays, write failures and cache invalidation (current API reference).
- [writable.md](../../../../../../docs/read_write/writable.md) — Force/wrench semantics, units, whole-group writes and execution-mode restrictions (current API reference).

Mapped requirement: [REQ-RT-OUTPUT-001](../../requirements/simulation/REQ-RT-OUTPUT-001.md).
