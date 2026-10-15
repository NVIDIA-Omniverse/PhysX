<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-OUTPUT-002
maps_to: REQ-RT-OUTPUT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Fixed poses update world matrices while preserving source authoring. Verifies REQ-RT-OUTPUT-001 AC-2.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- One scene with an admitted rigid body, articulation link and vehicle wheel, distinct orientations, valid evaluated matrices and nonuniform signed shear-free scales; preserve source local operations/reset-stack values.

## When

- Publish one completed pose pair per admitted type; separately request a position-only or orientation-only selection before stepping.

## Then

- Published world matrices reconstruct the returned poses and signed scales within 1e-5; source local operations, omni:xform and reset-stack are unchanged. An incomplete pose-pair selection is rejected before advancing physics. (REQ-RT-OUTPUT-001 AC-2).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [test_step_and_write_to_ovstage.py](../../../../../../tests/python_tests/test_step_and_write_to_ovstage.py) — Utility tests for world matrices, instancer holes, shadow arrays, write failures and cache invalidation (current API reference).

Mapped requirement: [REQ-RT-OUTPUT-001](../../requirements/simulation/REQ-RT-OUTPUT-001.md).
