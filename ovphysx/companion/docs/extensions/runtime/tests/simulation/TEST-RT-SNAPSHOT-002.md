<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-SNAPSHOT-002
maps_to: REQ-RT-SNAPSHOT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Instancer capture distinguishes simulated slots from untouched data. Verifies REQ-RT-SNAPSHOT-001 AC-3.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- One point instancer with dynamic slots 0 and 2, unsimulated slot 1, a trailing authored slot, distinct local poses/linear-angular velocities, scales and prototype indices.

## When

- Capture on Play and allow one output publication.

## Then

- Snapshot preserves each dynamic slot identity, initial pose/velocity and original array extent; it identifies untouched slots and retains original presentation presence without conflating arrays with world-space rigid output. (REQ-RT-SNAPSHOT-001 AC-3).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [test_step_and_write_to_ovstage.py](../../../../../../tests/python_tests/test_step_and_write_to_ovstage.py) — Utility tests for world matrices, instancer holes, shadow arrays, write failures and cache invalidation (current API reference).

Mapped requirement: [REQ-RT-SNAPSHOT-001](../../requirements/simulation/REQ-RT-SNAPSHOT-001.md).
