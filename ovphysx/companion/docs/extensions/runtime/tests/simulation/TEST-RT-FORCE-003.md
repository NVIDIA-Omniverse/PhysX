<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-FORCE-003
maps_to: REQ-RT-FORCE-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Interaction impulses do not persist or corrupt other force producers. Verifies REQ-RT-FORCE-001 AC-4.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Claimed-supported native or tensor force path with retain-accelerations enabled, a second uninvolved body, and a separate force producer with known contribution; input/force-group probes.

## When

- Apply push or grab for two scheduled h substeps in one Kit frame, release, then run a third substep without interaction.
- For an implemented tensor path, inspect J/h conversion, application point/torque, COM gravity and merge of full-group rows.

## Then

- Exactly two interaction applications occur, the third has none retained, and no h-squared scaling occurs. Unaffected rows and the independent producer retain their intended contributions. An unavailable tensor path must be reported unavailable rather than counted as passing parity. (REQ-RT-FORCE-001 AC-4).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [writable.md](../../../../../../docs/read_write/writable.md) — Force/wrench semantics, units, whole-group writes and execution-mode restrictions (current API reference).

Mapped requirement: [REQ-RT-FORCE-001](../../requirements/simulation/REQ-RT-FORCE-001.md).
