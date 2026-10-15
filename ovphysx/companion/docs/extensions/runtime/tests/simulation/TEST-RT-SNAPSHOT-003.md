<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-SNAPSHOT-003
maps_to: REQ-RT-SNAPSHOT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Admission requires restart and visible-output mappings for every selected type. Verifies REQ-RT-SNAPSHOT-001 AC-4.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Parameterize a single-scene fixture with articulation root/joint state, deformable points/velocities or particle points/velocities; provide a complete declared mapping, then repeat with one restoration or visible-output mapping removed.

## When

- Request Play and, for admitted types, inspect captured restart/presentation state before the first output.

## Then

- Complete mappings capture the specific root/joint or nodal/particle fields. Missing mappings reject admission before output mutation rather than advertising support or treating link poses as writable root state. (REQ-RT-SNAPSHOT-001 AC-4).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [simulation.py](../../../../../../python/ovphysx/utils/simulation.py) — step_and_write_to_ovstage and OvStageOutputCache (current API reference).
- [writable.md](../../../../../../docs/read_write/writable.md) — Force/wrench semantics, units, whole-group writes and execution-mode restrictions (current API reference).

Mapped requirement: [REQ-RT-SNAPSHOT-001](../../requirements/simulation/REQ-RT-SNAPSHOT-001.md).
