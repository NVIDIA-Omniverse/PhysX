<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-INPUT-002
maps_to: REQ-RT-INPUT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Control velocities retain intended identity and unit conversions. Verifies REQ-RT-INPUT-001 AC-3.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- One scene with dynamic body D and kinematic body K, stable object paths, and sealed supported input attributes.

## When

- Author D linear velocity (7,8,9) and angular velocity (0,0,90) degrees/s through ovstage; include K in a supported mixed input group. Drain before the next due step and inspect live state at that boundary.

## Then

- D reads linear (7,8,9) and angular (0,0,pi/2) radians/s within 1e-3. K is not assigned a dynamic body velocity by that batch; its authored surface-motion semantics remain separate. (REQ-RT-INPUT-001 AC-3).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [test_ovstage_drain.py](../../../../../../tests/python_tests/test_ovstage_drain.py) — test_drain_rigid_velocity and test_drain_rigid_velocity_mixed_kinematic (current API reference).
- [writable.md](../../../../../../docs/read_write/writable.md) — Force/wrench semantics, units, whole-group writes and execution-mode restrictions (current API reference).

Mapped requirement: [REQ-RT-INPUT-001](../../requirements/simulation/REQ-RT-INPUT-001.md).
