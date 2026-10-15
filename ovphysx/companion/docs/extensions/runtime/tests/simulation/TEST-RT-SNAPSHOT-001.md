<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-SNAPSHOT-001
maps_to: REQ-RT-SNAPSHOT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Capture exact rigid authoring before simulation touches it. Verifies REQ-RT-SNAPSHOT-001 AC-1, AC-2.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- One scene with a nested rigid body using ordered translate/rotate/scale operations, reset-stack metadata, linear velocity (1,2,3) and authored angular velocity (0,0,90) degrees/s; a second body has no authored velocities or sim:* shadows.
- Freeze expected host values/presence at the sealed Play input.

## When

- Play without stepping, inspect a test-only copy of the adapter snapshot, then let physics/output change the bodies.

## Then

- Snapshot values were copied before backend load/output and remain unchanged after backend/output buffers mutate or borrowed views close. (REQ-RT-SNAPSHOT-001 AC-1).
- The snapshot matches source operation order, reset metadata, scale, parent-relative pose, nonzero velocities with units, and absent-property flags exactly. (REQ-RT-SNAPSHOT-001 AC-2).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:


Mapped requirement: [REQ-RT-SNAPSHOT-001](../../requirements/simulation/REQ-RT-SNAPSHOT-001.md).
