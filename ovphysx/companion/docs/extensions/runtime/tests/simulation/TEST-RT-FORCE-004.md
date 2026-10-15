<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-FORCE-004
maps_to: REQ-RT-FORCE-001
type: unit
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Joint grab uses the legacy drive constants and owns its temporary objects. Verifies REQ-RT-FORCE-001 AC-5.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Declared-supported joint-grab path, m=2 and inertia vector (1,2,2); parameterize no extra constraint versus an additional constraint.

## When

- Begin joint grab, inspect created free-axis D6 drives, move target, and release.

## Then

- D=25, K=78.125. Without extra constraints caps are 6000 linear and 9.42 angular; with them, 1800 and 2.826. Both target/global kinematic pose are set without artificial target velocity; owned joint/target are released and borrowed body survives. (REQ-RT-FORCE-001 AC-5).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:


Mapped requirement: [REQ-RT-FORCE-001](../../requirements/simulation/REQ-RT-FORCE-001.md).
