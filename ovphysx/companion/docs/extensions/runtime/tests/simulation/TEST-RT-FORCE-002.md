<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-FORCE-002
maps_to: REQ-RT-FORCE-001
type: unit
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Grab includes point velocity, temporary damping and gravity assistance. Verifies REQ-RT-FORCE-001 AC-2, AC-3.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- m=2, pickingForce=1 (k=50), h=0.01, d=1, local/world pick p=(0,1,0), COM=0, ray origin=(0,1,0), u=(1,0,0), v=(1,0,0), omega=(0,0,2), original damping=0.1, g=(0,-10,0).

## When

- After initial pick acquisition, calculate/apply the next due grab substep; release afterwards.
- Repeat with gravity disabled, zero gravity and original damping 2.0.
- Begin with coefficient zero, then change the same grab to 0.5 between completed steps; test a host update with no due substep.

## Then

- vPoint=(-1,0,0), target=(1,1,0), action=(1.02,0,0), impulse=(1.02,0,0) and moment=(0,0,-1.02), each within 1e-6. (REQ-RT-FORCE-001 AC-2).
- Damping becomes 1.25 and restores to 0.1; enabled gravity adds COM delta-v=(0,0.05,0). Disabled/zero gravity adds none; original damping 2.0 is not lowered. (REQ-RT-FORCE-001 AC-3).

- Zero coefficient gives no spring impulse and retains gravity assistance; tuning affects the next due impulse without changing initial angular damping, and zero due substeps apply no impulse. (REQ-RT-FORCE-001 AC-2, AC-3).

Native service coverage is recorded separately in [TEST-COMP-INTERACTION-002](../../../../../plc/tests/live/TEST-COMP-INTERACTION-002.md); this host adapter scenario remains unimplemented.

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:


Mapped requirement: [REQ-RT-FORCE-001](../../requirements/simulation/REQ-RT-FORCE-001.md).
