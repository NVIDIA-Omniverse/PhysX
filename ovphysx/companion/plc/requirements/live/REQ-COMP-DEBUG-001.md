<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-DEBUG-001
title: "Diagnostic frames and statistics snapshots"
status: implemented
owner: ovphysx
---

## Description

Expose live debug geometry, statistics and optional export diagnostics as stable data. Stopped collision representations use the authoring service rather than solver debug output.

## Acceptance Criteria

- AC-1: **Frame identity.** Debug points, lines and triangles are copied or leased as one frame after step completion with declared native counter, geometry timing, color, coordinate, units and culling/scale semantics. Completion does not imply post-step vertex poses; see [Debug geometry](../../../samples/README.md#debug-geometry).
- AC-2: **Statistics.** Versioned counter snapshots document scope and units; clients cannot mutate internal scene-statistics objects or require Carbonite statistics UI.
- AC-3: **Stopped representations.** Collision approximations, per-shape overlays and normals remain available through authoring results when no simulation is attached.
- AC-4: **Export capability.** RepX export, legacy PVD connectivity and other optional diagnostic operations advertise availability separately from OmniPVD recording and refuse unsupported requests explicitly.

## Implementation scope

CPU debug frames copy completed-step primitives with owner, color, frame, scale and
culling contracts. Counter snapshots are immutable. collision_overlay derives
per-shape triangles and transformed normals from retained cooked data without an
attachment. export_availability reports RepX/legacy PVD unsupported separately
from OVD recording. GPU debug telemetry is not advertised.

## Test References

- [TEST-COMP-DEBUG-002](../../tests/live/TEST-COMP-DEBUG-002.md) - executed native/Python subset.

- [TEST-COMP-DEBUG-001](../../tests/live/TEST-COMP-DEBUG-001.md) - completed frames, stopped normals and explicit absent exports are executed.

## Code References

Companion implementation (native/core changes retain their own PLC ownership):

- [src/Companion.cpp](../../../src/Companion.cpp)
- [python/ovphysx_companion/native.py](../../../python/ovphysx_companion/native.py)
- [samples/live_statistics.py](../../../samples/live_statistics.py)

- [Debug C table](../../../include/ovphysx_companion/debug.h)
- [Debug bridge](../../../src/Debug.cpp)
- [Python debug frames](../../../python/ovphysx_companion/debug.py)
- [Debug tests](../../../tests_native/test_debug.py)
- [Debug sample](../../../samples/debug_geometry.py)

Existing integration anchors for remaining scope:

- [ovphysx/include/ovphysx/ovphysx.h](../../../../include/ovphysx/ovphysx.h)
- [ovphysx/ovruntime/include/omni/physx/IPhysxVisualization.h](../../../../ovruntime/include/omni/physx/IPhysxVisualization.h)
- [ovphysx/ovruntime/include/omni/physx/IPhysxStatistics.h](../../../../ovruntime/include/omni/physx/IPhysxStatistics.h)



## Dependencies

- [REQ-COMP-BUFFERS-001](../exchange/REQ-COMP-BUFFERS-001.md)
- [REQ-COMP-COOK-001](../authoring/REQ-COMP-COOK-001.md)
- [Companion ADR-0006](../../adr/ADR-0006-host-services-and-ux-migration.md)
