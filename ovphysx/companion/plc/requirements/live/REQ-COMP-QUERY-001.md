<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-QUERY-001
title: "Scene queries and live inspection reuse core operations"
status: implemented
owner: ovphysx
---

## Description

Scene-query and inspection adapters use core read/query sessions where equivalent, adding only missing shape-query or typed state operations needed by UX consumers.

## Acceptance Criteria

- AC-1: **Query semantics.** Raycast, sweep and overlap document any/closest/all, filtering, sidedness, shape/primitive inputs, truncation and hit ordering; unsupported variants report unsupported rather than approximate silently.
- AC-2: **Identity.** Query hits return owner/generation/object and instance identities, distances/positions/normals with declared units/frame, and the evaluated scene revision or completed step.
- AC-3: **Inspection.** Body/joint/material/articulation inspection uses supported typed data; joint frame conversion is explicit and debug mass inspection depends on no unit-test interface.
- AC-4: **Ordering.** Queries and live reads observe the owning core session completion rules and reject stale handles without dereferencing deleted actors.

## Implementation scope

`Inspector` and `SceneQueries` borrow a ToolsSession or independent PreviewSession.
C++ resolves typed records under one exact-owner idle admission. Native scene queries
supply identity/filter/capacity behavior unavailable through the legacy path-key
core query calls; they query the same existing PhysX scene-query structures without
another scene, runtime, or cooking path. Equivalent core results are regression
oracles; `capture_body_output` continues to delegate supported core read sessions.

CPU ray, sphere/box/capsule and already-cooked convex-shape sweep/overlap support
the documented modes. Overlap closest, unsupported bound geometry, GPU scenes,
and unsupported flags fail explicitly. Filters use validated body/shape records;
point-instance body identities distinguish shared shapes. Results own copies,
full counts, bounded nearest results and truncation. Optional geometric fields
have validity bits (initial-overlap position can be absent).

Inspection copies body pose/mass/COM/inertia/velocities, native joint local/world
frames and break fields, material properties and articulation root/link/DOF cache
state. Native type/axis interpretation is explicit: an authored revolute may become
a D6, and a USD Y-axis articulation joint may use native twist with rotated frames.
Applied joint force is not a measured constraint force. No test interface is used.

## Test References

- [TEST-COMP-QUERY-001](../../tests/live/TEST-COMP-QUERY-001.md).

## Code References

- [Typed inspection C table](../../../include/ovphysx_companion/inspection.h)
- [Query C table](../../../include/ovphysx_companion/queries.h)
- [Inspection bridge](../../../src/Inspection.cpp)
- [Query bridge](../../../src/Queries.cpp)
- [Python inspection](../../../python/ovphysx_companion/inspection.py)
- [Python query facade](../../../python/ovphysx_companion/queries.py)
- [Completed core body reads](../../../python/ovphysx_companion/output.py)
- [Extension-style Inspector sample](../../../samples/inspector_tools.py)
- [Inspection integration tests](../../../tests_native/test_inspection.py)
- [Query integration tests](../../../tests_native/test_queries.py)

## Dependencies

- [REQ-COMP-OWNER-001](../architecture/REQ-COMP-OWNER-001.md)
- [REQ-COMP-BUFFERS-001](../exchange/REQ-COMP-BUFFERS-001.md)
- [Companion ADR-0006](../../adr/ADR-0006-host-services-and-ux-migration.md)
