<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-ATTACH-COMPUTE-001
title: "Native attachment and filter computation"
status: implemented
owner: ovphysx
---

## Description

Native attachment geometry services compute point/triangle/tetrahedron correspondence and attachment/filter proposals. They never create USD prims.

## Acceptance Criteria

- AC-1: **Input model.** Attachment computation accepts typed body/mesh identities, topology versions, mesh arrays, transforms and options; point/tet finders and sampling are usable without USD or a simulation attachment.
- AC-2: **Mapping result.** Results specify attachment kind, body endpoints, indices, local anchors or barycentric weights, filter pairs and source revisions with explicit coordinate conventions.
- AC-3: **Geometry validation.** Degenerate elements, out-of-range indices, empty matches and points outside the supported domain yield documented diagnostics rather than invalid weights or native crashes.
- AC-4: **Data ownership.** Results contain copied or leased data with defined lifetime; computation performs no authored-stage mutation and supports cancellation under the common job contract.

## Implementation scope

Stopped CPU vertex matching, triangle/tetrahedron mapping, surface/volume sampling
and triangle collision-filter computation are implemented. `MeshEndpoint` retains
copied geometry, affine transforms, units, body path, source token and topology
version. Proposals retain both endpoints, explicit unmatched/projected diagnostics,
local element coordinates or lexicographic surface-index filter pairs. Element
thresholds use target-local units; filter thresholds use world meters. Triangle
normal offsets remain local. Projected bindings are omitted unless requested.

Filter computation uses triangle distance (including edge/face and coplanar
intersections), not AABB overlap. Requests are bounded to ten million candidate
pairs and one million output pairs; overflow/capacity failure publishes nothing.
Empty pairs never mean wildcard filtering. Pure kernels require no runtime
initialization; AuthoringService adds copied-input jobs, cancellation and drain.
Sampling uses the separately retained stopped sampling service.

## Test References

- [TEST-COMP-ATTACH-COMPUTE-002](../../tests/authoring/TEST-COMP-ATTACH-COMPUTE-002.md) - executed native/Python subset.

- [TEST-COMP-ATTACH-COMPUTE-001](../../tests/authoring/TEST-COMP-ATTACH-COMPUTE-001.md) - executed full proposal scenario.

## Code References

Implemented companion subset (native/core changes retain their own PLC ownership):

- [include/ovphysx_companion/companion.h](../../../include/ovphysx_companion/companion.h)
- [src/Companion.cpp](../../../src/Companion.cpp)
- [python/ovphysx_companion/native.py](../../../python/ovphysx_companion/native.py)
- [python/ovphysx_companion/usd/attachments.py](../../../python/ovphysx_companion/usd/attachments.py)
- [samples/point_matching.py](../../../samples/point_matching.py)
- [samples/attachment_authoring.py](../../../samples/attachment_authoring.py)

- [Native element bridge](../../../src/Mesh.cpp)
- [Element mapping and interpolation](../../../python/ovphysx_companion/mesh.py)
- [Copied-input jobs](../../../python/ovphysx_companion/jobs.py)

Historical integration anchors:

- [ovphysx/ovruntime/include/private/omni/physx/IPhysxAttachmentPrivate.h](../../../../ovruntime/include/private/omni/physx/IPhysxAttachmentPrivate.h)
- [ovphysx/ovruntime/source/omni.physx/plugins/usdBridge/AttachmentAuthoringBridge.cpp](../../../../ovruntime/source/omni.physx/plugins/usdBridge/AttachmentAuthoringBridge.cpp)

- [Typed proposals](../../../python/ovphysx_companion/attachments.py)
- [Element/filter worker sample](../../../samples/attachment_proposals.py)

## Dependencies

- [REQ-COMP-JOBS-001](../exchange/REQ-COMP-JOBS-001.md)
- [REQ-COMP-IDENTITY-001](../exchange/REQ-COMP-IDENTITY-001.md)
- [REQ-COMP-BUFFERS-001](../exchange/REQ-COMP-BUFFERS-001.md)
- [Companion ADR-0005](../../adr/ADR-0005-attachment-and-deformable-split.md)
