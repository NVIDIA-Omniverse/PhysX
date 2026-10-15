<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-ATTACH-USD-001
title: "Python attachment authoring through customer USD"
status: implemented
owner: ovphysx
---

## Description

Python extracts attachment inputs and applies computed attachment/filter data to the customer stage. Kit adds command/undo around the same utility.

## Acceptance Criteria

- AC-1: **Extraction.** Extraction resolves body paths and instance/topology identity, geometry and transforms in Python and passes only native data to attachment computation.
- AC-2: **Application.** Application authors the selected attachment schemas, endpoint relationships, anchors/indices/weights and collision filters into the explicit destination and edit target.
- AC-3: **Managed children.** Updating/removing generated attachments affects only helper-owned children/properties; repeated application is idempotent and preserves unrelated authored content.
- AC-4: **Stale safety.** Deleted endpoints, topology changes, undo or source remapping invalidate pending results before edits; application participates in the shared reversible edit contract.

## Implementation scope

Static polygon/TetMesh captures retain exact topology domains, affine transforms,
units and source revisions. Native worker proposals are checked against those
captures before authoring vertex/triangle/tetrahedron constraints and paired
surface collision filters. Vertex-to-transform anchors support rigid subtrees
with scale/shear. Every edit uses an explicit local destination layer and owned
prim; removal disables the owned constraint/filter without deleting user content.

Reapplication is idempotent after recapture/recomputation for the current revision.
Each authored command, undo or redo invalidates prior proposals. Deleted/recreated
endpoints and topology/transform edits reject stale data before mutation. Editable
instance proxies and time-varying topology are explicitly unsupported, rather
than silently expanded. The upstream work-in-progress curve schema has no
advertised companion solver/provider; no curve simulation support is implied.

## Test References

- [TEST-COMP-ATTACH-USD-002](../../tests/python/TEST-COMP-ATTACH-USD-002.md) - executed Python subset; see its scenario and limitations.
- [TEST-COMP-ATTACH-USD-001](../../tests/python/TEST-COMP-ATTACH-USD-001.md) - executed proposal/transform/ownership scenario.

## Code References

Python implementation:

- [usd/meshes.py](../../../python/ovphysx_companion/usd/meshes.py)
- [usd/attachments.py](../../../python/ovphysx_companion/usd/attachments.py)
- [Executable subset tests](../../../tests/test_usd_helpers.py)
- [Element/filter/TetMesh tests](../../../tests/test_attachment_elements.py)

Historical integration anchors:

- [ovphysx/ovruntime/source/omni.physx/plugins/usdBridge/AttachmentAuthoringBridge.cpp](../../../../ovruntime/source/omni.physx/plugins/usdBridge/AttachmentAuthoringBridge.cpp)
- [ovphysx/python/ovphysx/utils/codeless.py](../../../../python/ovphysx/utils/codeless.py)

- [Element/filter extension sample](../../../samples/attachment_proposals.py)

## Dependencies

- [REQ-COMP-ATTACH-COMPUTE-001](../authoring/REQ-COMP-ATTACH-COMPUTE-001.md)
- [REQ-COMP-USD-EDIT-001](REQ-COMP-USD-EDIT-001.md)
- [Companion ADR-0005](../../adr/ADR-0005-attachment-and-deformable-split.md)
