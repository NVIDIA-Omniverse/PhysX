<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-DEFORM-USD-001
maps_to: REQ-COMP-DEFORM-USD-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-DEFORM-USD-001; executable evidence and qualification limits follow.

## Scenario

Author and recook a deformable with different visual and simulation topology.

## Given

- A customer USD stage, registered codeless schemas, an unrelated material opinion and a valid native deformable cook result.
- A second cook result with changed simulation topology.

## When

- Construct the deformable, inspect its mesh-role relationships and rest arrays, then apply the second result through a reversible batch.

## Then

- The chosen layer contains the complete hierarchy, schemas and relationships (REQ AC-1).
- Authored arrays and embedding metadata match the result roles, counts and index domains (REQ AC-2).
- Unrelated material opinions survive and old topology-dependent mappings become stale (REQ AC-3).
- Headless execution imports no Kit/compiled USD companion module and the host wrapper can undo the recook (REQ AC-4).

## Executable coverage

[test_deformable_authoring.py](../../../tests/test_deformable_authoring.py):

Seven cases verify native cook-to-hierarchy, surface/volume roles, rest/bind/material data, distinct visual topology, owned recook and mapping invalidation, unrelated opinion preservation, TetMesh output and reversible publication.

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.


Surface and volume construction with separate simulation/collision siblings
binds the same physics material to both; the simulation mesh need not carry
CollisionAPI for the native material lookup to work.
