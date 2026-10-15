<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-DEFORM-USD-001
title: "Python deformable construction"
status: implemented
owner: ovphysx
---

## Description

Apply native cook results to a customer USD deformable hierarchy using codeless schema helpers and explicit rest/bind data.

## Acceptance Criteria

- AC-1: **Hierarchy.** The helper authors the intended body, simulation/collision/visual meshes, physics schemas, materials and relationships using customer USD in an explicit layer.
- AC-2: **Rest data.** Cooked topology, rest/bind arrays and embedding metadata are validated and authored to their declared mesh roles without substituting visual topology for simulation topology.
- AC-3: **Update ownership.** Reconstruction updates only helper-owned opinions and invalidates dependent attachment/embedding mappings when topology changes.
- AC-4: **Portability.** The same helper works headlessly without Kit and under a host undo wrapper without requiring compiled companion USD schema bindings.

## Test References

- [TEST-COMP-DEFORM-USD-001](../../tests/python/TEST-COMP-DEFORM-USD-001.md) - executed native cook-to-customer-USD hierarchy and reversible application.

## Code References

[usd/deformables.py](../../../python/ovphysx_companion/usd/deformables.py) applies
copied versioned surface/volume data into helper-owned simulation, collision,
visual and material prims with explicit rest/bind data and embedding metadata.
Stable role names are required on reconstruction. Five executed cases in
[test_deformable_authoring.py](../../../tests/test_deformable_authoring.py) cover
roles, material binding, rest/visual count separation, invalid input, source
invalidation, unowned content, embedding removal and reversible TetMesh output.
Native cook production and the worker-to-customer-USD hierarchy are tested.
Only declared material fields are authored; device deformable simulation is not
qualified by these authoring tests.

Existing integration anchors:

- [ovphysx/python/ovphysx/utils/deformable.py](../../../../python/ovphysx/utils/deformable.py)
- [ovphysx/python/ovphysx/utils/codeless.py](../../../../python/ovphysx/utils/codeless.py)

## Dependencies

- [REQ-COMP-DEFORM-COOK-001](../authoring/REQ-COMP-DEFORM-COOK-001.md)
- [REQ-COMP-USD-EDIT-001](REQ-COMP-USD-EDIT-001.md)
- [Companion ADR-0005](../../adr/ADR-0005-attachment-and-deformable-split.md)
