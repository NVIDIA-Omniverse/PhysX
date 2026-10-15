<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-DEFORM-COOK-001
maps_to: REQ-COMP-DEFORM-COOK-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-DEFORM-COOK-001; executable evidence and qualification limits follow.

## Scenario

Cook a deformable with distinct simulation, collision and visual meshes and validate its embedding.

## Given

- A closed tetrahedralizable mesh and a denser visual mesh with a different vertex count and transform.
- One visual point deliberately outside the declared embedding domain.

## When

- Cook the topology and embedding, reconstruct visual rest positions and request an unavailable backend.

## Then

- The result carries all supported mesh roles and their topology versions (REQ AC-1).
- Embedding reconstructs the valid visual rest positions within a fixture-declared tolerance after frame conversion (REQ AC-2).
- The outside point is identified and any incomplete mapping is explicitly marked (REQ AC-3).
- Unavailable modes return unsupported, while supported output is readable only after its documented completion (REQ AC-4).

## Executable coverage

[test_cooking.py](../../../tests_native/test_cooking.py):

Conforming and voxel topology, independent mesh identities, visual embedding and invalid/outside points are exercised with copied results. test_mesh.py verifies triangle/tet interpolation and host-buffer rules. tests/test_deformable_authoring.py runs the native worker through reversible customer-USD construction.

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.
