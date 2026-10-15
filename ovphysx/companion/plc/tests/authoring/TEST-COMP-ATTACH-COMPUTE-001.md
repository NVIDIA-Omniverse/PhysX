<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-ATTACH-COMPUTE-001
maps_to: REQ-COMP-ATTACH-COMPUTE-001
type: integration
---

## Coverage status

Implemented and executed. Covers AC-1 through AC-4.

## Executable coverage

Executed in [test_attachment_proposals.py](../../../tests_native/test_attachment_proposals.py):
`test_proposal_endpoints_transforms_containment_and_cancellation`,
`test_filter_exact_triangle_distance_intersections_and_empty`,
`test_filter_native_capacity_failure_and_abi_preserve_canaries`, and
`test_pure_kernels_need_no_runtime_initialization`. All four passed, alongside
ten existing mapping/motion cases (14 total). The separate child loads only the
library and proves the pure kernels need no SDK initialization, attach or pxr.
The proposal fixture uses the specified tetrahedron, affine scale/shear, explicit
source/topology metadata, outside/degenerate inputs and queued-job cancellation.
Filter coverage includes AABB false positives, intersections, exact thresholds,
empty results, capacity canaries and owned results after service close.

## Scenario

Map points into a known tetrahedron and compute an attachment proposal from it.

## Given

- A tetrahedron at (0,0,0), (1,0,0), (0,1,0), (0,0,1), a point at (0.25,0.25,0.25), an outside point and a degenerate element.
- Body identities, transforms, topology versions and a read-only source snapshot.

## When

- Run point/tet mapping and attachment computation, then cancel a second job and release results.

## Then

- The operation completes without USD or attach and preserves the supplied identity/topology metadata (REQ AC-1).
- The interior point maps to the correct tet with four weights of 0.25 within 1e-6 and explicit endpoint/filter records (REQ AC-2).
- Outside and degenerate inputs follow documented diagnostic policy without invalid indices/NaNs (REQ AC-3).
- Source data is unchanged and cancelled/completed result ownership returns to baseline after release (REQ AC-4).

The native mesh regression `test_embedding_work_budget_rejects_before_allocating_result`
requires INVALID_ARGUMENT for a request above the synchronous kernel's bounded
point/element-pair workload, without publishing a result.
