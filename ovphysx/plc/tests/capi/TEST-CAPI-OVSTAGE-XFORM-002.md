<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OVSTAGE-XFORM-002
maps_to: REQ-CAPI-OVSTAGE-XFORM-001
type: integration
---

## Scenario

Population retains a world-matrix column usable by output publication.
Automated coverage is
`OvStageOutputGpuTest.DirectGpuCachedAndUncachedMatricesUseDeviceScaleAndCallerContext`
in `tests/c_unittests/test_ovstage_output.cpp`.

## Given

- A CUDA-capable GPU and a fresh ovphysx instance.
- `tests/data/ovstage_output_bodies_gpu.usda`, a DirectGPU scene with a mirrored
  body and a body under a scaled parent, populated through
  `attach_usd_with_ovstage` at sealed ordinal 1.

## When

- Step once, read the populated world matrices at ordinal 1, and read the
  device-resident physics pose and `worldScale` columns.
- With no CUDA context current on the calling thread, publish world transforms
  at ordinals 2 through 5 with cached (2 and 3), uncached (4) and refreshed (5)
  `writeWorldTransformsToOvstage` calls, sealing each output ordinal.

## Then

- The initial world-matrix read returns a matrix for each body, and its
  decomposed scale matches the stored `worldScale` (REQ AC-1).
- Every publication succeeds and writes one matrix per body, each composed from
  the physics pose and the stored signed scale, and the calling thread still has
  no CUDA context current afterwards (REQ AC-1).
