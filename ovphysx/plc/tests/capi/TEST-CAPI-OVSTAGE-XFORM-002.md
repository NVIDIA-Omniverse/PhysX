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
`OvStageOutputGpuTest.DirectGpuCachedAndUncachedMatricesPreserveScaleAndCallerContext`
in `tests/c_unittests/test_ovstage_output.cpp`.

## Given

- A CUDA-capable GPU and a fresh ovphysx instance.
- `tests/data/boxes_falling_on_groundplane_gpu.usda`, populated through
  `attach_usd_with_ovstage` at sealed ordinal 1.

## When

- Step once, read the populated world matrix, write signed scale at ordinal 2,
  and seal it.
- Publish world transforms at ordinals 3 through 6 with cached and uncached
  `writeWorldTransformsToOvstage` calls, sealing each output ordinal.

## Then

- The initial world-matrix read returns a matrix (REQ AC-1).
- Every publication succeeds and writes at least one matrix; signed scale is
  preserved and the box's translation reflects gravity (REQ AC-1).
