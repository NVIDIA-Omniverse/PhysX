<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-ARTICULATION-NAMES-001
title: Legacy Binding String Arrays Report Overflow
status: implemented
owner: ovphysx
---

## Description

The deprecated articulation DOF, body, and joint name getters and tensor-binding
prim-path getter expose ordered string arrays through fixed-capacity caller
buffers. Each getter reports complete demand separately from the valid prefix
written, so callers can detect truncation and retry with sufficient capacity.

## Acceptance Criteria

- AC-1: **Complete demand.** `ovphysx_articulation_get_dof_names`,
  `ovphysx_articulation_get_body_names`, and
  `ovphysx_articulation_get_joint_names` set `out_count` to the complete name
  demand for the articulation.
- AC-2: **Detectable truncation.** When demand exceeds `max_names`, each getter writes the first
  `max_names` names in articulation order, writes no entry beyond that
  capacity, and returns `OVPHYSX_API_BUFFER_TOO_SMALL`.
- AC-3: **Complete fill.** When demand does not exceed `max_names`, each getter writes every name
  and returns `OVPHYSX_API_SUCCESS`. Every written string continues to satisfy
  REQ-CAPI-STRING-001.
- AC-4: **Binding path demand.** `ovphysx_tensor_binding_get_prim_paths`
  reports the complete binding-path demand through `out_count`. When demand
  exceeds `max_paths`, it writes the first `max_paths` entries in binding row
  order, writes no entry beyond that capacity, and returns
  `OVPHYSX_API_BUFFER_TOO_SMALL`; otherwise it writes every path and returns
  `OVPHYSX_API_SUCCESS`.

## Test References

- [TEST-CAPI-ARTICULATION-NAMES-001](../../tests/capi/TEST-CAPI-ARTICULATION-NAMES-001.md)

## Code References

- ovphysx/include/ovphysx/ovphysx_legacy.h
- ovphysx/src/ovphysx/ovphysxTensorBinding.cpp
- ovphysx/tests/c_unittests/test_articulation_metadata.cpp

## Dependencies

- [REQ-CAPI-STRING-001](REQ-CAPI-STRING-001.md)
- [REQ-CAPI-LEGACY-001](REQ-CAPI-LEGACY-001.md)
