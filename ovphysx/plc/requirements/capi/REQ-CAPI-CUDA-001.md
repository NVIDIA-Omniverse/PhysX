<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-CUDA-001
title: active_cuda_gpus Selection on Direct OVStage Attachment
status: implemented
owner: ovphysx
---

## Description

For an ovphysx instance created with a single explicit non-negative CUDA ordinal
in `ovphysx_create_args.active_cuda_gpus`, direct
`ovphysx_attach_ovstage()` must apply that ordinal to the physics runtime during
the synchronous scene attachment. The explicit selection also disables
multi-GPU scene distribution for that attachment. This requirement defines the
C API contract; backend configuration and test observation mechanisms are not
part of it.

## Acceptance Criteria

- AC-1: Given a valid non-negative ordinal `N` in
  `ovphysx_create_args.active_cuda_gpus`, a successful direct
  `ovphysx_attach_ovstage()` applies `N` as the runtime CUDA device selection
  before the call returns.
- AC-2: An explicit single ordinal disables multi-GPU scene distribution for
  that attachment.

## Test References

- TEST-CAPI-CUDA-001

## Code References

- ovphysx/src/ovphysx/ovphysx.cpp (`applyAttachTimeGpuSelection`, `ovphysx_attach_ovstage`)
- ovphysx/tests/c_unittests/test_instantiation.cpp (`ActiveCudaGpusAttachTest`)

## Dependencies

- [ADR-0011](../../../ovruntime/plc/adr/ADR-0011-ovphysx-device-execution-policy.md) -
  OVPhysX device selection, precedence, and attachment lifecycle policy
