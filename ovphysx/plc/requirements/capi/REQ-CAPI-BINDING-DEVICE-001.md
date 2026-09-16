<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-BINDING-DEVICE-001
title: Tensor Binding Native Device Introspection
status: implemented
owner: ovphysx
---

## Description

Tensor binding callers need to know where the binding's native TensorAPI data
resides before allocating a DLTensor. The public C API reports that residency as
a DLPack `DLDevice` without changing the stable layout of
`ovphysx_tensor_spec_t`. The reported device is the no-staging device used by
the binding's native read/write path; it does not change which cross-device
operations are otherwise supported.

## Acceptance Criteria

- AC-1: `ovphysx_get_tensor_binding_native_device` is an additive C ABI function with
  signature `(ovphysx_handle_t, ovphysx_tensor_binding_handle_t, DLDevice*)`.
- AC-2: A host-resident binding reports `DLDevice{kDLCPU, 0}`. A
  CUDA-resident binding reports `DLDevice{kDLCUDA, N}`, where `N` is the
  binding's native TensorAPI view ordinal.
- AC-3: Tensor types whose native TensorAPI property path is CPU-only report
  CPU even when their simulation view is DirectGPU. Other tensor types follow
  their native TensorAPI view: CUDA for DirectGPU and CPU otherwise, including
  when a non-DirectGPU scene uses GPU dynamics.
- AC-4: The getter rejects a null output pointer with
  `OVPHYSX_API_INVALID_ARGUMENT` and an unknown, attach-stale, or invalidated
  simulation-view binding with `OVPHYSX_API_NOT_FOUND` before reading through
  binding-owned views.
- AC-5: The experimental C++ `TensorBinding` wrapper exposes the same query as
  `nativeDevice(DLDevice&)`. Reporting the device does not alter read/write
  staging, device validation, or binding lifetime.

## Test References

- TEST-CAPI-BINDING-DEVICE-001

## Code References

- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_get_tensor_binding_native_device`)
- ovphysx/src/ovphysx/ovphysxTensorBinding.cpp (`getBindingNativeDeviceOrdinal`,
  `ovphysx_get_tensor_binding_native_device`)
- ovphysx/include/ovphysx/experimental/TensorBinding.hpp (`nativeDevice`)
- ovphysx/src/ovphysx/TensorBindingWrapper.cpp (`TensorBinding::nativeDevice`)

## Dependencies

- [ADR-0011](../../../ovruntime/plc/adr/ADR-0011-ovphysx-device-execution-policy.md) -
  tensor placement and CUDA ordinal policy
