<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-BINDING-DEVICE-001
title: Python Tensor Binding Native Device Introspection
status: implemented
owner: ovphysx
---

## Description

The Python `TensorBinding` surface exposes the C binding's native DLPack device
so callers can allocate on the no-staging CPU or CUDA device without maintaining
a tensor-type allowlist. The device is binding metadata, not a DLPack producer
protocol: `TensorBinding` does not implement `__dlpack__` or
`__dlpack_device__`.

## Acceptance Criteria

- AC-1: `TensorBinding.native_device` returns a Python-owned `DLDevice` whose
  values match `ovphysx_get_tensor_binding_native_device` for the live binding.
- AC-2: CPU bindings report `{kDLCPU, 0}`. DirectGPU state bindings report
  `{kDLCUDA, N}`, using the actual CUDA ordinal. Non-DirectGPU state bindings
  report CPU even when the scene uses GPU dynamics.
- AC-3: CPU-only property bindings report CPU in DirectGPU mode. The property
  describes the native no-staging device and does not change existing
  read/write behavior.
- AC-4: Mutating a returned `DLDevice` cannot change subsequent
  `native_device` results. `TensorBindingSpec` keeps its existing three-field
  public tuple shape (`dtype`, `ndim`, `shape`).

## Test References

- TEST-PYTHON-BINDING-DEVICE-001

## Code References

- ovphysx/python/ovphysx/_bindings.py
- ovphysx/python/ovphysx/api.py (`TensorBinding.native_device`)
- ovphysx/python/ovphysx/api.pyi

## Dependencies

- [REQ-CAPI-BINDING-DEVICE-001](../capi/REQ-CAPI-BINDING-DEVICE-001.md)
