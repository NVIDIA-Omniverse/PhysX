<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-BINDING-DEVICE-001
maps_to: REQ-PYTHON-BINDING-DEVICE-001
type: integration
---

## Scenario

Python callers inspect native binding residency without a hard-coded tensor-type
classification.

## Given

- A CPU rigid-body pose binding.
- A DirectGPU rigid-body pose binding and a CPU-only contact-offset property
  binding.
- A GPU-dynamics scene without DirectGPU enabled.
- The eight CPU-only binding types missing from the downstream static
  classification in the NVBug reproducer.
- A caller that mutates the `DLDevice` object returned by `native_device`.
- A binding destroyed or reset-stale after its native device was queried.

## When

The tests inspect `TensorBinding.native_device` for each binding and query the
CPU binding again after mutating the previously returned descriptor.

## Then

- The CPU binding reports `{kDLCPU, 0}`.
- The DirectGPU state binding reports `{kDLCUDA, 0}` and the CPU-only property
  binding reports `{kDLCPU, 0}`.
- The non-DirectGPU state binding reports `{kDLCPU, 0}`.
- All eight reported CPU-only types report `{kDLCPU, 0}` in a DirectGPU scene.
- A later property access returns the original values rather than the caller's
  mutation.
- A destroyed or reset-stale binding rejects `native_device` after an earlier
  successful query.
- The shipped type stub declares `native_device: DLDevice`.

## Test Location

- `ovphysx/tests/python_tests/cpu_tests/test_tensor_bindings_api.py`
- `ovphysx/tests/python_tests/test_tensor_bindings_api_gpu.py`
- `ovphysx/tests/python_tests/test_gpu_lifecycle_advanced.py`
- `ovphysx/tests/python_tests/test_type_stubs.py`
