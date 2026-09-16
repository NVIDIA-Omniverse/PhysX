<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-BINDING-DEVICE-001
maps_to: REQ-CAPI-BINDING-DEVICE-001
type: integration
---

## Scenario

Public C callers query native tensor-binding residency before allocating their
DLTensor storage.

## Given

- A CPU scene with a rigid-body state binding.
- A DirectGPU scene with an articulation state binding and a CPU-only
  articulation property binding.
- Null-output, invalid-instance, unknown-binding, attach-stale, and
  invalidated-simulation-view getter inputs.
- A pure C11 translation unit that includes the public ovphysx header.

## When

The tests call `ovphysx_get_tensor_binding_native_device` for each live binding and
for each invalid input, and compile the public function pointer against its
declared C signature.

## Then

- The CPU state binding reports `{kDLCPU, 0}`.
- The DirectGPU state binding reports `{kDLCUDA, 0}` in the test fixture, whose
  simulation and CUDA allocation both select ordinal zero.
- The DirectGPU CPU-only property binding reports `{kDLCPU, 0}`.
- A null output reports `OVPHYSX_API_INVALID_ARGUMENT`; unknown, stale, and
  invalidated-simulation-view bindings report `OVPHYSX_API_NOT_FOUND`.
- An invalid instance reports `OVPHYSX_API_ERROR` without modifying the
  caller's output descriptor.
- The pure C11 signature assertion compiles without changing
  `ovphysx_tensor_spec_t`.

## Test Location

- `ovphysx/tests/c_unittests/test_tensor_binding.cpp`
- `ovphysx/tests/c_unittests/test_articulation_metadata.cpp`
- `ovphysx/tests/c_unittests/test_c_api_compatibility.c`
- `ovphysx/tests/c_unittests/test_cpp_wrapper_comprehensive.cpp`
