<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-CUDA-003
title: Cross-Device Tensor-Binding Staging Runs in the Binding's CUDA Context
status: implemented
owner: ovphysx
---

## Description

A GPU-resident tensor binding can be read into host memory and written from it:
ovphysx allocates a staging buffer on the binding's device, runs the TensorAPI
call against it, and copies between staging and the caller's buffer.

Those driver calls carry no device argument, so they act on the CUDA context
current on the calling thread. That is not the simulation's, and once
`active_cuda_gpus` selects a non-zero ordinal it is a different device: the
staging buffer is then on a device the DirectGPU kernels cannot address, and the
read faults. Staging must therefore run under the context the binding's
simulation view reports.

Baseline requirement bootstrapped from NVBugs 6665594, where a rigid-body pose
read into a NumPy array failed with `cuda_status=700` under
`/physics/suppressReadback = true` and `active_cuda_gpus = "1"`. Scoped to the
staging path; ordinal selection itself is REQ-CAPI-CUDA-001.

## Acceptance Criteria

- AC-1: **Read.** `ovphysx_read_tensor_binding` with a GPU binding and host
  destination runs the staging allocation, the TensorAPI read, and the
  device-to-host copy under `simView->getCudaContext()`.
- AC-2: **Write.** `ovphysx_write_tensor_binding` with a GPU binding and a host
  source tensor runs the staging allocation and host-to-device copy under that
  same context. The mask and index tensors and
  `ovphysx_write_tensor_binding_masked` share the same staging helper and carry
  the same guarantee.
- AC-3: **Release.** A staging buffer is freed under the context it was
  allocated in, whatever the caller has current by then. If that context can no
  longer be made current the buffer is abandoned and the failure is logged,
  because the release runs from a destructor and has no error channel.
- AC-4: **The caller's context stack is restored** before returning. A push taken
  for staging is popped on every exit path from the call that took it.
- AC-5: **A missing context fails.** When the simulation view reports no CUDA
  context, the call returns `OVPHYSX_API_ERROR` with a message on
  `ovphysx_get_last_error()` rather than staging under the current context.

## Known limitations

The opposite direction — CPU binding, `kDLCUDA` caller tensor — still uses the
caller's context, and it does not degrade gracefully. The two unprotected calls
are `stageTensorForWrite`'s `memcpyDtoH` and `ovphysx_read_tensor_binding`'s
`memcpyHtoD`; both need some context current, so under the same no-context premise
as the tests below, both fail with `CUDA_ERROR_INVALID_CONTEXT`. The path is
reachable: a CPU-dynamics scene, a tensor type that is not CPU-only, and a
`kDLCUDA` caller tensor. It is not fixable at this layer, because
`ovruntime/include/omni/physx/IOptionalCuda.h` exposes no pointer-attribute query
and no primary-context retain, so ovphysx cannot derive or create the context that
owns a caller pointer. Cross-GPU staging stays rejected as
`OVPHYSX_API_DEVICE_MISMATCH`.

## Test References

- [TEST-CAPI-CUDA-003](../../tests/capi/TEST-CAPI-CUDA-003.md) - both directions
  with no context current, default ordinal
- [TEST-CAPI-CUDA-004](../../tests/capi/TEST-CAPI-CUDA-004.md) - the reported
  configuration: DirectGPU host read on ordinal 1

AC-5 has no automated case; see TEST-CAPI-CUDA-003. The only automated write is
`ovphysx_write_tensor_binding` with a host source tensor and no index tensor:
`ovphysx_write_tensor_binding_masked`, the mask tensor, and the index tensor have
no automated case. Neither does AC-4 on a failure path.

## Code References

- ovphysx/src/ovphysx/ovphysxTensorBinding.cpp (`ovphysx_read_tensor_binding`,
  `stageTensorForWrite`, `TensorStagingInfo::release`, `ScopedCudaContextPush`)
- ovphysx/tests/c_unittests/cuda_test_helpers.h (`ScopedCudaContextDetach`)
- ovphysx/tests/c_unittests/test_tensor_binding.cpp
  (`TensorBindingGpuTest.CrossDeviceStagingWithoutCallerCudaContext`)
- ovphysx/tests/c_unittests/test_instantiation.cpp
  (`ActiveCudaGpusAttachTest.DirectGpuHostReadOnNonZeroOrdinal`)

## Dependencies

- [REQ-CAPI-CUDA-001](REQ-CAPI-CUDA-001.md) - the ordinal selection that makes the
  simulation's context differ from the caller's
- [REQ-CAPI-WRITE-001](REQ-CAPI-WRITE-001.md) AC-11 - the tensor-binding surface
  specified here is deprecated and will be retired with it
- [ADR-0011](../../../ovruntime/plc/adr/ADR-0011-ovphysx-device-execution-policy.md) -
  OVPhysX device selection, precedence, and attachment lifecycle policy
