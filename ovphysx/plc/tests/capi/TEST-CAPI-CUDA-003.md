<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-CUDA-003
maps_to: REQ-CAPI-CUDA-003
type: integration
---

## Scenario

Both staging directions succeed when the calling thread carries no CUDA context,
proving staging pushes the binding's rather than inheriting one. Detaching is the
stand-in for the reported foreign-device context and the only one a single-GPU
machine can build, since `IOptionalCuda` exposes no context creation. Without the
fix it fails at an earlier point in the same path — at the staging allocation
rather than at the DirectGPU gather — so it does not subsume the foreign-context
case. TEST-CAPI-CUDA-004 covers a non-zero ordinal.

`TensorBindingGpuTest.CrossDeviceStagingWithoutCallerCudaContext` in
`tests/c_unittests/test_tensor_binding.cpp`.

## Given

- The shared DirectGPU fixture (`/physics/suppressReadback = true`, default
  ordinal), `links_chain_sample_gpu.usda` attached and warmed up.
- A binding on `/World/articulation` for
  `OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32` — GPU-resident and
  writable, so one binding covers both directions.
- A host source holding a known value and a zeroed host destination.

## When

- Every context is popped off the thread (`ScopedCudaContextDetach`).
- `ovphysx_write_tensor_binding`, then `ovphysx_read_tensor_binding`.
- The stack is restored.

## Then

- The write succeeds (REQ AC-2) and the read succeeds (REQ AC-1).
- The destination holds the written value, so staging was addressable by the
  DirectGPU kernels and not merely allocated without error (REQ AC-1, AC-2).
- No context is current after the write and again after the read, so neither
  call stranded a push (REQ AC-4).
- AC-3 is exercised on the write path: `release()` runs after
  `stageTensorForWrite` has popped its push, so the staging buffer is freed with
  nothing current on the caller's thread — "whatever the caller has current by
  then". Both staging buffers are function-local and are released when the read
  and the write return; binding destruction frees a different buffer. The test
  asserts nothing about the free, and `release()` discards `memFree`'s status, so
  a failed free is not detectable here.

## Not covered here

AC-5 has no reachable trigger: a GPU-resident view always carries a context, so
the guard is defensive rather than a state a test can construct.
