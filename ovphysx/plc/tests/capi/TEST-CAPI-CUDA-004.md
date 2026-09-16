<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-CUDA-004
maps_to: REQ-CAPI-CUDA-003
type: integration
---

## Scenario

The reported configuration end to end: a DirectGPU rigid-body pose read into a
host buffer with the simulation on CUDA ordinal 1. It cannot share
TEST-CAPI-CUDA-003's setup — ordinal selection and DirectGPU are process-global,
so it needs its own instance, plus a second CUDA device that not every runner has.

`ActiveCudaGpusAttachTest.DirectGpuHostReadOnNonZeroOrdinal` in
`tests/c_unittests/test_instantiation.cpp`, run by its own harness pass
(`cuda-selection-nonzero-ordinal` in `scripts/test_cpp.cmake`). That dedicated
process is what lets the case reach ordinal 1 at all. PhysX latches its CUDA device
at the first GPU attach in the process, so an earlier attach on ordinal 0 — which
`DirectOvstageAttachPropagatesExplicitOrdinal` performs — pins the device, and the
later `/physics/cudaDevice = 1` write does not move it. The case enforces that
precondition itself rather than relying on the harness filter: before it creates
anything it checks `test_to_run_count()`, the post-filter count of tests selected
for this process, and skips when it is not alone. The device assertion in `## Then`
remains the detector for the other failure — the process is the case's own but the
ordinal still comes back wrong. DirectGPU is switched back off on every exit path
that switched it on, and both skip decisions precede that.

## Given

- A process with no earlier GPU attach: the case owns the
  `cuda-selection-nonzero-ordinal` harness pass.
- An instance created with `active_cuda_gpus = "1"` and
  `/physics/suppressReadback = true`.
- Two or more CUDA devices, counted via `deviceGetCount` through a throwaway
  default instance created and destroyed first: create validates
  `active_cuda_gpus`, so the count must be known before ordinal 1 is requested,
  and the shim is not queryable until an instance has initialized the runtime.
  The probe overrides no ordinal and never attaches, so it latches no CUDA device
  and selects no ordinal, and a skip from here leaves the process clean.
- `boxes_falling_on_groundplane_gpu.usda` attached, a binding on `/World/Cube*`
  for `OVPHYSX_TENSOR_RIGID_BODY_POSE_F32`, warmed up.
- A host destination pre-filled with NaN, so an unwritten component is
  distinguishable from a zero pose.

## When

- The thread's contexts are detached, removing the accident that decides whether
  the failure surfaces.
- `ovphysx_read_tensor_binding` into the host destination; the stack is restored.

## Then

- The binding reports `DLDevice{kDLCUDA, 1}`, so the read takes the GPU staging
  path on the selected ordinal and is not a same-device host read.
- The spec reports a non-zero element count, so the read cannot pass vacuously.
- The read succeeds; on failure the assertion reports `ovphysx_get_last_error()`
  (REQ AC-1). Without the fix this configuration fails at the staging allocation
  with `cuda_status=201` (`CUDA_ERROR_INVALID_CONTEXT`), because the test detaches
  every context and the DirectGPU gather never runs. The reported gather fault
  (`cuda_status=700`) has no automated case: `IOptionalCuda` exposes no way to
  create a foreign context.
- Every pose component is finite, so the gather reached staging (REQ AC-1).
- No context is current after the read, so it stranded no push (REQ AC-4).
- AC-3 is exercised inside the read: `readStaging` is function-local and is
  released before `bindingCtxPush` pops, so the free runs under the context the
  buffer was allocated in. Binding destruction frees a different buffer, and the
  test asserts nothing about the free — `release()` discards `memFree`'s status.
- Binding, attachment, and instance destroy cleanly.
- Clearing `/physics/suppressReadback` succeeds.
