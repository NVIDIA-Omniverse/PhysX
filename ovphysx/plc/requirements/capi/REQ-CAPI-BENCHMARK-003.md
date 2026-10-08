<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-BENCHMARK-003
title: Cartpole First-Use Tensor Binding Creation Benchmark
status: implemented
owner: ovphysx
---

## Description

The hidden DirectGPU Cartpole benchmark contains a dedicated row for the
first creation and specification lookup of the five tensor bindings used by
the 4,096-environment control loop. The supported measurement runs under its
exact filter in a fresh process.

## Acceptance Criteria

- AC-1: **Measured boundary.**
  `Probe.cartpole_4096_tensor_binding_create` requires `--forceGpu` and
  `--directGpu`. Its timer starts immediately before the first binding create
  and stops after the fifth specification lookup. Each create is followed by
  its specification lookup, in this order: actuation force, position target,
  velocity target, DOF position, and DOF velocity. Internal work and
  allocation performed by those create calls are measured. Scene load,
  cloning 4,095 environments, `ovphysx_warmup()`, caller CUDA data-buffer
  allocation, tensor I/O, physics stepping, validation, destruction, and all
  other work are outside the timer.
- AC-2: **Fresh-process one-shot.** The supported invocation launches the
  exact creation-row filter in a fresh process. The row performs one timed
  sequence, keeps all five bindings alive until the timer has stopped,
  validates float32 `[4096, 2]` specifications and articulation topology after
  timing, and destroys the bindings during teardown. Step or run overrides
  that request a second measurement fail without publishing a row.

## Test References

- [TEST-CAPI-BENCHMARK-003](../../tests/capi/TEST-CAPI-BENCHMARK-003.md)

## Code References

- AC-1: `ovphysx/tests/benchmarks/benchmarks/LabCartpole.cpp`
- AC-2: `ovphysx/tests/benchmarks/benchmarks/LabCartpole.cpp` and
  `ovphysx/tests/benchmarks/Harness.cpp`
- Contract coverage: `ovphysx/scripts/test_benchmark_contract.cmake`,
  `ovphysx/tests/benchmarks/CMakeLists.txt`,
  `ovphysx/tests/benchmarks/tests/verify_benchmark_diagnostics.py`, and
  `ovphysx/tests/benchmarks/tests/test_verify_benchmark_diagnostics.py`

## Dependencies

- [REQ-CAPI-BENCHMARK-002](REQ-CAPI-BENCHMARK-002.md) defines the one-sample
  timing sidecar used to publish this row's bounded diagnostics.
