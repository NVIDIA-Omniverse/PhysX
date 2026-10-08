<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-BENCHMARK-003
maps_to: REQ-CAPI-BENCHMARK-003
type: integration
---

## Scenario

The Cartpole creation row reports exactly one fresh-process measurement of
the specified five-binding sequence.

## Given

- An ovphysx build configured with `OVPHYSX_BUILD_BENCHMARKS=ON`.
- The generated `cartpole_probe.usda` fixture.
- A CUDA-capable host and a fresh benchmark process launched with
  `--forceGpu --directGpu --hidden`.

## When

- The exact `Probe.cartpole_4096_tensor_binding_create` filter runs once.
- The same row is invoked without DirectGPU and with step or run overrides
  that request another measurement.

## Then

- The DirectGPU invocation publishes one positive
  `Probe.cartpole_4096_tensor_binding_create_GPU` row whose timer covers the
  five ordered create/specification pairs and excludes setup, caller buffers,
  tensor I/O, physics steps, validation, and destruction (REQ AC-1).
- The row validates all five float32 `[4096, 2]` specifications and Cartpole
  topology after timing, then destroys the bindings (REQ AC-2).
- Non-DirectGPU and second-measurement requests publish no report row and exit
  nonzero (REQ AC-1, AC-2).
