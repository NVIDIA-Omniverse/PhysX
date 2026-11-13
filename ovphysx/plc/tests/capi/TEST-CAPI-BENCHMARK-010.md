<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-BENCHMARK-010
maps_to: REQ-CAPI-BENCHMARK-010
type: integration
---

## Scenario

Every equivalent-workload row runs on a DirectGPU host and publishes a timing only after its setup validation passes.
Source inspection shows each pair doing the same work on the same data placement.

## Given

- A benchmark-enabled build and installed SDK, a CUDA device, and `PHYSX_USD_SCHEMAS_ROOT` naming the installed
  PhysX schemas.
- The committed `humanoid_envs_gpu.usda` and `humanoid.usda`.

## When

- Each `Lab.humanoid_8192_*` row runs in its own process with `--forceGpu --directGpu --hidden`, its exact
  `--filter` and the default 5 runs of 20 samples.
- Each `OutputWrite.writedevice_*_8192_gpu` row and its `OutputWrite.tensorbinding_*_8192_gpu` counterpart run the
  same way.
- Inspect `LabHumanoid.cpp` and the device-sourced paths of `OutputWriteBase`.

## Then

- Every row loads, validates and publishes one positive timing; each Humanoid row prints its PHASES line; every
  process exits zero (REQ AC-1, AC-2, AC-3, AC-4). Validation passing means each control step row's force reached
  PhysX as the last action, and each reset row restored exactly the selected environments' positions and velocities
  (REQ AC-4).
- The device-sourced rows pass the warm-up check that each group is one column of the payload's size (REQ AC-1).
- Source inspection confirms that the control step rows run the same sequence with the same actions, end each sample
  with a stream or blocking-call wait, and capture only the commit and the reads; that the reset rows build their
  masks from each row's own prim paths and write the same four quantities; and that a failure throws or records a
  row failure rather than returning a timing (REQ AC-2, AC-3, AC-4).

Not covered: CI runs none of these rows (they are hidden and need DirectGPU), and no automated case injects a
validation failure to show the process failing.
