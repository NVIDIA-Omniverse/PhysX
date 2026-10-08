<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-CPU-001
maps_to: REQ-CAPI-CPU-001
type: unit
---

## Scenario

The public `ovphysx_get_cpu_mode` query reports the effective hard CPU-only
policy, rejects a null out-parameter, and instance creation names the policy
in its INFO create line. Empty versus `"-1"` `active_cuda_gpus` create-log
labels follow ADR-0011. Fresh subprocesses also pin the first non-empty
`warp.array` construction through both the read and write frontends.

## Given

- A process where either `OVPHYSX_DISABLE_GPU` is set (CPU CTest pass) or
  `ovphysx_set_cpu_mode(true)` has succeeded (isolated
  `CpuNoCudaContextGpuTest` process / Python `cpu_tests` process).
- A fresh Python lifecycle subprocess with `OVPHYSX_DISABLE_GPU` unset for
  the default-false and pre-init setenv latch sequence.
- An isolated `ActiveCudaGpusAttachTest` process without
  `OVPHYSX_DISABLE_GPU` for empty versus `-1` create-log labels.
- Fresh Python subprocesses that construct the first non-empty CPU `warp.array`
  through `PhysX.read()` and `PhysX.write()` respectively.

## When

- `ovphysx_get_cpu_mode(NULL)` is called.
- `ovphysx_get_cpu_mode(&out)` is called after hard CPU-only mode is active.
- `ovphysx_get_cpu_mode(&out)` is called before `set_cpu_mode` in the
  isolated CPU-no-CUDA process (env unset).
- Python: `get_cpu_mode()` then setenv `OVPHYSX_DISABLE_GPU` then
  `PhysX()` / initialize in a fresh lifecycle subprocess.
- Python `PhysX.get_cpu_mode()` is called in the `cpu_tests` process after
  `PhysX.set_cpu_mode(True)`.
- An instance is created successfully (INFO logging enabled by the host as
  needed), including empty then `"-1"` then empty recreate under log
  capture.
- In separate fresh subprocesses, a non-empty CPU read group and a non-empty
  CPU write group are fetched after hard CPU-only mode is enabled.

## Then

- The null out-parameter call returns `OVPHYSX_API_INVALID_ARGUMENT` (REQ AC-2).
- The successful query writes true when hard CPU-only mode is active (REQ AC-1).
- The isolated CPU-no-CUDA process reports false before `set_cpu_mode` when
  the env is unset (REQ AC-1 inactive case).
- The lifecycle probe reports false, then true after setenv before
  initialize, and true after `PhysX()` even once `OVPHYSX_DISABLE_GPU` is
  cleared post-init (REQ AC-1 latch sequence).
- Python `PhysX.get_cpu_mode()` returns `True` after the CPU-mode session
  fixture has applied `set_cpu_mode(True)` (REQ AC-1).
- After Carbonite is available, a recreate under INFO log capture finds
  `process_cpu_only=true` in the create INFO line (REQ AC-3;
  `CpuNoCudaContextGpuTest`).
- Create INFO finds `active_cuda_gpus=no_override` for empty input and
  `active_cuda_gpus=-1` for explicit `"-1"`, including empty recreate after
  `-1` (REQ AC-4).
- For both the read and write frontend, constructing the first non-empty CPU
  array leaves CUDA-driver mapping unchanged on a CPU-only Warp build. On a
  CUDA-enabled build it opens the driver when that effect remains observable;
  a driver already opened by ovstage makes that arm inconclusive (REQ AC-5).
