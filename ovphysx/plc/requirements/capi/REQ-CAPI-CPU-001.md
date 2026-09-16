<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-CPU-001
title: Process-Wide Hard CPU-Only Mode Observability
status: implemented
owner: ovphysx
---

## Description

Hard CPU-only mode is a process-wide deployment policy established by
`ovphysx_set_cpu_mode(true)` before the first instance, or by setting
`OVPHYSX_DISABLE_GPU` before OVPhysX initialization. Callers must be able to
read that effective policy without inferring it from later simulation
behaviour, and instance creation must emit an INFO line naming the policy.

This requirement covers the hard CPU-only policy only. It does not cover
per-scene USD `physxScene:enableGPUDynamics`, CUDA ordinal selection via
`active_cuda_gpus`, or attach-time resolved CPU versus GPU dynamics / ordinal
outcome (see ADR-0011). Those remain a separate observability follow-up;
hard-policy getters and the create INFO line do not replace a host that still
needs attach-time confirmation of scene dynamics.

`OVPHYSX_DISABLE_GPU` is read live before `ovphysx_initialize` (and again
after `ovphysx_shutdown` until the next initialize). `ovphysx_initialize`
latches the env contribution for that initialized interval so an
observational pre-init `ovphysx_get_cpu_mode` cannot permanently miss a
later setenv before init.

## Acceptance Criteria

- AC-1: `ovphysx_get_cpu_mode(out)` returns success and writes true when hard
  CPU-only mode is active (`ovphysx_set_cpu_mode(true)` succeeded or
  `OVPHYSX_DISABLE_GPU` is active per the live/latched rules above), and
  writes false when neither input is active.
- AC-2: `ovphysx_get_cpu_mode(NULL)` returns `OVPHYSX_API_INVALID_ARGUMENT`.
- AC-3: A successful `ovphysx_create_instance()` emits an INFO log line that
  includes `process_cpu_only=` with the effective hard CPU-only policy.
- AC-4: When not hard CPU-only, the create INFO line reports empty
  `active_cuda_gpus` as `no_override` and an explicit `"-1"` request as
  `-1` (not `auto` for empty).
- AC-5: **The no-CUDA-driver guarantee, and the two things outside ovphysx that can
  break it.** Under hard CPU-only mode ovphysx's own code loads no CUDA driver for
  the process lifetime: ovphysx neither ships nor requests the Fabric-era
  `omni.gpucompute-cuda.plugin`, and every
  `cudaShim::isCudaAvailable()` caller on the create/attach path — the shim being the
  only place ovphysx `dlopen`s the driver — sits behind `isCpuMode()`.

  Two dependencies outside that boundary can still open it, and the guarantee is
  stated against ovphysx's own code precisely because neither is ovphysx's to fix.

  **Warp.** The Python read and write frontends expose `warp.array` tensors, and
  building the first array initializes the Warp runtime, which opens the driver when
  Warp itself was **built** with CUDA — a property of the installed Warp, not of this
  flag, and one Warp exposes no runtime switch for. A read or write that resolves no
  groups never reaches array construction and stays driverless on any Warp build.

  **ovstage.** Loading a stage pulls in ovstage's own
  `bin/plugins/gpucompute/libomni.gpucompute-cuda.plugin.so`, which `dlopen`s
  `libcuda.so.1` at runtime — it carries no `DT_NEEDED` on the driver, so this is the
  plugin's own doing rather than link time. ovphysx neither ships nor loads that
  plugin. Until ovstage offers a way to suppress it, a deployment that needs the
  process-wide guarantee cannot get it through `ovphysx_set_cpu_mode` alone, and the
  public wording must not imply otherwise.

  Because of this, coverage of the guarantee is stated as a DELTA across the operation
  under test rather than as an absolute "never loaded". An absolute assertion is a test
  of ovstage: it fails on the stage load, before reaching the step it means to cover,
  and reads as an ovphysx defect. The one exception is process start through instance
  creation, which is entirely ovphysx's and is asserted absolutely.

  Two further obligations follow. The Python conversions remain driverless on a
  CPU-only Warp build, so `read()` and `write()` keep exposing `warp.array` tensors and
  nothing is given up; and the frontend shall detect a CUDA-enabled Warp under CPU-only
  mode and say so once, rather than leaving it to be discovered from a driver trace.
  The detection must itself be driverless — `wp_is_cuda_enabled()` is readable off the
  loaded library without initializing the runtime.

  Written because `cpu_tests/test_cpu_mode_cuda_driver.py` asserts exactly this and
  no criterion stood behind it; it had been annotated against
  `REQ-PYTHON-READ-001` AC-3, which is zero-copy aliasing.

## Test References

- TEST-CAPI-CPU-001

## Code References

- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_get_cpu_mode`)
- ovphysx/src/ovphysx/ovphysx.cpp (`ovphysx_get_cpu_mode`, `createInstanceInternal`,
  env latch in `ovphysx_initialize` / clear in `ovphysx_shutdown`)
- ovphysx/tests/c_unittests/test_instantiation.cpp (`CpuModeAPI`,
  `ActiveCudaGpusAttachTest.CreateLogEmptyVsMinusOneAcrossRecreate`)
- ovphysx/tests/c_unittests/test_cpu_no_cuda_context.cpp (`CpuNoCudaContextGpuTest`)
- ovphysx/tests/python_tests/cpu_tests/test_cpu_mode.py
- ovphysx/tests/python_tests/cpu_tests/test_cpu_mode_cuda_driver.py
- ovphysx/tests/python_tests/lifecycle_tests/test_cpu_mode_env_observability.py

## Dependencies

- [ADR-0011](../../../ovruntime/plc/adr/ADR-0011-ovphysx-device-execution-policy.md) -
  OVPhysX device selection, precedence, and attachment lifecycle policy
