<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-NVTX-001
title: NVTX Profiling Opt-In for the Public API
status: implemented
owner: ovphysx
---

## Description

Hosts profiling an ovphysx workload with NVIDIA Nsight Systems must be able to
see which ovphysx API call each span of wall time belongs to, without rebuilding
the library. NVTX instrumentation is therefore compiled into release builds and
shipped wheels, and emitting it is a runtime opt-in that is off by default.

The opt-in has two equivalent forms: the `OVPHYSX_NVTX` environment variable,
which needs no change to the host application, and the
`OVPHYSX_CONFIG_NVTX_ENABLED` config entry on `ovphysx_create_args` (Python:
`PhysXConfig.nvtx_enabled`), which a host can set programmatically. Both resolve
to the process-wide `/physics/nvtxEnabled` Carbonite setting, so the omni.physx
runtime observes the same decision when it creates the PhysX SDK.

The ranges ovphysx itself emits carry the name of the C entry point and live in
the `ovphysx` NVTX domain. The PhysX SDK's own profile zones are a separate
concern of the runtime (see REQ-SIM-NVTX-001) and appear in the `PhysX` domain.

This requirement covers the opt-in and the instrumented surface. It does not
cover which zones the PhysX SDK emits, nor Nsight capture itself.

## Acceptance Criteria

- AC-1: In a process that has not previously enabled it, with neither
  `OVPHYSX_NVTX` set nor the config entry supplied, NVTX ranges are not emitted
  and `ovphysx_get_global_config_bool(OVPHYSX_CONFIG_NVTX_ENABLED, out)` writes
  false. `/physics/nvtxEnabled` is process-global and sticky, as Carbonite
  settings are: once enabled, a later instance created with no config entry and no
  environment variable inherits the enabled state. Turning it back off is explicit
  -- `OVPHYSX_NVTX=0` or a false config entry -- and both are honoured.
- AC-2: Supplying `OVPHYSX_CONFIG_NVTX_ENABLED` as true on
  `ovphysx_create_args` enables emission, and the same getter reports true after
  instance creation.
- AC-3: When `OVPHYSX_NVTX` is present before instance creation, its boolean value
  is written through to `/physics/nvtxEnabled` in both directions: a value other
  than `0` / `false` enables emission, and `0` / `false` disables it even if an
  earlier instance in the same process had enabled it. The getter reflects the
  result in either case, without a config entry being supplied.
- AC-4: Enabling NVTX changes no simulation result and no API status: an
  otherwise identical run enabled and disabled produces the same state.
- AC-5: The instrumented entry points are `ovphysx_step`, `ovphysx_step_sync`,
  `ovphysx_step_n_sync`, `ovphysx_wait_op`, `ovphysx_attach_ovstage`,
  `ovphysx_clone`, `ovphysx_create_tensor_binding`,
  `ovphysx_read_tensor_binding`, `ovphysx_write_tensor_binding`, and
  `ovphysx_write_tensor_binding_masked`; each opens one range named after itself
  in the `ovphysx` domain for the duration of the call.
- AC-6: `PhysXConfig.nvtx_enabled` rejects a non-bool value with `TypeError`,
  and passing `/physics/nvtxEnabled` through `carbonite_overrides` instead
  raises `ValueError` for conflicting with the typed field.
- AC-7: The decision to emit is re-read on every instrumented call, not latched
  the first time a call site executes. A process that creates an instance with
  NVTX off and a later one with it on emits ranges for the later instance from
  every instrumented entry point, including those already executed while off.

## Test References

- TEST-CAPI-NVTX-001

## Code References

- ovphysx/include/ovphysx/ovphysx_types.h (`OVPHYSX_CONFIG_NVTX_ENABLED`)
- ovphysx/include/ovphysx/ovphysx_config.h (`ovphysx_config_entry_nvtx_enabled`)
- ovphysx/src/include/internal/Nvtx.h
- ovphysx/src/ovphysx/Nvtx.cpp
- ovphysx/src/ovphysx/ovphysx.cpp (`s_boolKeyPaths`, `resolveEnabled` call before
  `loadPhysxPlugins`, instrumented step / wait / attach / clone entry points)
- ovphysx/src/ovphysx/ovphysxTensorBinding.cpp (instrumented tensor binding
  entry points)
- ovphysx/python/ovphysx/config.py (`PhysXConfig.nvtx_enabled`)
- ovphysx/python/ovphysx/types.py (`ConfigBool.NVTX_ENABLED`)
- ovphysx/tests/c_unittests/test_nvtx_profiling.cpp
- ovphysx/tests/python_tests/lifecycle_tests/test_nvtx_config.py
- ovphysx/tests/python_tests/cpu_tests/test_physxconfig_validation.py

## Dependencies

- REQ-SIM-NVTX-001 - the omni.physx runtime side, which reads the same setting
  and emits the PhysX SDK zones.
