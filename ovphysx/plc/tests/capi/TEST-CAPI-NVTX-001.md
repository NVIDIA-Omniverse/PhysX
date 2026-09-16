<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-NVTX-001
maps_to: REQ-CAPI-NVTX-001
type: unit
---

## Scenario

The NVTX opt-in is off by default, can be turned on through either the config
entry or the `OVPHYSX_NVTX` environment variable, is observable through the
public config getter, and changes neither simulation results nor API statuses.
The Python `nvtx_enabled` field validates its input and refuses to be shadowed by
a raw Carbonite override.

Whether Nsight Systems records the ranges cannot be asserted from a test
process, so the tests cover the opt-in and its side-effect freedom; the timeline
itself is verified manually.

## Given

- A C unit-test process that creates an instance with no NVTX config entry and
  `OVPHYSX_NVTX` unset.
- A C unit-test process that creates an instance with
  `OVPHYSX_CONFIG_NVTX_ENABLED` set to true.
- An isolated process with `OVPHYSX_NVTX=1` set before `ovphysx_initialize`.
- A stage with a rigid body that can be stepped a fixed number of times.
- The `PhysXConfig` dataclass, without a native library (validation is pure
  Python).

## When

- `ovphysx_get_global_config_bool(OVPHYSX_CONFIG_NVTX_ENABLED, &out)` is called
  after each of the three creation paths above.
- The same stage is stepped a fixed number of times with NVTX enabled and with
  it disabled, reading body poses back through a tensor binding.
- A subset of the instrumented entry points is exercised with NVTX enabled:
  `ovphysx_step`, `ovphysx_wait_op`, `ovphysx_attach_ovstage`,
  `ovphysx_create_tensor_binding`, and `ovphysx_read_tensor_binding`.
- The whole suite runs in a single process under Nsight Systems, so that enabled
  cases follow a case that ran with NVTX off.
- `PhysXConfig(nvtx_enabled="yes")` is constructed.
- `PhysXConfig(carbonite_overrides={"/physics/nvtxEnabled": True})` is converted
  to C config entries.

## Then

- The getter writes false for the default path (REQ AC-1) and true for both the
  config-entry path (REQ AC-2) and the environment-variable path (REQ AC-3).
- After an instance enabled it via the config entry, a second instance created with
  no config entry and `OVPHYSX_NVTX=0` reports false, so the sticky setting can be
  turned back off through the environment variable alone (REQ AC-1 stickiness,
  REQ AC-3 both directions).
- The enabled and disabled runs report the same statuses and the same final
  poses within tolerance (REQ AC-4).
- The entry points this test exercises return success with NVTX enabled, so the
  added ranges do not disturb those call paths (REQ AC-5, partial). The remaining
  entry points named in AC-5 -- `ovphysx_step_sync`, `ovphysx_step_n_sync`,
  `ovphysx_clone`, `ovphysx_write_tensor_binding` and
  `ovphysx_write_tensor_binding_masked` -- are not asserted here; that each opens
  its range is verified by manual Nsight capture, since a test process cannot
  observe emitted ranges.
- `PhysXConfig(nvtx_enabled="yes")` raises `TypeError` and the
  `carbonite_overrides` form raises `ValueError` (REQ AC-6).
- Running the whole suite in one process, where the first case runs with NVTX
  off, still yields ranges for the later enabled cases: a capture shows one
  `ovphysx` range per instrumented call of the three enabled workflows and none
  from the two disabled ones (REQ AC-7). Emission is not observable in-process,
  so this is checked by capture, and the instance counts are the assertion: a
  gate latched at first execution yields zero for every site instead.
