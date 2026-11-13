<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-DIRECTGPU-DIAG-001
maps_to: REQ-CAPI-DIRECTGPU-DIAG-001
type: integration
---

## Scenario

The per-instance effective-DirectGPU line reflects the create-config override,
not the process bootstrap snapshot. An instance created with the documented
opt-in logs the effective line as enabled; a control instance with no override
logs it as disabled. Both run on a CPU-created instance with no scene attached,
because the line fires during instance creation, after create-config is applied
and before plugins load — the point the bootstrap snapshot could not see.

`DirectGpuStartupDiagnostic.*` in
`tests/c_unittests/test_directgpu_startup_diagnostic.cpp`.

## Given

- A log callback registered at INFO (`ovphysx_set_log_callback`) that captures
  every record's message, installed before each instance is created.
- A fresh instance is created per case and destroyed before the callback is
  cleared, so only that creation's records are examined.

## When

- Treatment: an instance is created with `suppressReadback` set true via a
  create-config carbonite entry.
- Control: an instance is created with `suppressReadback` set false the same way.
  An explicit value is used rather than omitting the entry because the setting is
  process-global and instance destroy does not revert it, so a prior instance
  could leave it set; the effective line reads the same for an explicit false as
  for an unset default.

## Then

- The treatment's captured records contain the effective line naming
  `/physics/suppressReadback=true` and DirectGPU enabled (REQ AC-1); ovphysx
  never writes the setting itself, so the override is the only thing that set it
  (REQ AC-3).
- The control's captured records contain the effective line naming
  `/physics/suppressReadback=false` and DirectGPU disabled, so a non-DirectGPU
  instance still reports a usable effective mode rather than nothing (REQ AC-1,
  AC-3).
- Each case emits exactly one effective line for its creation (REQ AC-1).

## Coverage status

Implemented in `test_directgpu_startup_diagnostic.cpp`. CPU-only — instance
creation needs no CUDA device, so the case is not GPU-gated. AC-2 (the bootstrap
snapshot wording) is inspection-verified, not cased: the CarboniteLoader bootstrap
line fires once per process before any test instance exists, so a test cannot
observe it (noted in the requirement).
