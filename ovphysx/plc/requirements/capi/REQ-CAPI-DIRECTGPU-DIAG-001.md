<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-DIRECTGPU-DIAG-001
title: DirectGPU Startup Mode Diagnostic
status: implemented
owner: ovphysx
---

## Description

DirectGPU (the PxDirectGPUAPI read/write path) is opted into by setting the
Carbonite override `/physics/suppressReadback=true`, carried on the legacy
PhysXConfig / create_args config entries — there is no scoped GlobalConfigKey for
it (REQ-CAPI-CONFIG-001). The setting is process-wide: the last value written
applies to every instance, and a GPU scene reads it when it is created at attach.
ovphysx never writes the setting itself.

CarboniteLoader logs the `suppressReadback` value it observes once per process,
during loader bootstrap. That bootstrap runs *before* an instance's create-config
overrides are applied, so on the documented opt-in path it always sees the value
unset. Read on its own, that line suggests DirectGPU is off even for an instance
that goes on to enable it, because the override is applied — and separately
logged by the settings layer — immediately afterwards. A developer checking the
startup log to confirm a DirectGPU instance must not be misled by that
pre-override snapshot.

Baseline requirement for a startup diagnostic that reports the
`suppressReadback` value as it stands after each instance's create-config, rather
than only the process bootstrap snapshot. The opt-in itself is unchanged and
works; this requirement is scoped to the diagnostic, not to DirectGPU execution.

## Acceptance Criteria

- AC-1: **Effective mode at instance creation.** After an instance's
  create-config entries are applied and before its PhysX plugins load, ovphysx
  logs the `/physics/suppressReadback` value as it then stands and the DirectGPU
  mode it implies, at INFO, once per instance creation. The value is read live,
  so a create-config override is reflected whether or not that override set this
  particular key. The line states that the setting is process-wide and read at
  attach, since the value — and so the mode a GPU scene is created in — can still
  change before this instance attaches.
- AC-2: **Bootstrap snapshot is labelled, not a verdict.** The once-per-process
  CarboniteLoader line that reports `suppressReadback` does not state a final
  DirectGPU verdict: the unset case identifies the value as the bootstrap
  (pre-override) snapshot and defers the effective mode to the per-instance line,
  and the host-set case is labelled "at bootstrap".
- AC-3: **Diagnostic only.** Neither line writes `/physics/suppressReadback` or
  changes DirectGPU selection; the create-config override stays the sole opt-in.
  A no-opt-in instance still reports a useful effective line
  (`suppressReadback=false`, DirectGPU disabled), preserving the diagnostic when
  the application supplies no override.

## Test References

- [TEST-CAPI-DIRECTGPU-DIAG-001](../../tests/capi/TEST-CAPI-DIRECTGPU-DIAG-001.md) —
  a `suppressReadback=true` create-config instance logs the effective line as
  enabled; a no-override instance logs it as disabled; both on a CPU instance,
  since instance creation (where the line fires) needs no GPU.

AC-2 is inspection-verified. The CarboniteLoader bootstrap line fires once per
process, before the c_unittests shared instance exists, so a later test cannot
observe it; the per-instance AC-1 line is the automated one.

## Code References

- ovphysx/src/ovphysx/ovphysx.cpp (the per-instance effective-mode INFO line,
  emitted after `applyCreateConfigEntries`)
- ovphysx/src/CarboniteLoader/CarboniteLoader.cpp (the bootstrap `suppressReadback`
  snapshot line, labelled pre-override)
- ovphysx/tests/c_unittests/test_directgpu_startup_diagnostic.cpp

## Dependencies

- [REQ-CAPI-LOG-001](REQ-CAPI-LOG-001.md) — the log callback and delivery the INFO
  line rides on
- [REQ-CAPI-CONFIG-001](REQ-CAPI-CONFIG-001.md) — create-config entries, which
  carry the legacy `suppressReadback` override
