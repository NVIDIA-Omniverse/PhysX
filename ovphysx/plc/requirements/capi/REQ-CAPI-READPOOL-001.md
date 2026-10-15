<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-READPOOL-001
title: C Config Surface for the ovstage Read-Buffer Pool Budget
status: implemented
owner: ovphysx
---

## Description

The ovstage output read retains its device and pinned column buffers in a
per-context pool bounded by a retention budget. Hosts must be able to set that
budget through the public C API without reaching for a raw Carbonite path.

This requirement owns the C configuration SURFACE for the budget: the
`OVPHYSX_GLOBAL_CONFIG_OVSTAGE_READ_POOL_MAX_MB` process-global key and its
mapping onto the process-wide `/physics/ovstageReadPoolMaxMB` Carbonite setting
that the omni.physx runtime reads. The value is carried by
`ovphysx_config_value_t` as int64, validated to fit the backend int32 storage,
set through `ovphysx_set_global_config`, and read through
`ovphysx_get_global_config`.

It does NOT own what the budget MEANS — the default of 256 MiB, the retain/free
admission and trim behaviour, and that 0 or a negative value disables the pool
are the allocator's contract, owned by
[REQ-READ-POOL-001](../../../ovruntime/plc/requirements/output/REQ-READ-POOL-001.md)
in the runtime. The value is forwarded to the setting unchanged; its meaning is
interpreted there.

## Acceptance Criteria

- AC-1: `OVPHYSX_GLOBAL_CONFIG_OVSTAGE_READ_POOL_MAX_MB` is a member of
  `ovphysx_global_config_key_t`. Setting an int64 tagged value for it routes to
  `/physics/ovstageReadPoolMaxMB`.
- AC-2: A value applied through `ovphysx_set_global_config` is read back
  verbatim by `ovphysx_get_global_config`.
  Because 0 and negative values are valid inputs (they disable the pool per
  REQ-READ-POOL-001), they round-trip like any other int32 rather than being
  rejected.
- AC-3: The key goes through the same tagged-value handling as other
  int64-backed global keys: a wrong tag, unknown key, null getter output, or
  value outside the backend int32 range is rejected with
  `OVPHYSX_API_INVALID_ARGUMENT`. The budget accepts the full int32 range.
- AC-4: The setting is process-global and sticky, as Carbonite settings are;
  unset, the runtime uses its documented default (256). The value seen by the
  runtime is the last one written to `/physics/ovstageReadPoolMaxMB`.

## Test References

- TEST-CAPI-READPOOL-001
- ovphysx/tests/c_unittests/test_global_settings.cpp (`ScopedConfig.GlobalTypeRangeAndFailureAtomicity`)

## Code References

- ovphysx/include/ovphysx/ovphysx_types.h (`OVPHYSX_GLOBAL_CONFIG_OVSTAGE_READ_POOL_MAX_MB`)
- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_set_global_config`, `ovphysx_get_global_config`)
- ovphysx/src/ovphysx/ovphysx.cpp (scoped-key mapping to `/physics/ovstageReadPoolMaxMB`)

## Dependencies

- REQ-READ-POOL-001 — the runtime allocator that reads the setting and owns the
  retention behaviour (default, disable-on-non-positive, admission and trim).
