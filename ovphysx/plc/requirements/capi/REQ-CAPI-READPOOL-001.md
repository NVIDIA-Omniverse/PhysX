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

This requirement owns the C configuration SURFACE for the budget: the int32
config key `OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB` on `ovphysx_config_int32_t`,
the typed entry helper `ovphysx_config_entry_ovstage_read_pool_max_mb`, and their
mapping onto the process-wide `/physics/ovstageReadPoolMaxMB` Carbonite setting
that the omni.physx runtime reads. The value is carried by the generic int32
config path, set through `ovphysx_create_args` or `ovphysx_set_global_config` and
read back through `ovphysx_get_global_config_int32`.

It does NOT own what the budget MEANS — the default of 256 MiB, the retain/free
admission and trim behaviour, and that 0 or a negative value disables the pool
are the allocator's contract, owned by
[REQ-READ-POOL-001](../../../ovruntime/plc/requirements/output/REQ-READ-POOL-001.md)
in the runtime. The value is forwarded to the setting unchanged; its meaning is
interpreted there.

## Acceptance Criteria

- AC-1: `OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB` is a member of
  `ovphysx_config_int32_t`, and `ovphysx_config_entry_ovstage_read_pool_max_mb(v)`
  builds an int32 config entry for it. Both route the value to
  `/physics/ovstageReadPoolMaxMB`.
- AC-2: A value applied through the typed entry (via `ovphysx_set_global_config`
  or `ovphysx_create_args`) is read back verbatim by
  `ovphysx_get_global_config_int32(OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB, out)`.
  Because 0 and negative values are valid inputs (they disable the pool per
  REQ-READ-POOL-001), they round-trip like any other int32 rather than being
  rejected.
- AC-3: The key goes through the SAME generic int32 handling as every other int32
  config key — an out-of-range key index is rejected with
  `OVPHYSX_API_INVALID_ARGUMENT`, and a null `out` to the getter is an error. No
  key-specific value validation is added: the budget accepts the full int32 range.
- AC-4: The setting is process-global and sticky, as Carbonite settings are;
  unset, the runtime uses its documented default (256). The value seen by the
  runtime is the last one written to `/physics/ovstageReadPoolMaxMB`.

## Test References

- TEST-CAPI-READPOOL-001
- ovphysx/tests/c_unittests/test_global_settings.cpp (`TypedConfig.OvstageReadPoolMaxMbRoundTrip`)

## Code References

- ovphysx/include/ovphysx/ovphysx_types.h (`OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB`)
- ovphysx/include/ovphysx/ovphysx_config.h (`ovphysx_config_entry_ovstage_read_pool_max_mb`)
- ovphysx/src/ovphysx/ovphysx.cpp (`s_int32KeyPaths` mapping to `/physics/ovstageReadPoolMaxMB`)

## Dependencies

- REQ-READ-POOL-001 — the runtime allocator that reads the setting and owns the
  retention behaviour (default, disable-on-non-positive, admission and trim).
