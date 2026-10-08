<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-READPOOL-001
title: Python Config Surface for the ovstage Read-Buffer Pool Budget
status: implemented
owner: ovphysx
---

## Description

`PhysXConfig` and the runtime config setters expose the ovstage read-buffer pool
budget to Python, mirroring the C surface (REQ-CAPI-READPOOL-001) one-for-one and
validating input before it crosses the ABI.

This requirement owns the Python SURFACE: the `ConfigInt32.OVSTAGE_READ_POOL_MAX_MB`
enum member (kept in sync with the C enum), the `PhysXConfig.ovstage_read_pool_max_mb`
typed field, and the runtime `set_config_int32` / `get_config_int32` round-trip for
the key. The retention behaviour is the allocator's (REQ-READ-POOL-001); the value
is forwarded to `/physics/ovstageReadPoolMaxMB` unchanged.

## Acceptance Criteria

- AC-1: `ConfigInt32.OVSTAGE_READ_POOL_MAX_MB` exists and its integer value equals
  the C `OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB`, verified by the C/Python enum
  sync check so the two cannot drift.
- AC-2: `PhysXConfig(ovstage_read_pool_max_mb=v)` accepts any `int` — including 0,
  negative, and `INT32_MAX`, which are valid because 0/negative disable the pool —
  and rejects a non-`int` (`bool`, `str`, `float`) with `TypeError`, like every
  sibling int field. Passing `/physics/ovstageReadPoolMaxMB` through
  `carbonite_overrides` instead raises `ValueError` for conflicting with the typed
  field.
- AC-3: `PhysX.set_config_int32(ConfigInt32.OVSTAGE_READ_POOL_MAX_MB, v)` followed
  by `get_config_int32(...)` round-trips the value at runtime through the C ABI,
  for a positive budget and for 0.

## Test References

- TEST-PYTHON-READPOOL-001
- ovphysx/tests/python_tests/test_types_sync.py (`test_config_int32_and_string_values_match_c_header`)
- ovphysx/tests/python_tests/cpu_tests/test_physxconfig_validation.py (accepted/rejected values)
- ovphysx/tests/python_tests/cpu_tests/test_physx_advanced.py (`test_set_config_int32_then_get_config_int32_ovstage_read_pool`)

## Code References

- ovphysx/python/ovphysx/types.py (`ConfigInt32.OVSTAGE_READ_POOL_MAX_MB`)
- ovphysx/python/ovphysx/config.py (`PhysXConfig.ovstage_read_pool_max_mb`, `_OVSTAGE_READ_POOL_MAX_MB`, `_FIELD_TO_ENTRY`)
- ovphysx/python/ovphysx/api.py (`set_config_int32` / `get_config_int32`)

## Dependencies

- REQ-CAPI-READPOOL-001 — the C surface this mirrors.
- REQ-READ-POOL-001 — the runtime allocator that owns the retention behaviour.
