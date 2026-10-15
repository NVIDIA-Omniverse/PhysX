<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-READPOOL-001
maps_to: REQ-PYTHON-READPOOL-001
type: unit
---

## Scenario

The Python read-pool budget surface matches the C enum, preserves the legacy
typed field, and round-trips through module-level global configuration.

## Given

- The C/Python config-enum parity check over `GlobalConfigKey`.
- The `PhysXConfig` dataclass (validation is pure Python, no native library).
- A live `PhysX` instance while exercising runtime global set/get.

## When

- The enum-sync test compares every `GlobalConfigKey` member to the parsed C header.
- `PhysXConfig(ovstage_read_pool_max_mb=v)` is constructed for accepted values
  (`0`, `256`, `-1`, `INT32_MAX`) and rejected ones (non-`int`), and the
  `carbonite_overrides` conflict form is converted to C entries.
- `set_global_config(GlobalConfigKey.OVSTAGE_READ_POOL_MAX_MB, 64)` then
  `get_global_config(...)` is run, and again with `0`, restoring the original.

## Then

- `GlobalConfigKey.OVSTAGE_READ_POOL_MAX_MB` equals the C key value, so the enums
  cannot drift (REQ AC-1).
- The accepted values construct and store; a non-`int` raises `TypeError`; the
  `carbonite_overrides` form raises `ValueError` (REQ AC-2).
- `get_global_config` returns `64` then `0` after the respective sets (REQ AC-3) -
  0 (disable) round-trips like any other value.

## Coverage

- `ovphysx/tests/python_tests/test_types_sync.py` -
  `test_scoped_config_values_match_c_header` (AC-1).
- `ovphysx/tests/python_tests/cpu_tests/test_physxconfig_validation.py` -
  accepted legacy `ovstage_read_pool_max_mb` values incl. `-1` / `INT32_MAX`,
  and the shared type-rejection cases (AC-2).
- `ovphysx/tests/python_tests/lifecycle_tests/test_config_global.py` -
  `test_scoped_config_global_and_instance_paths` (AC-3).

## Not covered

- The retention behaviour is REQ-READ-POOL-001's; a live config→setting round-trip
  through `PhysX(config=...)` belongs alongside the other typed-config init tests
  and is not required for this surface.
