<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-READPOOL-001
maps_to: REQ-CAPI-READPOOL-001
type: unit
---

## Scenario

The ovstage read-pool budget key round-trips through the public scoped C config
API, and 0 (disable) is accepted like any other int32-backed value. The
retention behaviour itself is REQ-READ-POOL-001's and is not exercised here.

## Given

- A C unit-test process with the shared instance available.
- The key `OVPHYSX_GLOBAL_CONFIG_OVSTAGE_READ_POOL_MAX_MB`.

## When

- A positive budget (64) is applied as an int64 tagged value and the getter is
  called.
- A wrong tag and an int64 value above `INT32_MAX` are rejected.
- The original value is restored.

## Then

- The getter reports 64 after the positive set (REQ AC-1, AC-2), proving the
  scoped key maps to `/physics/ovstageReadPoolMaxMB`.
- Wrong-tag and out-of-backend-range writes return
  `OVPHYSX_API_INVALID_ARGUMENT`, and the getter still reports 64 (REQ AC-3).

## Coverage

- `ovphysx/tests/c_unittests/test_global_settings.cpp` -
  `ScopedConfig.GlobalTypeRangeAndFailureAtomicity`.

## Not covered

- The retention behaviour (default 256, disable-on-non-positive, admission and
  trim) is REQ-READ-POOL-001's and is exercised by the ovruntime device-read
  suites, not here.
