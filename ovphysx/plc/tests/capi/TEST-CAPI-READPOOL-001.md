<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-READPOOL-001
maps_to: REQ-CAPI-READPOOL-001
type: unit
---

## Scenario

The ovstage read-pool budget key round-trips through the public C config API: a
value set through the typed entry helper is read back through the int32 getter,
and 0 (disable) is accepted like any other value. The retention behaviour itself
is REQ-READ-POOL-001's and is not exercised here.

## Given

- A C unit-test process with the shared instance available.
- The typed entry helper `ovphysx_config_entry_ovstage_read_pool_max_mb` and the
  key `OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB`.

## When

- The current value is read and cached, a positive budget (64) is applied through
  the typed entry, and the getter is called.
- 0 is applied through the typed entry and the getter is called again.
- The original value is restored.

## Then

- The getter reports 64 after the positive set (REQ AC-1, AC-2) — the typed entry
  and getter agree, proving both map to `/physics/ovstageReadPoolMaxMB`.
- The getter reports 0 after the disable set (REQ AC-2) — 0 is an accepted,
  round-tripping value, not rejected.
- Every set and get returns `OVPHYSX_API_SUCCESS` (REQ AC-3, the generic int32
  handling). The out-of-range-key and null-`out` error paths are covered generally
  by the other int32-key cases in the same suite and are not duplicated per key.

## Coverage

- `ovphysx/tests/c_unittests/test_global_settings.cpp` — `TypedConfig.OvstageReadPoolMaxMbRoundTrip`.

## Not covered

- The retention behaviour (default 256, disable-on-non-positive, admission and
  trim) is REQ-READ-POOL-001's and is exercised by the ovruntime device-read
  suites, not here.
