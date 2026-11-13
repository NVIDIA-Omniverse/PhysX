<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-CONFIG-001
title: Scoped Typed Configuration
status: implemented
owner: ovphysx
---

## Description

OVPhysX separates process-global configuration from values retained by one
instance. Both scopes use `ovphysx_config_value_t`; keys encode scope in
non-overlapping numeric ranges.

| Key | Scope | Type | Default | Valid values | Mutation phase |
|---|---|---|---|---|---|
| `DISABLE_CONTACT_PROCESSING` | global | bool | false | bool | runtime |
| `COLLISION_CONE_CUSTOM_GEOMETRY` | global | bool | true | bool; stored as the inverse of `/physics/collisionApproximateCones` | no stage attached |
| `COLLISION_CYLINDER_CUSTOM_GEOMETRY` | global | bool | true | bool; stored as the inverse of `/physics/collisionApproximateCylinders` | no stage attached |
| `NUM_THREADS` | global | int64 | 8 | 0..255 | no stage attached |
| `SCENE_MULTI_GPU_MODE` | global | int64 | 0 | 0..2 | no stage attached |
| `OVSTAGE_READ_POOL_MAX_MB` | global | int64 | 256 | backend int32 range; <=0 disables | runtime |
| `PROFILER_BACKEND` | global | int64 | NONE | `ovphysx_profiler_backend_t` | before shared runtime start |
| `COOKED_COLLIDER_CACHE_DIRECTORY` | global | string | unset; process-private cache | no embedded NUL | before shared runtime start |
| `PROFILER_OUTPUT_PATH` | global | string | unset; dated working-directory file | no embedded NUL | before shared runtime start |
| `CPU_ONLY` | global | bool | false | false/true; true is sticky | before shared runtime start |
| `LOG_LEVEL` | global | int64 | WARNING | `ovphysx_log_level_t` | runtime |
| `DEFAULT_LOG_OUTPUT` | global | bool | true | bool | runtime |
| `OVD_RECORDING_CAPABLE` | global | bool | false | bool | before shared runtime start |
| `ASYNC_EXECUTION` | global | bool | true | bool | runtime; future instances only |
| `COMPILED_STAGE_CACHE` | global | int64 | AUTOMATIC | `ovphysx_compiled_stage_cache_mode_t` | runtime; read by each later attach |
| `ACTIVE_CUDA_GPUS` | instance | string | unset; no ordinal override | supported ordinal patterns | after create, before attach |

`OVPHYSX_DISABLE_GPU`, `OVPHYSX_PROFILER`, and
`OVPHYSX_PROFILER_OUTPUT` override corresponding global setter values.
Recording output/destination configuration and the legacy arbitrary Carbonite
entry are not part of this scoped key set. Recording capability is the scoped
BOOL key `OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE` (`0x0000000c`). The
compiled ovstage cache mode is the INT64 key
`OVPHYSX_GLOBAL_CONFIG_COMPILED_STAGE_CACHE` (`0x0000000e`); its attach-time
resolution is [REQ-CAPI-COMPILED-CACHE-001](REQ-CAPI-COMPILED-CACHE-001.md).

## Acceptance Criteria

- **AC-1: Scope separation.** Global keys are in
  `0x00000000..0x0000ffff`; instance keys are in
  `0x00010000..0xffff0000`. Unknown keys and cross-scope casts return
  `OVPHYSX_API_INVALID_ARGUMENT`.
- **AC-2: Tagged values.** Setters reject key/type mismatch, embedded-NUL
  strings, unsupported enum/range values, and int64/double values that do not
  fit backend storage without changing the prior effective value.
- **AC-3: Creation lifecycle.** `ovphysx_create_instance(out)` takes no
  configuration. `ACTIVE_CUDA_GPUS` can be set on its handle before attach and
  is rejected after attach without changing the retained value.
- **AC-4: Global lifecycle.** Startup-only and no-attached-stage restrictions
  in the table are enforced. Runtime keys remain mutable where the backend
  honors them. A successful shutdown with no live instances drains the shared
  runtime and reopens startup-only configuration for the next lifecycle.
  `OVD_RECORDING_CAPABLE` rejects writes after first successful create,
  including after the last instance is destroyed, until that successful
  shutdown; failed creates do not freeze the setter and configured values
  remain retained across shutdown.
  For cone/cylinder keys, true (default) selects convex-core geometry at attach;
  false selects a convex-mesh approximation.
- **AC-5: Precedence and observation.** Environment values override global
  setter values until a profiler backend is bound. Getters report the effective
  value, with two exceptions. `COMPILED_STAGE_CACHE` reports the configured
  mode, `AUTOMATIC` included, not the mode an attach resolves from it.
  `COOKED_COLLIDER_CACHE_DIRECTORY` reports its current
  `/UJITSO/datastore/localCachePath` setting rather than querying the active
  datastore location. Before runtime startup, it returns the configured value
  or `OVPHYSX_API_NOT_FOUND` if unset. With local caching and cooking enabled,
  startup replaces an unset or unusable directory with a process-private path
  if one can be created. If creation fails, local caching is disabled and the
  setting is not replaced; disabling local caching or cooking also prevents
  replacement. A later accepted setting change is reported even though the
  datastore keeps its initially selected directory. The getter observes the
  setting, not user intent or persistence. Strings that remain unset return
  `OVPHYSX_API_NOT_FOUND`; the getter itself synthesizes no fallback paths.
  Scalar keys with a declared default return that default when
  unset. After a profiler backend binds, the getter reports the bound backend
  and the trace path captured at bind. A failed bind, and a host-owned
  Carbonite profiler, report `NONE` and an unset output path.
  The recording-capability getter reports its configured bool.
- **AC-6: String ownership.** Setters borrow strings only for the call.
  Getters write into caller storage; insufficient capacity returns
  `OVPHYSX_API_BUFFER_TOO_SMALL` and the required capacity including NUL.
- **AC-7: Compatibility.** Legacy creation entries, typed getters/builders,
  CPU-mode controls, and scalar logging controls remain callable. Controls
  represented by new global keys forward to the scoped implementation: they
  update its retained value when applied, including cone/cylinder polarity,
  and they use its validation and lifecycle rejection. Pre-create entries
  remain queued until creation as specified in AC-11. A Carbonite path matches with or
  without one leading slash. Carbonite tasking/TBB pool sizes stay on the legacy
  path. Retired OVD startup output/destination and capability entries return
  `OVPHYSX_API_INVALID_ARGUMENT` with modern configuration/recording guidance,
  without changing the retained capability. Their numeric slots remain reserved;
  the legacy OVD header/module and capability helpers are removed. Other
  create-time entries may still apply
  startup-only values while the instance is being created; the post-create
  setter may not.
- **AC-9: Log-callback reentrancy.** A log callback may read process-global
  configuration, including the bound profiler backend and output path. A
  startup-only write from inside that callback is rejected before it waits on
  instance creation. Neither path deadlocks.
- **AC-8: Python parity.** Python has module-level global set/get and
  instance-level set/get with the same validation, lifecycle, precedence, and
  copied-string semantics; type-specific instance methods are not primary API.

- **AC-10: Execution mode.** `OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION` is a BOOL
  key, true by default and mutable after startup. Each instance samples it once
  at creation; later changes affect only future instances. False creates no
  worker and completes accepted work on the caller before returning. True is
  subject to the thread/interface fallback in REQ-CAPI-THREADING-001 AC-6; the
  getter continues to report the configured value. Python exposes the same key
  as `GlobalConfigKey.ASYNC_EXECUTION`.
- **AC-11: Pre-create legacy entries.** `ovphysx_set_global_config_entry`
  called before Carbonite starts copies the entry and returns success. The
  next create applies queued entries once, in call order, after retained
  global values and before that create's `config_entries` and plugin load.
  Later creates do not replay them. Getters report the current effective value
  until the queued entry is applied. Entries that map to a scoped key use that
  key's validation at the call. `/physics/cudaDevice` and entries the
  post-start call would reject with `OVPHYSX_API_INVALID_ARGUMENT` for an
  invalid key type or key, a missing key or value string, or an embedded NUL
  in a Carbonite key or value are rejected at the call without changing
  already queued entries. Other Carbonite
  values are not validated by ovphysx.

## Test References

- [TEST-CAPI-CONFIG-001](../../tests/capi/TEST-CAPI-CONFIG-001.md)
- [TEST-CAPI-CONFIG-002](../../tests/capi/TEST-CAPI-CONFIG-002.md)
- [TEST-CAPI-CONFIG-003](../../tests/capi/TEST-CAPI-CONFIG-003.md)

## Code References

- `ovphysx/include/ovphysx/ovphysx_types.h` (scoped keys and tagged value)
- `ovphysx/include/ovphysx/ovphysx.h` (four scoped operations and creation)
- `ovphysx/include/ovphysx/ovphysx_legacy.h` (compatibility declarations)
- `ovphysx/include/ovphysx/ovphysx_legacy_config.h` (compatibility builders)
- `ovphysx/src/ovphysx/ovphysx.cpp` (validation, lifecycle, precedence, storage)
- `ovphysx/python/ovphysx/api.py` (Python scoped operations)
- `ovphysx/python/ovphysx/config.py` (legacy int32 entries reject values outside int32 before storage)
- `ovphysx/python/ovphysx/types.py` (Python scoped keys)
- `ovphysx/tests/c_unittests/test_global_settings.cpp`
- `ovphysx/tests/python_tests/cpu_tests/test_contact_query_cpu.py` (live contact-processing toggle)
- `ovphysx/tests/python_tests/lifecycle_tests/test_config_global.py`
- `ovphysx/tests/python_tests/lifecycle_tests/test_legacy_config_int_narrowing.py`
- `ovphysx/tests/c_unittests/test_omnipvd_recording.cpp` (scoped recording capability lifecycle)
- `ovphysx/tests/c_unittests/test_worker_pool.cpp` (pre-create legacy entry queue)
- `ovphysx/tests/python_tests/lifecycle_tests/test_config_compiled_stage_cache.py` (compiled stage cache key)

## Dependencies

- [REQ-CAPI-CPU-001](REQ-CAPI-CPU-001.md)
- [REQ-CAPI-LOG-001](REQ-CAPI-LOG-001.md)
- [REQ-CAPI-PROFILER-001](REQ-CAPI-PROFILER-001.md)
- [REQ-CAPI-COMPILED-CACHE-001](REQ-CAPI-COMPILED-CACHE-001.md)
