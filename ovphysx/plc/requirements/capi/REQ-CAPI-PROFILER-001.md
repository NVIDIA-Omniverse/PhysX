<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-PROFILER-001
title: Profiler Backend Selection for the Public API
status: implemented
owner: ovphysx
---

## Description

Hosts profiling an ovphysx workload must be able to see which ovphysx API call each
span of wall time belongs to, without rebuilding the library. The instrumentation is
therefore compiled into release builds and shipped wheels, and the consumer it
publishes to is a runtime choice that defaults to none.

The zones are emitted through `carb::profiler`, so one selection covers them, the
runtime's host-side zones, and the PhysX SDK zones alike. Two backends are
selectable: `NVTX`, for an NVIDIA Nsight Systems capture, and `TRACE`, which writes
a chrome trace JSON.

The selection has two forms: the `OVPHYSX_PROFILER` environment variable, which needs
no change to the host application, and
`OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND` passed to
`ovphysx_set_global_config`, which a host can set programmatically before the
first instance. Both resolve to the process-wide `/physics/profilerBackend`
Carbonite setting. Where both are supplied the environment variable wins, so that an
application which hard-codes a backend can still be profiled another way.

This requirement covers the selection and the instrumented surface. Binding the
backend is REQ-CAPI-PROFILER-002; which zones the PhysX SDK emits, and how the runtime
routes them, is REQ-SIM-PROFILER-001.

## Acceptance Criteria

- AC-1: In a process that has not previously selected one, with neither
  `OVPHYSX_PROFILER` set nor global config supplied, no backend is selected and
  `ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND, out)` writes
  `OVPHYSX_PROFILER_BACKEND_NONE`. `/physics/profilerBackend` is process-global and
  sticky, as Carbonite settings are: once selected, a later instance created with no
  setter call and no environment variable inherits the selection. Turning it back
  off is explicit -- `OVPHYSX_PROFILER=none` or a `NONE` global value -- and both
  are honoured.
- AC-2: Setting `OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND` before the first
  instance selects that backend, and the same getter reports it after instance
  creation. A later setter call is rejected because backend binding is startup-only.
- AC-3: When `OVPHYSX_PROFILER` is present before instance creation, its value is
  parsed as a backend name -- `none`, `nvtx` or `trace`, case-insensitively -- and
  written through to `/physics/profilerBackend` in both directions: a naming value
  selects that backend, and `none` deselects even if an earlier instance in the same
  process had selected one. An unrecognized name is reported as a warning and
  contributes nothing: it is neither guessed at nor treated as `none`, so a typo
  leaves whatever the config entry or an earlier instance already selected in place
  rather than silently switching profiling off. The getter reflects the result in
  either case, without a config entry being supplied. A recognized value outranks a
  global setter value, because environment resolution writes over it.
- AC-4: Selecting a backend changes no simulation result and no API status: an
  otherwise identical run with and without one produces the same state.
- AC-5: Every public C entry point declared with `OVPHYSX_API` in
  `include/ovphysx/ovphysx.h` or `include/ovphysx/ovphysx_legacy.h` has exactly
  one scoped zone as the first statement of its definition. Its name is the
  exported symbol, `ovphysx_<entry_point>`
  (for example, `ovphysx_step_sync`). Deprecated aliases are independently
  callable ABI boundaries and therefore have their own zones. Test-only and
  internal exports, and the C++ and Python wrappers around the C API, are not
  additional public C entry points and do not get duplicate boundary zones. Every
  name keeps the established `ovphysx_` prefix because `carb::profiler` has no
  domain concept and the Carbonite NVTX backend publishes into NVTX's default domain.
  A consumer records a complete zone only when its backend is bound for the whole
  call. The first `ovphysx_create_instance` that binds a backend begins before that
  recording boundary; calls before creation and after final destruction are outside
  it. The final `ovphysx_destroy_instance` explicitly closes its zone before
  releasing the last loader and flushing the backend, so that boundary is complete.
  A create/destroy pair made while another instance keeps the backend bound is also
  recorded normally.
- AC-6: Python `set_global_config(GlobalConfigKey.PROFILER_BACKEND, value)`
  accepts a `ProfilerBackend` or integer in range and rejects wrong types or
  out-of-range values. Deprecated `PhysXConfig.profiler_backend` validation
  remains compatible for the legacy creation path.
- AC-7: Whether a zone records is decided per call, not latched the first time a
  call site executes. After a later instance binds a backend, subsequent calls at
  every instrumented entry point record even if the same site already executed while
  none was bound. The binding `ovphysx_create_instance` call itself is the recording
  boundary described in AC-5.
- AC-8: `OVPHYSX_GLOBAL_CONFIG_PROFILER_OUTPUT_PATH` (Python:
  `GlobalConfigKey.PROFILER_OUTPUT_PATH`) names the destination file for the `TRACE`
  backend. It is ignored by the other backends, and leaving it unset is not an
  error: the trace backend falls back to its own timestamped default in the working
  directory.

## Test References

- TEST-CAPI-PROFILER-001

## Code References

- ovphysx/include/ovphysx/ovphysx_types.h (`ovphysx_profiler_backend_t`,
  `OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND`,
  `OVPHYSX_GLOBAL_CONFIG_PROFILER_OUTPUT_PATH`)
- ovphysx/include/ovphysx/ovphysx.h
  (`ovphysx_set_global_config`, `ovphysx_get_global_config`)
- ovphysx/src/include/internal/Profiling.h (the settings path constants and
  `OVPHYSX_API_PROFILE_ZONE`)
- ovphysx/src/ovphysx/Profiling.cpp (`resolveBackend`, `parseBackendName`)
- ovphysx/src/ovphysx/*.cpp (one `OVPHYSX_API_PROFILE_ZONE` at every public C API
  definition)
- ovphysx/python/ovphysx/api.py (`set_global_config`, `get_global_config`)
- ovphysx/python/ovphysx/types.py (`ProfilerBackend`, `GlobalConfigKey`)
- ovphysx/tests/c_unittests/test_profiling.cpp
- ovphysx/tests/python_tests/test_api_profile_zones.py
- ovphysx/tests/python_tests/lifecycle_tests/test_profiler_config.py
- ovphysx/tests/python_tests/cpu_tests/test_physxconfig_validation.py

## Dependencies

- REQ-CAPI-PROFILER-002 - binding the selected backend to the Carbonite framework.
- REQ-SIM-PROFILER-001 - the omni.physx runtime side, which owns the PhysX SDK zones
  and drops the cross-thread ones that carb::profiler cannot express.
