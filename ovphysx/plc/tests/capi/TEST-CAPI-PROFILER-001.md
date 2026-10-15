<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-PROFILER-001
maps_to: REQ-CAPI-PROFILER-001
type: unit
---

## Scenario

No profiler backend is selected by default; one can be selected through either
process-global configuration or the `OVPHYSX_PROFILER` environment variable; the selection is
observable through the public config getter; and selecting one changes neither
simulation results nor API statuses. The Python `profiler_backend` field normalizes
names and enum values, validates its input, and refuses to be shadowed by a raw
Carbonite override on the deprecated compatibility path.

Whether a consumer records the zones cannot be asserted from a test process for the
NVTX backend, so those tests cover the selection and its side-effect freedom; the
timeline itself is verified manually. The trace backend does leave a checkable
artifact, and that is asserted by TEST-CAPI-PROFILER-002.

The instrumented surface is also checked statically against the public header: every
`OVPHYSX_API` declaration must have exactly one definition whose first statement is
the matching API-boundary zone, and no zone may exist without a public declaration.
This makes a missing, swapped, late, duplicated, added, or renamed annotation a test
failure. The final destroy path is additionally checked for its required order: internal
teardown, explicit zone close, then deferred profiler release.

## Given

- A C unit-test process that creates an instance with no profiler config entry and
  `OVPHYSX_PROFILER` unset.
- A C unit-test process that creates an instance with
  the profiler backend set to `OVPHYSX_PROFILER_BACKEND_NVTX`.
- The same process with `OVPHYSX_PROFILER` set before instance creation, to each of
  `nvtx`, `NVTX`, `none`, and an unrecognized name.
- A stage with a rigid body that can be stepped a fixed number of times.
- The `PhysXConfig` dataclass, without a native library (validation is pure
  Python).
- The set of public C declarations marked `OVPHYSX_API` in
  `include/ovphysx/ovphysx.h` and `include/ovphysx/ovphysx_legacy.h`, including
  deprecated declarations, and the zone annotations in `src/ovphysx/*.cpp`.

## When

- `ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_PROFILER_BACKEND, &out)` is called
  after each of the creation paths above.
- The same stage is stepped a fixed number of times with a backend selected and
  with none, reading body poses back through the session read.
- A representative subset of the instrumented entry points is exercised with a
  backend selected.
- The public declarations and C definitions are parsed after stripping comments and
  string literals; balanced parameter lists identify definitions and the first
  statement in each body is associated with that definition's zone. The final destroy
  body is checked for zone-close-before-profiler-release ordering.
- The whole suite runs in a single process under Nsight Systems, so that selected
  cases follow a case that ran with no backend.
- `PhysXConfig(profiler_backend=...)` is constructed with a name, a
  `ProfilerBackend` value, an unknown name, an out-of-range integer, and a `bool`.
- `PhysXConfig(carbonite_overrides={"/physics/profilerBackend": 1})` is converted to
  C config entries.

## Then

- The getter writes `NONE` for the default path (REQ AC-1) and the selected backend
  for both the global-config path (REQ AC-2) and the environment-variable path
  (REQ AC-3), with names matched case-insensitively.
- An unrecognized `OVPHYSX_PROFILER` value leaves the getter reporting `NONE`
  (REQ AC-3).
- After an instance selected NVTX via global configuration, a second instance created
  with no new setter and `OVPHYSX_PROFILER=none` reports `NONE`, so the sticky
  selection can be turned back off through the environment variable alone
  (REQ AC-1 stickiness, REQ AC-3 both directions).
- The selected and unselected runs report the same statuses and the same final
  poses within tolerance (REQ AC-4).
- The exercised entry points return success with a backend selected, so the added
  zones do not disturb those call paths. The static inventory has no missing, extra,
  duplicate, late, or mismatched names, proving every public C API definition starts
  with exactly its own boundary zone (REQ AC-5). It also proves final destroy closes
  that zone before the deferred loader releases the profiler. The trace artifact in
  TEST-CAPI-PROFILER-002 proves representative annotations emit complete runtime
  events; manual Nsight capture covers the NVTX route.
- Private binding helpers `ovphysx_internal_get_attach_handle` and
  `ovphysx_internal_is_query_valid` have no public declarations and carry no
  `OVPHYSX_API_PROFILE_ZONE`; the inventory rejects an annotation on either
  helper as an extra public boundary zone (REQ AC-5).
- `PhysXConfig(profiler_backend="nvtx")` and `PhysXConfig(profiler_backend=
  ProfilerBackend.NVTX)` both normalize to the integer `1`; an unknown name and an
  out-of-range integer raise `ValueError`; a `bool` raises `TypeError`; and the
  `carbonite_overrides` form raises `ValueError` (REQ AC-6).
- Running the whole suite in one process, where the first case runs with no backend,
  still yields zones for the later selected cases (REQ AC-7). Emission is not
  observable in-process for NVTX, so this is checked by capture, and the instance
  counts are the assertion: a gate latched at first execution yields zero for every
  site instead.
- Leaving `OVPHYSX_GLOBAL_CONFIG_PROFILER_OUTPUT_PATH` unset with the trace backend
  selected still produces a trace, at the plugin's default location (REQ AC-8);
  the configured-path case is asserted by TEST-CAPI-PROFILER-002.

## Test location

`ovphysx/tests/c_unittests/test_profiling.cpp` (the `ProfilerBackend` fixture, plus
`ProfilerSideEffects.EnabledRunMatchesDisabledRun`, which `scripts/test_cpp.cmake`
runs in its own `profiler-side-effects` pass so its unprofiled arm has no backend
bound),
`ovphysx/tests/python_tests/lifecycle_tests/test_profiler_config.py`,
`ovphysx/tests/python_tests/cpu_tests/test_physxconfig_validation.py`, and
`ovphysx/tests/python_tests/test_api_profile_zones.py`.
