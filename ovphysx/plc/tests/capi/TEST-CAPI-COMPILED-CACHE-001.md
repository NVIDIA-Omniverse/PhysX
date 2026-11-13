<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-COMPILED-CACHE-001
maps_to: REQ-CAPI-COMPILED-CACHE-001
type: integration
---

## Scenario

The compiled stage cache key validates and retains its mode, stays writable for
the whole process lifetime, and each attach resolves and reports the mode it
used. With the default mode and a configured directory, a result stored by one
process is restored by the next. Other hit, miss and bypass rules are
REQ-CACHE-ATTACH-001's and are not asserted here.

## Given

- A C unit-test process and a fresh Python process with no instance.
- `tests/data/minimal_scene.usda` populated into an ovstage Stage.
- For resolution, one fresh child process per case, in hard CPU-only mode,
  with `LOG_LEVEL` INFO, the Python logging bridge enabled, UJITSO collision
  cooking on or off, and either no cooked-collider cache directory or a new
  temporary one.

## When

- The key's numeric value and its unset value are read.
- Each mode is set and read back. Then `-1`, `4`, a value above `INT32_MAX`,
  and BOOL, DOUBLE and STRING tags are written after `READ_ONLY`.
- A mode is written with a live instance and while a stage is attached; the
  value is read again after the last instance is destroyed.
- The scene is attached once under each mode.
- Each child sets its mode, directory and cooking setting, attaches the scene
  and prints the `compiled stage cache:` log lines. Cases: AUTOMATIC without a
  directory, AUTOMATIC with one, AUTOMATIC with one and cooking off, DISABLED
  with one, READ_ONLY and READ_WRITE without one.
- Two children attach the scene one after the other with AUTOMATIC and the same
  new directory, nested in another new directory.
- On Linux, children attach the scene with AUTOMATIC and then READ_WRITE in an
  existing directory that is group-writable, world-writable, readable by others,
  reached through a symlink, or owner-only inside a group-writable parent; and
  with AUTOMATIC in an owner-only directory inside a sticky, world-writable
  parent.
- One child creates two instances, the first configuring a group-writable
  directory and the second an owner-only one, and attaches the scene with each
  while both live, with AUTOMATIC.
- Two children attach the scene with READ_WRITE and cooking off, one without a
  directory and one with a new one, and print the datastore directory.
- With a CUDA device, four children share one new directory and attach a stage
  whose rigid body has an SDF mesh collider with AUTOMATIC: two CPU-only, then
  two that may use the GPU.
- With a CUDA device, a Direct GPU stage of sixteen boxes over a static triangle
  mesh grid of 256 by 256 vertices is attached and simulated for a second by one
  child with DISABLED and no directory, then by three children with READ_WRITE
  sharing one new directory.
- One child runs two lifecycles with READ_WRITE and a new directory, attaching
  one scene in each; on Linux, Carbonite shuts down after the first. Then a
  READ_ONLY child attaches each scene.

## Then

- The key is `0x0000000e`, the unset value is `AUTOMATIC`, and every mode
  round-trips (REQ AC-1, AC-2).
- Every invalid value or tag returns `OVPHYSX_API_INVALID_ARGUMENT` (Python
  `ValueError`) and the getter still reports `READ_ONLY` (REQ AC-1).
- Writes with a live instance and with an attached stage succeed, and the value
  outlives the last instance (REQ AC-2, AC-3).
- The attach succeeds under every mode (REQ AC-3).
- Each child logs exactly one cache line, with the log level at INFO.
  AUTOMATIC resolves to `read-write` with the configured directory and to
  `disabled` without one or with cooking off; explicit modes resolve to
  themselves either way. `, automatic` appears only for AUTOMATIC, and the
  result is `disabled` exactly when the resolved mode is (REQ AC-4, AC-5).
- The first of the two children reports `stored` and the second `hit`
  (REQ AC-6). On Linux, both directories the first created have mode 0700
  (REQ AC-4).
- AUTOMATIC reports `disabled (mode disabled, automatic)` for each directory
  that is not private, and READ_WRITE reports `stored (mode read-write)` there;
  in the owner-only directory inside the sticky parent AUTOMATIC reports
  `stored (mode read-write, automatic)` (REQ AC-4, AC-5).
- In the two-instance child both attaches report
  `disabled (mode disabled, automatic)` (on Windows, where no permission check
  applies, the first reports `stored`), and the second logs that the datastore
  was not built from the second directory (REQ AC-4).
- With READ_WRITE and cooking off, the child without a directory reports
  `unavailable (mode read-write)` and no process-private directory
  (`ovphysx-cache-*`); the child with one reports `stored (mode read-write)` in
  that directory (REQ AC-4, AC-5).
- The CPU-only children report `stored`, then `hit`; the first GPU child
  reports `stored`, not `hit`, because SDF meshes cook on a different device,
  and the second reports `hit` (REQ AC-6; REQ-CACHE-ATTACH-001 AC-2 in the
  ovruntime tree).
- The triangle-mesh children all exit cleanly and log no error. The READ_WRITE
  children report `stored`, `hit` and `hit`, and every child reads the same poses,
  bit for bit, as the DISABLED one (REQ AC-6; REQ-CACHE-ATTACH-001 AC-7 and AC-15
  in the ovruntime tree).
- The two-lifecycle child exits cleanly after Carbonite's shutdown completed,
  and the READ_ONLY child hits both scenes, so each lifecycle stored its own
  (REQ AC-6; REQ-CACHE-STORE-001 AC-8 in the ovruntime tree).

## Coverage

- `ovphysx/tests/c_unittests/test_global_settings.cpp` -
  `ScopedConfig.CompiledStageCacheDefaultsToAutomaticAndRoundTrips`,
  `ScopedConfig.CompiledStageCacheRejectsInvalidValuesWithoutChange`,
  `ScopedConfig.CompiledStageCacheModesAttachAndStayWritableWhileAttached`.
- `ovphysx/tests/c_unittests/test_c_api_compatibility.c` - C ABI values.
- `ovphysx/tests/python_tests/lifecycle_tests/test_config_compiled_stage_cache.py` -
  Python round trip, rejection, live writes, the six resolution cases,
  `test_automatic_stores_then_hits_through_the_configured_directory`,
  `test_automatic_uses_only_a_private_directory` (Linux only),
  `test_automatic_judges_the_directory_the_datastore_uses`,
  `test_cooking_off_sets_up_no_cache_directory`,
  `test_sdf_meshes_cooked_on_another_device_are_not_restored` and
  `test_triangle_meshes_restored_by_another_process_simulate_on_the_gpu` (each
  skipped without a CUDA device) and `test_stores_again_after_the_runtime_restarts`.
- `ovphysx/tests/python_tests/lifecycle_tests/test_config_scalar_defaults.py` -
  fresh-process default.
- `ovphysx/tests/python_tests/test_types_sync.py` -
  `test_compiled_stage_cache_mode_values_match_c_header`.

## Coverage by clause

Each clause of REQ AC-3, AC-4 and AC-5, with the case that asserts it. *Not
automated* marks a clause that no case asserts.

AC-3, runtime mutable, read per attach:

- Writes succeed before the first instance and with a live instance:
  `test_compiled_stage_cache_round_trip_rejection_and_live_writes`; while a
  stage is attached:
  `ScopedConfig.CompiledStageCacheModesAttachAndStayWritableWhileAttached`.
- Each attach reads the value current when it is called: *Not automated*; each
  resolution child sets the key before it creates its instance, and the C case
  checks no log line.
- The same holds when the attach task runs later: *Not automated*; every case
  waits for the attach before it changes the key.
- An attachment keeps the mode it attached with, also for its later
  insertions: *Not automated*; no case changes the key after an attach and then
  inserts a unit.
- With nothing stored, every mode attaches a loadable stage:
  `ScopedConfig.CompiledStageCacheModesAttachAndStayWritableWhileAttached`.

AC-4, mode resolution:

- DISABLED, READ_ONLY and READ_WRITE pass their own mode:
  `test_attach_resolves_and_logs_the_configured_mode`, and
  `test_cooking_off_sets_up_no_cache_directory` for READ_WRITE with cooking off,
  with and without a configured directory.
- AUTOMATIC passes read-write with a configured directory and cooking on, and
  disabled without a directory (the process-private fallback) or with cooking
  off: `test_attach_resolves_and_logs_the_configured_mode`. The branches where
  `/UJITSO/datastore/allowLocalDataStore` is false or
  `/UJITSO/datastore/localCachePath` is empty: *Not automated*.
- AUTOMATIC uses only the directory the running datastore was built from, not
  one a later instance configures:
  `test_automatic_judges_the_directory_the_datastore_uses`. A datastore that a
  host started before ovphysx (nothing recorded), a directory replaced at the
  same path (another inode) and shutdown forgetting the record: *Not automated*.
- On POSIX the directory must be private. Group- and world-writable, readable by
  others, a symlink, and a group-writable parent each disable it, and a sticky
  world-writable parent does not: `test_automatic_uses_only_a_private_directory`.
  A parent owned by another user: *Not automated*; a test cannot create one
  without privileges.
- Startup creates each missing directory of a configured path owner-only:
  `test_automatic_stores_then_hits_through_the_configured_directory`.
- With cooking off, startup sets up no directory, and without a configured one
  turns local caching off so that an explicit mode reports `unavailable`:
  `test_cooking_off_sets_up_no_cache_directory`.
- The check takes no loader lock: *Not automated*; no case observes lock use.

AC-5, outcome report:

- Every attach passes a result slot: indirectly; the logged `stored` and `hit`
  results can only come from it.
- One `compiled stage cache: <result> (mode <resolved>[, automatic])` line per
  attach, with `, automatic` only for AUTOMATIC: every child above finds exactly
  one. Its INFO level is not checked; the children only set the log level to
  INFO. A busy refusal, which logs no line, is *Not automated*.
- `<result>` names the runtime's result: `disabled`, `stored` and `hit` are
  checked; `bypassed`, `miss`, `restore failed`, `store failed` and
  `unavailable` are *Not automated* here.
- A disabled mode reports `disabled`:
  `test_attach_resolves_and_logs_the_configured_mode`.
- Only the cache options differ from their defaults in the attach options
  ovphysx passes: *Not automated*; no case inspects those options.

## Debug build

The hit cases (`test_automatic_stores_then_hits_through_the_configured_directory`,
`test_sdf_meshes_cooked_on_another_device_are_not_restored`,
`test_stores_again_after_the_runtime_restarts`) also run in the Debug lifecycle
job, where the runtime allocator asserts on a `new`/`delete` pairing mismatch at
destroy. They are the regression test for restored runtime metadata being
allocated through `ICE_NEW` (TEST-CACHE-ATTACH-001, *Restored runtime metadata
lifetime*).

## Not covered

- Other cache outcomes and datastore refusal belong to REQ-CACHE-ATTACH-001 and
  REQ-CACHE-UPDATE-001 and are exercised by the ovruntime suites.
- The AC-3, AC-4 and AC-5 clauses marked *Not automated* under *Coverage by
  clause*.
