<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-CONFIG-001
maps_to: REQ-CAPI-CONFIG-001
type: integration
---

## Scenario

Scoped configuration preserves scope, type, lifecycle, precedence, ownership,
and failure atomicity across the C and Python frontends.

## Given

- A fresh process with no instance and controlled configuration environment.
- A second process with profiler or CPU environment overrides.
- A created instance before and after stage attachment.
- Scalar and string buffers at exact, insufficient, and zero capacity.

## When

- Representative global and instance keys are set and read with valid types,
  wrong tags, and out-of-range values. Fresh-process scalar getters are read
  before the first instance.
- Keys are cast across scopes.
- Startup-only profiler configuration is changed after creation. `NUM_THREADS`
  is changed while a stage is attached.
- A valid value is followed by a rejected write and read again.
- `OVD_RECORDING_CAPABLE` is read unset, set with BOOL, and rejected with
  wrong tags. After plain create, writes are attempted
  with a live instance and after last-instance destruction, then retried after
  successful shutdown with no live instances. Shutdown with a live handle is
  checked not to reopen the setter. Retained capability is read after shutdown.
- Creation fails after settings bootstrap; the capability setter is retried
  before any successful create.
- Environment and setter values conflict, including a profiler environment
  change after a backend has bound.
- A scoped global write is followed by the legacy entry for the same key, then
  another instance is created. The Carbonite path is written both with and
  without one leading slash.
- Retired typed and raw OVD capability/startup entries are tried at create and
  post-create, including both Carbonite path spellings; the retained scoped
  capability is read after rejection.
- A legacy overlapping write uses an out-of-range value, runs while a stage is
  attached, and sets the profiler backend after creation.
- Python legacy creation is given `num_threads=2**32 + 4`,
  `ovstage_read_pool_max_mb=2**31`, and `scene_multi_gpu_mode=2**31`, then a
  representable `num_threads=3` with pool budget `512`.
- Python legacy creation is given an omitted `active_cuda_gpus` argument and,
  in a separate fresh process, an explicitly empty argument.
- Cone custom geometry is set and the approximate-mesh setting is read back.
- Cone and cylinder custom geometry are set to opposite values before attach,
  then swapped after detach and attached again.
- A log callback reads the bound profiler during a later create, and a log
  callback sets CPU-only during the first create. Each callback returns.
- A runtime-mutable global key the backend honors (`DISABLE_CONTACT_PROCESSING`)
  is set and then cleared, with a contact read taken after each.
- `COMPILED_STAGE_CACHE` is read unset, set to each mode, given out-of-range
  values and wrong tags, and written while a stage is attached.
- Equivalent operations are performed through Python.
- In a fresh process, before the first instance, legacy Carbonite entries set
  the tasking thread count from strings that go out of scope and turn
  `/physics/suppressReadback` on. `/physics/cudaDevice` and an out-of-range
  `NUM_THREADS` entry are also set. An instance is created, the readback
  setting is turned off, and a second instance is created.
- Malformed key types, out-of-range typed keys, and embedded-NUL Carbonite
  keys and values are submitted between valid pre-create entries.
- A legacy scoped-key entry is queued and read before and after creation.
  Immediate legacy round-trip tests explicitly create an instance first.

## Then

- Valid values round-trip through the correct scoped getter (REQ AC-1, AC-2,
  AC-5, AC-8).
- The actual cone and cylinder shapes use `eCONVEXCORE` when their custom
  geometry key is true and `eCONVEXMESH` when false (REQ AC-4), in
  `tests/c_unittests/test_global_settings.cpp`,
  `CollisionGeometryConfigTest.SelectsConeAndCylinderShapeGeometry`.
- Cross-scope keys, invalid tags/ranges, and invalid mutation phases are
  rejected with the prior effective value unchanged (REQ AC-1, AC-2, AC-3,
  AC-4).
- `OVD_RECORDING_CAPABLE` has numeric value `0x0000000c`, defaults false,
  accepts only BOOL, and rejects mutation after first successful create until
  successful shutdown with no live instances. Shutdown with a live handle does
  not reopen the setter; failed create after settings bootstrap does not freeze
  it. Rejection leaves the value unchanged, shutdown retains the configured
  value, and the getter reports that configured bool (REQ
  AC-1, AC-2, AC-4, AC-5, AC-8).
- A runtime-mutable key the backend honors takes effect on the dependent read
  and is restored when cleared: `DISABLE_CONTACT_PROCESSING` makes the contact
  read report no contacts, then report them again once cleared (REQ AC-4), in
  `tests/python_tests/cpu_tests/test_contact_query_cpu.py::test_cpu_disable_contact_processing_zeroes_the_contact_read`.
- `COMPILED_STAGE_CACHE` has numeric value `0x0000000e`, reports `AUTOMATIC`
  when unset, round-trips modes 0..3 without resolving `AUTOMATIC`, rejects
  `-1`, `4` and non-INT64 tags with the prior value unchanged, and accepts
  writes while a stage is attached (REQ AC-1, AC-2, AC-4, AC-5), in
  `tests/c_unittests/test_global_settings.cpp`
  (`ScopedConfig.CompiledStageCache*`) and
  `tests/python_tests/lifecycle_tests/test_config_compiled_stage_cache.py`.
  Attach-time resolution is TEST-CAPI-COMPILED-CACHE-001's.
- Environment values are observed instead of conflicting setter values (REQ
  AC-5).
- String setters retain copies and getters report exact required capacity
  without returning borrowed native storage (REQ AC-6, AC-8).
- Legacy wrappers remain callable and observe the scoped implementation where
  the surfaces overlap, including validation, lifecycle rejection, and either
  Carbonite path spelling (REQ AC-7).
- Python legacy creation rejects those integers that do not fit int32 before
  an instance exists, leaving threads at 8, the pool budget at 256, and
  scene multi-GPU mode at 0. The representable pair applies as 3 and 512
  (REQ AC-2, AC-7), in
  `tests/python_tests/lifecycle_tests/test_legacy_config_int_narrowing.py`.
- Python's legacy creation warning directs scoped settings to
  `set_global_config()` and keeps Carbonite-only settings on
  `PhysXConfig.carbonite_overrides` (REQ AC-7, AC-8), covered by
  `tests/python_tests/lifecycle_tests/test_typed_config_init.py`.
- Python legacy creation reports an omitted active CUDA GPU list as unset
  (`None`) while preserving an explicitly empty list as `""` (REQ AC-5, AC-7,
  AC-8), covered by
  `tests/python_tests/lifecycle_tests/test_typed_config_init.py` and
  `tests/python_tests/lifecycle_tests/test_legacy_active_cuda_gpus_empty.py`.
- Retired OVD typed/raw keys return `INVALID_ARGUMENT` with migration guidance,
  including slash-prefixed and slashless paths, without changing retained
  capability. Their numeric slots remain reserved and public legacy OVD
  builders/header/module are absent (REQ AC-7).
- The profiler read from a log callback returns, and the startup-only write
  from a log callback is rejected without waiting on instance creation (REQ
  AC-9).
- The pre-create entries return success, and the pool starts with the queued
  thread count. The first instance sees readback suppressed, and the second
  sees it off. The `/physics/cudaDevice` and out-of-range entries return
  `INVALID_ARGUMENT` at the call (REQ AC-11), in
  `tests/c_unittests/test_worker_pool.cpp`
  (`WorkerPoolQueuedEntryTest.PreCreateEntriesApplyOnceBeforePluginsLoad`).
- Malformed entries return `INVALID_ARGUMENT` at the setter. The first create
  still succeeds and applies the valid entries queued before and after them
  (REQ AC-11), in the same fresh-process worker-pool test.
- A getter reports the prior effective value before creation and the queued
  value after creation (REQ AC-5, AC-7, AC-11), in that same test.
