<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-CACHE-001
title: Process-Private Cooked-Collider Cache Teardown
status: implemented
owner: ovphysx
---

## Description

When ADR-0001's process-private cooked-collider cache fallback is active
(no `cooked_collider_cache_dir` configured), ovphysx removes that
process-wide temp directory when the last live `CarboniteLoader` releases
the framework. Python process-exit cleanup first destroys any still-live
instances so it reaches that same last-loader path before interpreter
finalization. Cleanup does not pull the directory out from under a
concurrently starting loader or leave a failure undiagnosed.

## Acceptance Criteria

- AC-1: Removal is refcounted against live `CarboniteLoader` instances in
  the process (`g_activeLoaders`); it runs only when the last one shuts
  down, never on an arbitrary per-instance shutdown that could pull the
  directory out from under another instance still cooking against it.
- AC-2: The last-loader decision and the removal it triggers are
  serialized against `CarboniteLoader::initialize()`'s own
  `g_activeLoaders` increment (both hold `g_bootstrapMutex`), so a
  concurrent `ovphysx_create_instance()` cannot become the new first
  loader in the process while a prior shutdown's removal is still in
  progress.
- AC-3: Removal retries up to 5 times, 20 ms apart, before giving up, to
  close the race where `carb.ujitso.default`/`carb.datastore` are still
  writing a just-completed cook's bytes to disk when teardown starts
  (`ovphysx_wait_op()`/`PhysX::wait_all()` only drain the cook compute
  queue, not the datastore's write-back). A removal error classified as
  non-transient (permission denied) stops retrying immediately instead of
  sleeping through the remaining budget. This is best-effort and narrows
  the race window rather than closing it: a write-back slower than the
  full retry budget can still leave the directory (or part of it) behind.
  The remembered path is not cleared when removal reports success, so a
  later call (the `atexit` backstop) safely re-attempts it; removing an
  already-gone path is a no-op.
- AC-4: When every retry attempt fails, ovphysx logs one `WARN` naming the
  path and the last error before giving up, so a leaked temp directory is
  diagnosable from shutdown logs rather than silently abandoned.
- AC-5: After the first Python `PhysX` instance completes native bootstrap,
  the wrapper registers an `atexit` callback and weakly tracks every live
  instance. On normal interpreter exit, including an uncaught
  `KeyboardInterrupt`, that callback destroys all still-live instances
  before interpreter finalization; the last destruction reaches the same
  `ovphysx_shutdown()` and last-loader retry path as explicit
  `PhysX.destroy()`. The native `atexit` hook remains a final single-attempt
  cache-removal backstop for non-Python callers or an earlier cleanup
  failure. Abrupt termination that does not run process-exit handlers
  (including `SIGKILL` and hard crashes) is outside this guarantee, as is a
  process that forks after constructing `PhysX`.

## Test References

- TEST-CAPI-CACHE-001
- TEST-CAPI-CACHE-002

## Code References

- ovphysx/src/CarboniteLoader/CarboniteLoader.cpp
- ovphysx/python/ovphysx/api.py
- ovphysx/tests/c_unittests/test_multi_instance.cpp
- ovphysx/tests/python_tests/test_ujitso_cooking_cache.py

## Dependencies

- ADR-0001
