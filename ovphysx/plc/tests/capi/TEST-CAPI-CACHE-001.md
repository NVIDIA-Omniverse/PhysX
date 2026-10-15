<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-CACHE-001
maps_to: REQ-CAPI-CACHE-001
type: integration
---

## Scenario

`tests/python_tests/test_ujitso_cooking_cache.py` observes the private directory
between native instance destruction and SDK shutdown, then after process exit.

## Given

- A CPU-only Python subprocess with an isolated temporary directory and no configured cache.
- A loaded mesh scene with completed collision cooking.
- A witness around `ovphysx_destroy_instance` that runs before Python calls SDK shutdown.

## When

- The last instance is destroyed and the Python lifecycle shuts down the SDK.
- The child exits normally.

## Then

- The directory still exists after native instance destruction (REQ AC-1).
- The child exits without datastore errors and on Linux leaves no private directory
  (REQ AC-3). Windows retains its established best-effort removal contract.

## Additional Coverage

- `test_cache_dir_reconfiguration_follows_framework_lifetime` cooks in two successive
  SDK lifecycles. Linux uses each configured directory; Windows keeps the first
  datastore and directory until process exit (REQ AC-3).
- `MultiInstance.ConcurrentCreateDestroyMaintainsLoaderLifecycle` is a loader
  concurrency smoke test only. It does not call SDK shutdown and supplies no
  evidence for the framework-release/directory-removal critical section (REQ AC-2).
  That locking boundary is verified by inspection of `bootstrapMutex` coverage;
  no deterministic concurrent SDK-shutdown test is claimed here.
