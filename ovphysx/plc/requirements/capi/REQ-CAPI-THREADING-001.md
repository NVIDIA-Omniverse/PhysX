<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-THREADING-001
title: Worker Pool Startup Configuration
status: implemented
owner: ovphysx
---

## Description

Carbonite tasking and TBB establish their process-wide worker limits when
ovphysx acquires their plugin interfaces. Under ovphysx's startup-only
configuration policy, an application's choice has to be in place before
ovphysx starts the pools rather than after.

ovphysx supplies no worker-count default of its own. An unconfigured pool keeps
Carbonite's machine-sized default; a bounded default was measured on GPU-heavy
replicated scenes and was slower than the machine-sized pool, so the count is
left to the application.

The Carbonite tasking pool is distinct from `/persistent/physics/numThreads`, which
limits the PhysX CPU dispatcher (default 8) but does not resize Carbonite
workers. `/physics/physxDispatcher` (default false) chooses whether that
dispatcher submits into the Carb pool or into PhysX's own worker set.

## Acceptance Criteria

- AC-1: ovphysx writes no default for `/plugins/carb.tasking.plugin/threadCount`
  or `/plugins/omni.tbb.globalcontrol/maxThreadCount`. With neither configured,
  both keep Carbonite's machine-sized default.
- AC-2: Create-time Carbonite overrides for either worker setting are applied
  before the corresponding pool starts, so the requested count is the count the
  pool starts with. The TBB assertion reads the effective
  `max_allowed_parallelism`, not only its stored setting.
- AC-3: ovphysx acquires both `ITasking` and `IGlobalControl` only after
  create-time configuration. It reports the effective Carbonite worker count
  and configured TBB limit. A tasking pool started by another Carbonite tenant
  is reused without a resize attempt.
- AC-4: A create-time or global worker-pool override submitted after tasking has
  started fails instead of reporting success for an ineffective change.
  Classification accepts Carbonite's equivalent slash-prefixed and slashless
  setting paths.
  The legacy `ovphysx_set_global_config_entry` serializes that check and the
  settings write with `ovphysx_create_instance`, so a concurrent create cannot start the pool
  between a passing check and apply.
- AC-5: Invalid worker counts are rejected before any process-wide setting is
  written. Tasking counts must be non-negative; an explicitly configured TBB
  limit must be positive because absence, not zero, selects its machine-sized
  default. A failed create therefore does not poison a later default create.

## Test References

- TEST-CAPI-THREADING-001

## Code References

- ovphysx/src/CarboniteLoader/CarboniteLoader.cpp
- ovphysx/src/ovphysx/ovphysx.cpp
- ovphysx/src/ovphysx/ovphysxTestHelpers.cpp
- ovphysx/tests/c_unittests/test_worker_pool.cpp
- ovphysx/docs/guides/performance.md

## Dependencies

- None
