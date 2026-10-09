<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-THREADS-001
title: Typed Solver Thread Configuration Reaches the Dispatcher
status: implemented
owner: ovphysx
---

## Description

The typed solver-thread setting addresses the same process-global setting that
PhysX reads when constructing its CPU dispatcher. This baseline comes from
NVBugs 6847518. It does not promise to resize supporting pools or reconfigure
a dispatcher while scenes are attached.

## Acceptance Criteria

- AC-1: **Dispatcher configuration.** In a fresh process with supporting tasking
  capacity of at least N, passing `ovphysx_config_entry_num_threads(N)` to
  `ovphysx_create_instance()` gives an attached scene a CPU dispatcher reporting
  N workers for N in {1, 2, 4, 8}. Applying subsequent ovstage inputs and stepping
  completes without changing that count.
- AC-2: **Canonical setting.** The typed setter and
  `ovphysx_get_global_config_int32(OVPHYSX_CONFIG_NUM_THREADS, ...)` address
  `/persistent/physics/numThreads`, including values independently written through
  the raw Carbonite config API. Python's typed-path conflict validation recognizes
  that canonical path.

## Test References

- [TEST-CAPI-THREADS-001](../../tests/capi/TEST-CAPI-THREADS-001.md)

## Code References

- `ovphysx/src/ovphysx/ovphysx.cpp`: `s_int32KeyPaths`, typed setter/getter.
- `ovphysx/include/ovphysx/ovphysx_config.h`: `ovphysx_config_entry_num_threads`.
- `ovphysx/python/ovphysx/config.py`: typed field paths and conflict validation.
- `ovphysx/tests/c_unittests/test_solver_thread_count.cpp`.
- `ovphysx/scripts/test_cpp.cmake`: one process per requested count.
- `ovphysx/tests/python_tests/cpu_tests/test_physxconfig_validation.py`.

## Dependencies

None.
