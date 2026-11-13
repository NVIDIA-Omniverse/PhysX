<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-THREADS-001
maps_to: REQ-CAPI-THREADS-001
type: integration
---

## Scenario

`SolverThreadCount.Workers1`, `Workers2`, `Workers4`, and `Workers8` in
`ovphysx/tests/c_unittests/test_solver_thread_count.cpp` verify the actual
CPU dispatcher instead of only round-tripping the configuration store.
`scripts/test_cpp.cmake` runs each case in a separate process with a 60-second timeout.

## Given

- A fresh CPU-only process with no earlier PhysX instance or attached scene.
- A shared tasking pool with capacity for the requested worker count. Cases
  exceeding Carbonite's container-aware default capacity skip on smaller hosts.
- The small boxes/ground-plane USD scene, populated and sealed through ovstage.

## When

- Create an instance with a typed request for 1, 2, 4, or 8 workers and attach.
- Populate and seal a later ordinal adding a moving rigid body, apply its inputs,
  verify the actor exists, and run ten steps.
- Detach the scene, then independently write 3 to `/persistent/physics/numThreads` through the raw config
  API and read the typed setting.
- Run `test_carbonite_overrides_conflicts_typed_path` in
  `tests/python_tests/cpu_tests/test_physxconfig_validation.py` with and without
  one leading slash. Also try typed `num_threads=2` with raw value 5 in
  `test_num_threads_override_conflict`.
- Run `test_carbonite_overrides_rejects_non_string_key_at_conversion` with
  `int`, `bytes`, and `PurePosixPath` keys supplied at construction, added by
  later dict mutation, or assigned by replacing `carbonite_overrides`.
- Run `test_carbonite_overrides_preserves_unknown_string_key` with and without
  one leading slash.

## Then

- `PxScene::getCpuDispatcher()->getWorkerCount()` equals the request both before
  input application and after stepping, and all operations finish (REQ AC-1).
- The typed getter returns the independently authored value 3 (REQ AC-2).
- Python rejects either spelling of the typed setting's path with the same
  typed-field conflict, including when the typed value is also supplied
  (REQ AC-2).
- `_to_c_config` rejects non-string override keys before path normalization
  with a `TypeError` naming `PhysXConfig.carbonite_overrides` and the offending
  type, including after construction. Unknown string keys retain their spelling
  and their values are converted normally.

The Python cases exercise configuration conversion. Runs using mock C entry
containers do not validate native settings or the CPU dispatcher.
