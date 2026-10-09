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
- Independently write 3 to `/persistent/physics/numThreads` through the raw config
  API and read the typed setting.
- Run `test_carbonite_overrides_conflicts_typed_path` in
  `tests/python_tests/cpu_tests/test_physxconfig_validation.py` for the canonical path.

## Then

- `PxScene::getCpuDispatcher()->getWorkerCount()` equals the request both before
  input application and after stepping, and all operations finish (REQ AC-1).
- The typed getter returns the independently authored value 3 (REQ AC-2).
- Python rejects a raw override of the typed setting's canonical path (REQ AC-2).
