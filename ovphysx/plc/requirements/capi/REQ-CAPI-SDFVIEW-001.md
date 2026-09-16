<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-SDFVIEW-001
title: SDF View Stage-Lifetime Safety
status: implemented
owner: ovphysx
---

## Description

An SDF view (`ovphysx_sdf_view_handle_t`, `SdfView` in Python) is created against
the USD stage attached at creation time and borrows per-stage backend state: the
native `omni::physics::tensors::ISdfShapeView` points into the `SimulationBackend`
data owned by that stage. Detaching or resetting the stage destroys that data, so
a view that outlives its stage holds a dangling pointer.

This requirement fixes the contract for that lifetime. Stage teardown releases the
views it invalidates, and any subsequent use of a surviving handle is rejected with
a catchable error. Terminating the process is never an acceptable outcome for a
handle the API already tracks, including on the misuse path the API documents
against -- `reset_stage()` is the documented way to reload a scene, so a live view
across a reload is on the mainline workflow rather than an exotic edge case.

The rejection is ordered ahead of the work it protects: `ovphysx_evaluate_sdf()`
auto-warms the simulation (a `simulate()` + `fetchResults()` pass over per-stage
state) when warmup has not run for the attached stage, so validating the handle
after the warmup would let a stale view reach GPU work before any check ran.

## Acceptance Criteria

- AC-1: Stage teardown releases every SDF view owned by the instance before the
  PhysX / tensor backend data those views point into is destroyed. This holds for
  all three teardown routes: `ovphysx_detach_ovstage()`, `omni_sdk_physx_unload_usd()`,
  and `omni_sdk_physx_destroy()`. `ovphysx_reset_stage()` inherits it by delegating
  to `ovphysx_detach_ovstage()` for an attached ovstage.

- AC-2: Using an SDF view handle whose stage is no longer the attached stage returns
  `OVPHYSX_API_NOT_FOUND` with the message `"SDF view invalidated (stage changed);
  recreate view"`. A handle released by AC-1 teardown returns `OVPHYSX_API_NOT_FOUND`
  with `"SDF view not found"`. Both surface in Python as `RuntimeError`. This applies
  to `ovphysx_evaluate_sdf()`, `ovphysx_sdf_view_get_count()`, and
  `ovphysx_sdf_view_get_max_query_points()`.

- AC-3: No use of a stale or unknown SDF view handle terminates the process. The
  calling process stays alive and the caller can continue, on both the
  `reset_stage()` and the `detach_ovstage()` teardown route.

- AC-4: `ovphysx_evaluate_sdf()` resolves the handle and applies the AC-2 validity
  check **before** `ovphysx_warmup_if_needed()`, so a stale handle is rejected
  before any simulation step or GPU access. The check is re-applied under the
  instance lock after the warmup, because the warmup runs with locks released and
  another thread may destroy the view in that window.

- AC-5: `ovphysx_destroy_sdf_view()` is idempotent for a handle whose native view
  was already released by AC-1 teardown: it returns success rather than an error,
  so a caller holding a stale handle can always safely destroy it.

## Test References

- TEST-CAPI-SDFVIEW-001

## Code References

- ovphysx/src/ovphysx/ovphysxSdfView.cpp (`checkSdfViewStageValid`, `checkSdfViewUsable`, `ovphysx_sdf_view_cleanup_instance`, evaluate / count / max-query-points / destroy entry points)
- ovphysx/src/ovphysx/ovphysx.cpp (`ovphysx_detach_ovstage`, `omni_sdk_physx_unload_usd`, `omni_sdk_physx_destroy` teardown cleanup call sites)
- ovphysx/tests/python_tests/test_sdf_view_lifecycle_gpu.py

## Dependencies

- None
