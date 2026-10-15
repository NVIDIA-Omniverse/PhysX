<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-SDF-001
title: SDF Shape Evaluation on the Public API
status: implemented
owner: ovphysx
---

## Description

ovphysx exposes SDF evaluation on the session query, the successor of the deprecated SDF view
([ADR-0048](../../../ovruntime/plc/adr/ADR-0048-sdf-shape-evaluation-through-the-session-query.md)):
`OVPHYSX_QUERY_TARGET_SDF_SHAPE` and `ovphysx_evaluate_sdf_shapes` in C, `SimObjectType.SDF_SHAPE` and
`PhysX.evaluate_sdf_shapes` in Python. The runtime contract is REQ-READ-SDF-001; this requirement fixes the
public surface around it.

## Acceptance Criteria

- AC-1: **Type and verb.** `OVPHYSX_QUERY_TARGET_SDF_SHAPE` is 24 and opens through `ovphysx_query` and
  `ovphysx_query_selection`. `ovphysx_evaluate_sdf_shapes(handle, query, points, out, points_ready)` returns an
  `ovphysx_enqueue_result_t`. It is declared in `ovphysx/ovphysx_geometry_query.h`, which compiles on its own
  and alongside `ovphysx.h`; `ovphysx.h` does not include it. Before a task is created it refuses with
  `OVPHYSX_API_INVALID_ARGUMENT` a null tensor or data pointer, a tensor that is not float32 [N, Q, 3] /
  [N, Q, 4] and C-contiguous on a CUDA device, tensors that disagree on N, Q or device, Q of 0, N above 65535
  (one PhysX launch's limit), and N x Q above 2^31 - 1 (PhysX's 32-bit point indexing); and with
  `OVPHYSX_API_NOT_FOUND` a query the instance does not own. Either way `op_index` is 0.
- AC-2: **Execution.** The task attaches physics if needed, then refuses a call the runtime check
  (`ovxCheckSdfShapes`) rejects -- a stale, wrong-type or empty query, mismatched rows, rows outside one
  DirectGPU scene -- before running the automatic DirectGPU warmup, so a refused call never steps the
  simulation, as the SDF view refused a stale view before its warmup. It then warms up before the first step as
  the SDF view did, the only step it ever takes, and evaluates through the runtime. `points_ready` is ovstage's
  `{stream, wait_event}` producer handoff for `points`, as `ovphysx_commit_group` takes it: a non-zero stream is
  drained and a non-zero event waited on before the points are read, and only `{0, 0}` asserts nothing is
  outstanding. A runtime refusal (another type, no SDF shape, N other than the row count, more than 65535 rows,
  rows outside one DirectGPU scene, a scene that has not simulated since its physics objects last changed
  (REQ-READ-SDF-001 AC-7), tensors on a device other than the scene's, a query kept across `detach_ovstage`, a
  handoff that cannot be honored) fails the task with the runtime's message, which
  `ovphysx_get_last_task_error` returns; `out` is complete when the task completes.
- AC-3: **Legacy pointer.** The SDF view's C and Python documentation names the successor.
- AC-4: **Python.**
  `PhysX.evaluate_sdf_shapes(points, out, *, prim_paths=None, pattern=None, cuda_stream=0, cuda_wait_event=0)`
  takes DLPack-compatible arrays, opens an SDF_SHAPE query over the selection, waits for the evaluation and
  releases the query. `cuda_stream` and `cuda_wait_event` are the producer handoff, as for
  `WriteSession.commit`. It has no count accessor: N is the arrays' first dimension, and a mismatch raises
  `RuntimeError` naming the shape count. An empty pattern or a non-C-contiguous array raises `ValueError`; a
  pattern with an embedded NUL or an oversized component, host arrays, N above 65535 or N x Q above 2^31 - 1, or
  a runtime refusal raise `RuntimeError`.

## Test References

- [TEST-CAPI-SDF-001](../../tests/capi/TEST-CAPI-SDF-001.md)

## Code References

- ovphysx/include/ovphysx/ovphysx_types.h (`OVPHYSX_QUERY_TARGET_SDF_SHAPE`),
  ovphysx/include/ovphysx/ovphysx_geometry_query.h (`ovphysx_evaluate_sdf_shapes`): AC-1
- ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp (`sdfTensorsError`, `ovphysx_evaluate_sdf_shapes`): AC-1, AC-2
- ovphysx/include/ovphysx/experimental/ovphysx.hpp and ovphysx/src/ovphysx/PhysXWrapper.cpp
  (`PhysX::evaluateSdfShapes`): AC-1
- ovphysx/include/ovphysx/ovphysx_legacy.h and ovphysx/python/ovphysx/api.py (`create_sdf_view`): AC-3
- ovphysx/python/ovphysx/types.py (`SimObjectType.SDF_SHAPE`), ovphysx/python/ovphysx/api.py
  (`PhysX.evaluate_sdf_shapes`), ovphysx/python/ovphysx/_bindings.py and ovphysx/python/ovphysx/api.pyi: AC-4

## Dependencies

- REQ-READ-SDF-001 (ovruntime: the SDF_SHAPE rows and the evaluation)
- REQ-CAPI-SELECT-001 (`ovphysx_query_selection_t` and its validation rules)
- REQ-CAPI-ASYNC-002 (the enqueued-task model)
