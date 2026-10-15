<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-TASK-004
maps_to: REQ-CAPI-TASK-001
type: integration
---

## Scenario

GPU simulation failure survives task collection with logging disabled.

## Given

- `GpuStepErrorGpuTest.AsyncFailureSurvivesDisabledLogging` in
  `tests/c_unittests/test_step_errors.cpp`, in GPU readback and DirectGPU modes.
- A fresh GPU scene that completes a healthy step.

## When

- Disable logging, set the scene's PhysX CUDA context to abort mode, and submit
  `ovphysx_step`.
- Collect the accepted task through `ovphysx_wait_task`.
- Submit and collect another step while the context remains in abort mode.
- Clear abort mode before teardown; scene recovery is not asserted.

## Then

- Collection returns `OVPHYSX_API_ERROR` with the submitted task's identity
  (REQ AC-6).
- The wait result retains a CUDA diagnostic accessible through
  `ovphysx_get_last_task_error` until result destruction (REQ AC-5, AC-6).
