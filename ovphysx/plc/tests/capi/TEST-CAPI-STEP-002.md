<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-STEP-002
maps_to: REQ-CAPI-STEP-001
type: integration
---

## Scenario

An accepted asynchronous step must report a failed GPU completion through both
operation-wait paths, including when logging is disabled.

## Given

- `GpuStepErrorGpuTest.AsyncFailureSurvivesDisabledLogging` in
  `tests/c_unittests/test_step_errors.cpp`, in GPU readback and DirectGPU modes.
- A fresh GPU scene that completes a healthy step and two-step batch (REQ AC-2).

## When

- Disable logging and set the scene's PhysX CUDA context to abort mode.
- Enqueue and wait for two steps individually with `OVPHYSX_TIMEOUT_INFINITE`.
- Enqueue two further steps, each followed by an `OVPHYSX_OP_INDEX_ALL` wait.
- Clear abort mode before teardown; scene recovery is not asserted.

## Then

- Admission succeeds, but every wait returns `OVPHYSX_API_ERROR` (REQ AC-3).
- Each wait identifies exactly its failed step, reports no pending operation,
  and retains a diagnostic containing the CUDA error code (REQ AC-3).
