<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-STEP-001
maps_to: REQ-CAPI-STEP-001
type: integration
---

## Coverage limits

Recoverable overflow (REQ AC-2) requires separate qualification; this fixture
does not induce it.

## Scenario

A stopped GPU simulation must not report synchronous success with logging disabled.

## Given

- `GpuStepErrorGpuTest.SyncFailureSurvivesDisabledLogging` in
  `tests/c_unittests/test_step_errors.cpp`, in GPU readback and DirectGPU modes.
- A fresh GPU scene that completes a healthy step (REQ AC-2).

## When

- Disable logging and set the scene's PhysX CUDA context to abort mode.
- Call `ovphysx_step_sync` twice, reading each immediate error.
- Clear abort mode before teardown; scene recovery is not asserted.

## Then

- Both calls return `OVPHYSX_API_ERROR` with a CUDA diagnostic (REQ AC-1).
