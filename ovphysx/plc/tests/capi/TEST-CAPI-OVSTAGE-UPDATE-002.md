<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OVSTAGE-UPDATE-002
maps_to: REQ-CAPI-OVSTAGE-UPDATE-002
type: integration
---

## Coverage status

Specification only - not implemented. See **Why not implemented** below.

## Scenario

The public update API reports each native update's result and its own diagnostic
without retaining a previous call's error (REQ AC-1, AC-2, AC-3).

## Given

- A valid attached ovphysx instance and valid update ranges.
- A fixture that can control `IPhysxSimulation::updateFromOvStage()` results
  and record known runtime errors synchronously during each call.

## When

- A native update records two distinct errors and returns false.
- The next native update records no error and returns false.
- A final native update records a recoverable error and returns true.

## Then

- The first C API call returns `OVPHYSX_API_ERROR`; its last error contains
  the update context and the first cause, without replacing it with the second
  cause (REQ AC-1).
- The second call returns `OVPHYSX_API_ERROR`; its last error contains the
  update context and neither earlier cause (REQ AC-2).
- The final call returns `OVPHYSX_API_SUCCESS` and leaves an empty public
  error despite the recorded diagnostic (REQ AC-3).

## Why not implemented

The current public test suite has no controlled failing
`IPhysxSimulation::updateFromOvStage()` callback fixture that emits a known
runtime error. Existing invalid-range and unattached-instance tests return
before that callback and cannot verify this boundary.

The runtime error-scope tests and
`PhysXTestFixture.UnsealedAttachFailsAndSealedRetrySucceeds` in
`ovphysx/tests/c_unittests/test_usd_loading.cpp` support the shared capture and
error-formatting behavior. They are not direct coverage of update failure
diagnostics. A controlled callback fixture or a deterministic update failure
that records a known cause is needed to automate this scenario.
