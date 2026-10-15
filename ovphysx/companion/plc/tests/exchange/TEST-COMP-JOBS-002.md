<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-JOBS-002
maps_to: REQ-COMP-JOBS-001
type: integration
---

## Scenario

Submit real mass and element-mapping work and gate running/queued jobs to exercise cancellation and drain deterministically.

This is executed subset coverage, not the entire requirement. See its Implementation scope.

## Given

- A built/staged CPU SDK and the companion package in the locked Python environment.
- Customer-USD tests run in a separate process from native fixtures.

## When

- Execute the cases listed below through the sample guide test commands.

## Then

- Mutating the caller list after submission cannot change captured input, and failures reach the waiter (REQ AC-1).
- Running and queued cancellation suppress results; close waits for quiescence, is repeatable and prevents submissions (REQ AC-2).
- Results are obtained by explicit caller-thread wait with no user completion callback (REQ AC-4).

## Executable coverage

- [tests_native/test_jobs.py](../../../tests_native/test_jobs.py).

The deterministic completion-during-cancellation case in `test_jobs.py` verifies that clearing a terminal job's future cannot race the cancellation call. [test_mesh.py](../../../tests_native/test_mesh.py) verifies copied mapping inputs and retained results.
