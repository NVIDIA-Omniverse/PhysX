<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-ASYNC-004
maps_to: REQ-CAPI-ASYNC-003
type: integration
---

## Scenario

Clone and update jobs own their inputs and report execution-state failures as tasks.

## Given

- `WorkerClone` and `WorkerUpdate` in `tests/c_unittests/test_worker_clone.cpp`,
  a clonable scene and deterministic worker hold.

## When

- Submit clone, overwrite/free the source and target path arrays, transforms and
  environment IDs, then let the job run.
- Submit clone after an unwaited step; submit a valid operation after a failed clone.
- Submit update on an unattached instance.

## Then

- Clone uses its copied inputs after submission returns (REQ AC-1, AC-4).
- Invalid execution state fails the admitted task; later valid work still runs
  and earlier failures remain collectible (REQ AC-2, AC-3, AC-6).
- Update returns pending and reports missing attachment through completion
  (REQ AC-1, AC-3).

## Automated coverage

- `WorkerClone.OwnsInputsAfterSubmit`, `AfterUnwaitedStepFailsAsTask`,
  `RecoverableFailureThenValidWork`.
- `WorkerUpdate.ReturnsWhilePendingAndFailsWithoutAttach`.
