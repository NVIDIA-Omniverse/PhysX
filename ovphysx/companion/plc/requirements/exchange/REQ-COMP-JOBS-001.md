<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-JOBS-001
title: "Cancellable authoring jobs and ordered live tasks"
status: implemented
owner: ovphysx
---

## Description

Snapshot computations run independently of timeline ticks. Operations on a borrowed core owner reuse its task order. An owned companion attachment has its own scheduler and exact native handle; it must not submit its work into playback's task queue or use an implicit active owner.

## Acceptance Criteria

- AC-1: **Input retention.** A job owns or leases immutable input until terminal completion; it progresses without Kit timeline callbacks and returns a typed result or failure.
- AC-2: **Cancellation.** A cancel accepted before terminal completion yields Cancelled to the caller, even if native work must finish internally; an already-terminal job retains its outcome. Close/drain waits for quiescence, suppresses later delivery and releases owned inputs/results exactly once.
- AC-3: **Owner ordering.** Borrowed core operations follow core task order and outcome collection. Owned companion sessions serialize their own submissions and map native admission/Busy/invalid-owner outcomes explicitly. Independent native owners may have simultaneous pending intervals; no process-wide simulation gate is introduced. Admission errors and execution failures remain distinguishable.
- AC-4: **Delivery.** Polling/waiting documents completion delivery thread; bindings acquire the GIL only for Python delivery and reject reentrant waits that require the held owner lock.

## Implementation scope

Mass, mapping, attachment/filter, cooking and sampling jobs retain copied input and
progress without timeline ticks. Cancellation suppresses publication while close
joins native work. Results/exceptions are delivered on the caller's thread; native
callbacks never invoke Python. Live borrowed work uses core receipt ordering;
independent previews have separate pending intervals and Busy admission. Accepted
job execution failures are distinct from rejected admission and reentrant owner
waits fail explicitly.

- AC-5: **Worker reply isolation.** One-shot worker transport uses a parent-created private working directory and an exclusively created fixed reply filename. Existing destinations are never overwritten. Command-line arguments cannot select the destination; JSON replies remain separate from diagnostic stdout.

## Test References

- [TEST-COMP-JOBS-002](../../tests/exchange/TEST-COMP-JOBS-002.md) - executed native/Python subset.

- [TEST-COMP-JOBS-001](../../tests/exchange/TEST-COMP-JOBS-001.md) - executed supported-contract scenario.

## Code References

- [Worker transport](../../../python/ovphysx_companion/_worker.py) and [reply regression tests](../../../tests/test_worker.py).

Companion implementation (native/core changes retain their own PLC ownership):

- [python/ovphysx_companion/_worker.py](../../../python/ovphysx_companion/_worker.py) - isolated sample reply transport.
- [python/ovphysx_companion/jobs.py](../../../python/ovphysx_companion/jobs.py)
- [samples/mass_properties.py](../../../samples/mass_properties.py)

Existing integration anchors for remaining scope:

- [ovphysx/include/ovphysx/ovphysx.h](../../../../include/ovphysx/ovphysx.h)
- [ovphysx/python/ovphysx/api.py](../../../../python/ovphysx/api.py)
- [ovphysx/ovruntime/include/omni/physx/IPhysxCookingService.h](../../../../ovruntime/include/omni/physx/IPhysxCookingService.h)

- [ovphysx/ovruntime/include/omni/physx/RuntimeResult.h](../../../../ovruntime/include/omni/physx/RuntimeResult.h)
- [ovphysx/ovruntime/source/omni.physx/plugins/RuntimeExecution.h](../../../../ovruntime/source/omni.physx/plugins/RuntimeExecution.h)

- [cooking.py](../../../python/ovphysx_companion/cooking.py)
- [particles.py](../../../python/ovphysx_companion/particles.py)
- [test_jobs.py](../../../tests_native/test_jobs.py)
- [test_preview.py](../../../tests_native/test_preview.py)
- [test_interaction.py](../../../tests_native/test_interaction.py)

## Dependencies

- [REQ-COMP-OWNER-001](../architecture/REQ-COMP-OWNER-001.md)
- [REQ-COMP-BUFFERS-001](REQ-COMP-BUFFERS-001.md)
- [Companion ADR-0003](../../adr/ADR-0003-data-services-and-jobs.md)
