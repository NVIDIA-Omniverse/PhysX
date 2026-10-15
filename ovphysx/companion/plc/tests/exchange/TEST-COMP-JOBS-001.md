<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-JOBS-001
maps_to: REQ-COMP-JOBS-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-JOBS-001; executable evidence and qualification limits follow.

## Scenario

Cancel a stopped authoring job while interleaving live tasks with core operations.

## Given

- A copied mesh snapshot, a controllable long-running cook, and a core task that intentionally fails after admission.
- A callback recording thread identity, an owner-lock reentry probe and an independently scheduled companion-native attachment B.

## When

- Submit while the timeline is absent, request cancellation and drain; interleave a borrowed-owner mutation with core step/read, submit B before A completes, collect each owner's outcomes and attempt a callback-side reentrant wait.

## Then

- The stopped job progresses with stable input and emits a terminal typed outcome (REQ AC-1).
- An accepted pre-terminal cancel yields Cancelled; an already-terminal job retains its outcome. After drain, no delivery occurs and allocations return to baseline (REQ AC-2).
- Borrowed core results follow core order/collection; B has its own pending interval and same-owner conflicts report Busy without consuming peer work. Rejected admission remains distinct from accepted task failure (REQ AC-3).
- Python delivery occurs with the GIL on the documented thread, and forbidden reentry reports an error instead of deadlocking (REQ AC-4).

## Executable coverage

[test_jobs.py](../../../tests_native/test_jobs.py):

Real retained inputs, accepted execution failure, queued/running cancellation and completion races pass. Cooking/sampling tests cover other authoring jobs. Independent preview and host tests verify owner receipt ordering, Busy admission and no peer consumption; native-access interaction tests reject recursive core waits.

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.

[Worker transport regressions](../../../tests/test_worker_transport.py) cover native stdout noise before and after a reply and rejection of a reply from a failed process (AC-2/3).

Worker transport regression: [test_worker.py](../../../tests/test_worker.py) verifies a fixed reply destination in the parent-created private working directory, argument-independent destination selection, refusal to overwrite an existing destination, and JSON delivery independent of native log output (REQ AC-5).
