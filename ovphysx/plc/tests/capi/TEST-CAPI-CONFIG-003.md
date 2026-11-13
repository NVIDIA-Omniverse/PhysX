<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-CONFIG-003
maps_to: REQ-CAPI-CONFIG-001
type: integration
---

## Scenario

The asynchronous-execution setting affects newly created instances only.

## Given

- A running runtime and the BOOL `OVPHYSX_GLOBAL_CONFIG_ASYNC_EXECUTION` setting.

## When

- Create one instance with the setting on, change it to off, and create a second.
- Submit attach, update, clone, step, a callback job and detach to the second.
- Restore the setting and exercise its Python getter/setter.

## Then

- Only the first instance owns an extra worker thread. The second executes jobs
  on its caller and returns terminal receipts; existing instances keep their
  selected mode (REQ AC-10).
- The Python key reports the configured bool and the same workerless behavior
  (REQ AC-8, AC-10).

## Automated coverage

- `WorkerLifecycle.InstanceWithoutAWorkerThreadFinishesEachCallBeforeItReturns`
  in `tests/c_unittests/test_worker_lifecycle.cpp`.
- `test_instance_without_a_worker_thread_finishes_each_call_before_it_returns`
  in `tests/python_tests/cpu_tests/test_task_completion.py`.

## Coverage limits

The setting's default/type are checked by the scoped configuration implementation
and public enum parity checks; these mode tests focus on creation-time sampling.
