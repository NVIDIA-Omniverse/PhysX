<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-FAULT-003
maps_to: REQ-RT-FAULT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Failed teardown or restoration retains only recoverable ownership. Verifies REQ-RT-FAULT-001 AC-3.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Running session with copied initial state; parameterize an outstanding-work failure, detach failure, instance-destroy invocation failure and host restoration-write failure.

## When

- Request Stop or stage close, inject the selected fault, then attempt Play. Finish the failing phase using the supported cleanup path.

## Then

- The adapter reports the failed phase, retains in-use stage/snapshot ownership and blocks Play. It does not falsely report Stopped/Detached or create a replacement instance. Only completed cleanup/restoration permits Stopped/Detached; resource counts then match that state. (REQ-RT-FAULT-001 AC-3).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [test_lifecycle_failure_injection.py](../../../../../../tests/python_tests/test_lifecycle_failure_injection.py) — Native failure, retained ownership and terminal lifecycle cases (current API reference).
- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — Current attach/update/detach/step/read/query API and ownership contracts (current API reference).
- [test_step_and_write_to_ovstage.py](../../../../../../tests/python_tests/test_step_and_write_to_ovstage.py) — Utility tests for world matrices, instancer holes, shadow arrays, write failures and cache invalidation (current API reference).

Mapped requirement: [REQ-RT-FAULT-001](../../requirements/simulation/REQ-RT-FAULT-001.md).
