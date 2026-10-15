<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-DETACH-003
maps_to: REQ-RT-DETACH-001
type: unit
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Cleanup handles each partial construction boundary without allocating. Verifies REQ-RT-DETACH-001 AC-4.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Parameterize no Play, created instance before attach, successful attach before first step, and already-stopped session; acquired-resource counters are known.

## When

- Invoke cleanup/close twice in each fixture.

## Then

- Only acquired resources are released once; backend detach occurs only for a successful attach, instance destruction only for a created instance, and zero new instances are allocated to handle cleanup. (REQ-RT-DETACH-001 AC-4).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [test_usd_loading.cpp](../../../../../../tests/c_unittests/test_usd_loading.cpp) — AttachOvstageTwiceRejected, UnsealedAttachFailsAndSealedRetrySucceeds, AttachOvstageRejectsZeroReadOrdinal, DetachWithoutStepDropsPendingArticulations (current API reference).
- [test_lifecycle_failure_injection.py](../../../../../../tests/python_tests/test_lifecycle_failure_injection.py) — Native failure, retained ownership and terminal lifecycle cases (current API reference).

Mapped requirement: [REQ-RT-DETACH-001](../../requirements/simulation/REQ-RT-DETACH-001.md).
