<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-FAULT-001
maps_to: REQ-RT-FAULT-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Play failure at each initialization boundary leaves no hidden session. Verifies REQ-RT-FAULT-001 AC-1.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Parameterize missing stage, snapshot validation failure, instance creation failure, zero/unsealed required attach ordinal, native attach failure and post-attach initialization failure. Also attempt Play while another legitimate owner holds the process attachment (one scene per sequential owner).

## When

- Request Play with one injected fault per fixture and inspect cleanup; separately inject a cleanup failure.

## Then

- No dynamics step occurs. Successfully acquired resources are destroyed without displacing the existing owner; successful cleanup returns Stopped, while failed cleanup retains only necessary ownership in Faulted and blocks Play. (REQ-RT-FAULT-001 AC-1).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [test_usd_loading.cpp](../../../../../../tests/c_unittests/test_usd_loading.cpp) — AttachOvstageTwiceRejected, UnsealedAttachFailsAndSealedRetrySucceeds, AttachOvstageRejectsZeroReadOrdinal, DetachWithoutStepDropsPendingArticulations (current API reference).
- [test_lifecycle_failure_injection.py](../../../../../../tests/python_tests/test_lifecycle_failure_injection.py) — Native failure, retained ownership and terminal lifecycle cases (current API reference).

Mapped requirement: [REQ-RT-FAULT-001](../../requirements/simulation/REQ-RT-FAULT-001.md).
