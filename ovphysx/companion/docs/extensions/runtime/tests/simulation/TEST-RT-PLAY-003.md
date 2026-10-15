<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-PLAY-003
maps_to: REQ-RT-PLAY-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

The play epoch retains the borrowed ovstage until teardown. Verifies REQ-RT-PLAY-001 AC-2.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- One live ovstage, host-reference counters and an in-flight output fence after Play.

## When

- Release the caller temporary reference and request Stop while the fence remains pending; complete the fence.

## Then

- The stage remains alive through completion and backend detach; only session-owned references are released, and the host stage remains available in Stopped. (REQ-RT-PLAY-001 AC-2).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — Current attach/update/detach/step/read/query API and ownership contracts (current API reference).
- [test_usd_loading.cpp](../../../../../../tests/c_unittests/test_usd_loading.cpp) — AttachOvstageTwiceRejected, UnsealedAttachFailsAndSealedRetrySucceeds, AttachOvstageRejectsZeroReadOrdinal, DetachWithoutStepDropsPendingArticulations (current API reference).

Mapped requirement: [REQ-RT-PLAY-001](../../requirements/simulation/REQ-RT-PLAY-001.md).
