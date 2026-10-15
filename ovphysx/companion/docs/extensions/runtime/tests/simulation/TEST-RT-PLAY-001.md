<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-PLAY-001
maps_to: REQ-RT-PLAY-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

First Play loads the latest sealed contents and does not step. Verifies REQ-RT-PLAY-001 AC-1, AC-2, AC-3, AC-4.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Stopped host stage with a sphere at (0,0,4), then authored at (0,0,8) after open; a sealed input ordinal for those edits and a separate unrelated unsealed attribute.

## When

- Play, inspect attach arguments and the created object population, and issue duplicate start while Running before any update. Then Pause and resume, recording instance and snapshot identity.

## Then

- Exactly one new instance and one attach use the live stage and nonzero sealed ordinal containing position 8; no file-reload call occurs. (REQ-RT-PLAY-001 AC-1).
- Required initial input is sealed and consumed once; unrelated unsealed data is not drained. Running is reported only after initialization. (REQ-RT-PLAY-001 AC-2).
- Step count, solver elapsed time, remainder and output-write count are zero. (REQ-RT-PLAY-001 AC-3).
- Duplicate start and paused resume preserve the instance, generation and original snapshot without another attach. (REQ-RT-PLAY-001 AC-4).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [test_usd_loading.cpp](../../../../../../tests/c_unittests/test_usd_loading.cpp) — AttachOvstageTwiceRejected, UnsealedAttachFailsAndSealedRetrySucceeds, AttachOvstageRejectsZeroReadOrdinal, DetachWithoutStepDropsPendingArticulations (current API reference).

Mapped requirement: [REQ-RT-PLAY-001](../../requirements/simulation/REQ-RT-PLAY-001.md).
