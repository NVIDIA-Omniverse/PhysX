<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-RT-CACHE-001
title: Limit output resource lifetime to its play session
status: proposed
owner: physics-runtime
---

## Description

Limit output resource lifetime to its play session. This is a runtime-extension adapter obligation for the single-scene ovphysx/ovstage integration, derived from the reviewed ADRs. The adapter migration is not implemented by these records.

## Acceptance Criteria

- AC-1: **Allocation lifetime.** Output caches, read views, buffers and CUDA events shall be created only for a successfully loaded play session, preserved as needed across Pause, and closed/released before backend detach on Stop/stage close; the next Play shall use fresh resources.
- AC-2: **Refresh.** A cache shall bind to one exact instance/attachment and keep copied scale/instancer baselines rather than persistent borrowed stage views; supported authored pose/scale/topology changes shall refresh/recreate its cached derivations before affected output.
- AC-3: **Snapshot independence.** Refreshing or closing an output cache shall never modify the immutable initial-state snapshot used by Stop.
- AC-4: **Completion lifetime.** Borrowed reads, queries and device buffers shall remain valid until their last dependent operation completes; writes/fences and failure cleanup shall not free in-use resources or claim an incomplete write ordinal as published.

## Test References

Specification only — the mapped adapter tests are not implemented yet; legacy/API cases below are inspiration, not claimed adapter coverage.

- [TEST-RT-CACHE-001](../../tests/simulation/TEST-RT-CACHE-001.md) — Output cache lifetime follows the play session rather than the open stage.
- [TEST-RT-CACHE-002](../../tests/simulation/TEST-RT-CACHE-002.md) — Cache refresh uses current authored scale without rewriting the Stop snapshot.
- [TEST-RT-CACHE-003](../../tests/simulation/TEST-RT-CACHE-003.md) — Failed publication releases queries without freeing in-use device buffers.

## Code References

- [Current ovphysx lifecycle API](../../../../../../include/ovphysx/ovphysx.h) — playback implementation substrate, not completed Kit adapter coverage.
- [ovstage integration](../../../../../../docs/ovstage_integration.md) — input/output ownership and synchronization.


Existing integration boundaries and source behavior to adapt; these references do not claim the new adapter already implements this requirement. New implementation paths and source annotations must be recorded when that code lands.

- [simulation.py](../../../../../../python/ovphysx/utils/simulation.py) — step_and_write_to_ovstage and OvStageOutputCache (current API reference).
- [test_step_and_write_to_ovstage.py](../../../../../../tests/python_tests/test_step_and_write_to_ovstage.py) — Utility tests for world matrices, instancer holes, shadow arrays, write failures and cache invalidation (current API reference).
- [ovphysx.h](../../../../../../include/ovphysx/ovphysx.h) — Current attach/update/detach/step/read/query API and ownership contracts (current API reference).

## Dependencies

- [ADR-0010](../../adr/ADR-0010-ovstage-simulation-output.md).
- [REQ-RT-PLAY-001](REQ-RT-PLAY-001.md).
- [REQ-RT-DETACH-001](REQ-RT-DETACH-001.md).
- [REQ-RT-OUTPUT-001](REQ-RT-OUTPUT-001.md).
