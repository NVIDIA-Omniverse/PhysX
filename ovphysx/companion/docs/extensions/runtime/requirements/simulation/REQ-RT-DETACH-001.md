<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-RT-DETACH-001
title: Drain and release a session before forgetting its stage
status: proposed
owner: physics-runtime
---

## Description

Drain and release a session before forgetting its stage. This is a runtime-extension adapter obligation for the single-scene ovphysx/ovstage integration, derived from the reviewed ADRs. The adapter migration is not implemented by these records.

## Acceptance Criteria

- AC-1: **Ordered teardown.** Session teardown shall reject new work, finish submitted simulation/output, cancel interaction while actors are valid, close caches/release borrowed views and session callbacks, then detach the backend and destroy the adapter-owned instance.
- AC-2: **Host lifetime.** The host ovstage shall remain alive through submitted work and backend detach; the adapter shall neither release borrowed actors nor destroy host-owned stage/framework resources. Stage-close completion shall release its retained stage reference and lifecycle subscriptions.
- AC-3: **Generation isolation.** Stop shall invalidate the play generation; stage close/replacement shall additionally invalidate host-stage identity. No old pointer, hit, view, queued command or completion shall mutate or step a later generation.
- AC-4: **Partial/idempotent cleanup.** Teardown shall release only acquired resources, call backend detach only after successful attach, and never create an instance to clean up. Repeated detach and cleanup of never-played/already-stopped stages shall be harmless.

## Test References

Specification only — the mapped adapter tests are not implemented yet; legacy/API cases below are inspiration, not claimed adapter coverage.

- [TEST-RT-DETACH-001](../../tests/simulation/TEST-RT-DETACH-001.md) — Close waits for output and cleans up before releasing host ownership.
- [TEST-RT-DETACH-002](../../tests/simulation/TEST-RT-DETACH-002.md) — Old generations cannot mutate a newly played stage.
- [TEST-RT-DETACH-003](../../tests/simulation/TEST-RT-DETACH-003.md) — Cleanup handles each partial construction boundary without allocating.

## Code References

- [Current ovphysx lifecycle API](../../../../../../include/ovphysx/ovphysx.h) — playback implementation substrate, not completed Kit adapter coverage.
- [ovstage integration](../../../../../../docs/ovstage_integration.md) — input/output ownership and synchronization.


Existing integration boundaries and source behavior to adapt; these references do not claim the new adapter already implements this requirement. New implementation paths and source annotations must be recorded when that code lands.

- [ovphysx.cpp](../../../../../../src/ovphysx/ovphysx.cpp) — ovphysx_attach_ovstage, ovphysx_detach_ovstage and explicit step implementation (current API reference).
- [test_usd_loading.cpp](../../../../../../tests/c_unittests/test_usd_loading.cpp) — AttachOvstageTwiceRejected, UnsealedAttachFailsAndSealedRetrySucceeds, AttachOvstageRejectsZeroReadOrdinal, DetachWithoutStepDropsPendingArticulations (current API reference).
- [test_lifecycle_failure_injection.py](../../../../../../tests/python_tests/test_lifecycle_failure_injection.py) — Native failure, retained ownership and terminal lifecycle cases (current API reference).

## Dependencies

- [ADR-0008](../../adr/ADR-0008-ovstage-detach.md).
- [REQ-RT-STAGE-001](REQ-RT-STAGE-001.md).
