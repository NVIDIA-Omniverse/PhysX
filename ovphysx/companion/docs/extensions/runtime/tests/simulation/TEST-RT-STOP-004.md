<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-STOP-004
maps_to: REQ-RT-STOP-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Play then Stop without a step leaves no latent objects for the next epoch. Verifies REQ-RT-STOP-001 AC-1, AC-4, AC-5.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- A stopped host stage with one scene and an articulation whose initialization has not advanced; lifecycle/resource probes.

## When

- Play and immediately Stop, repeat Stop, edit a host velocity while stopped, repeat Stop again, then Play/update once.

## Then

- Even the unstepped session detaches and destroys its pending backend objects and instance. (REQ-RT-STOP-001 AC-1).
- No warm-up step or automatic reattach occurs during Stop. (REQ-RT-STOP-001 AC-4).
- Repeated Stop preserves the stopped edit; the new Play creates a fresh instance/generation and loads that velocity, with no old articulation record or callback reused. (REQ-RT-STOP-001 AC-5).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:

- [test_usd_loading.cpp](../../../../../../tests/c_unittests/test_usd_loading.cpp) — AttachOvstageTwiceRejected, UnsealedAttachFailsAndSealedRetrySucceeds, AttachOvstageRejectsZeroReadOrdinal, DetachWithoutStepDropsPendingArticulations (current API reference).

Mapped requirement: [REQ-RT-STOP-001](../../requirements/simulation/REQ-RT-STOP-001.md).
