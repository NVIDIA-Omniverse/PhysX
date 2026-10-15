<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-HOST-001
maps_to: REQ-COMP-HOST-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU/customer-USD contract.
Covers REQ-COMP-HOST-001; executable evidence and qualification limits follow.

## Scenario

Drive a headless host-adapter fixture through timeline, authoring batch and unavailable-feature actions.

## Given

- A fake timeline/selection/command interface, a step recorder, a collider job queue and capability stubs.
- A second, independently scheduled Inspector attachment with separate output; an optional later Kit integration fixture using the same adapter contract.

## When

- Enable both legacy-style stageupdate entry points, run one frame, cancel an authoring batch, perform units conversion with notice coordination and request an absent feature.

## Then

- Presentation and editor joint registration remain above the native service boundary (REQ AC-1).
- The adapter elects one coordinator per attachment and admits exactly one intended step per owner update; preview updates do not step/reset playback or consume its results (REQ AC-2).
- Cancelled jobs cannot late-author and successful batch edits/units conversion are undoable without notice feedback (REQ AC-3).
- Unavailable actions show a capability-specific result and no test/private API is acquired (REQ AC-4).

## Executable coverage

[test_host_edits.py](../../../tests/test_host_edits.py):

Five customer-USD cases cover cancellation, preflight, metric conversion, failed second-item rollback, stale/thread rejection and undo. Four native host cases in tests_native/test_host.py cover coordinator election, duplicate ticks, pause/reentry/failure and independent preview. The batch_colliders and host_stageupdate samples execute.

Qualification: Linux x86_64 CPU and customer USD 26.8. No GPU/device execution
or same-process legacy-provider coexistence is claimed.
