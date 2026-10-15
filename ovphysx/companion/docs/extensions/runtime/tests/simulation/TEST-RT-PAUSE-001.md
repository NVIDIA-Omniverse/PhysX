<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-RT-PAUSE-001
maps_to: REQ-RT-PAUSE-001
type: integration
---

## Coverage status

Specification only — not implemented. See **Why not implemented** below.

## Scenario

Pause and resume preserve instance, baseline and fractional time. Verifies REQ-RT-PAUSE-001 AC-1, AC-2, AC-3, AC-4.

## Given

- The [runtime fixture and oracle conventions](../../README.md#fixture-and-oracle-conventions) apply; each active stage has exactly one physics scene.
- Running scene at 60 Hz with nonzero current velocity, initial snapshot differing from current pose, r=h/2 and active grab.
- Record instance/generation, snapshot, completed time and last published matrix.

## When

- Request Pause during a pre-step callback, complete that admitted substep, send one second of paused updates, repeat Pause, then resume with dt=h/2.
- Repeat Pause in Stopped and Detached fixtures.

## Then

- Pause waits for that completed substep/publication and restores interaction properties before Paused is reported. (REQ-RT-PAUSE-001 AC-1).
- During paused updates instance/attachment, baseline, current solver state/time, remainder and last output are preserved with no drain/output work. (REQ-RT-PAUSE-001 AC-2).
- Resume uses the same session and makes one step due from the retained remainder; no reload/capture or paused-time catch-up occurs and held input does not restart a grab. (REQ-RT-PAUSE-001 AC-3).
- Duplicate Pause is harmless; Stopped/Detached variants allocate nothing. (REQ-RT-PAUSE-001 AC-4).

## Why not implemented

The ovphysx-backed Kit adapter and its lifecycle/ordinal instrumentation are not implemented in this companion design set. Existing component/legacy tests do not execute this new adapter scenario. Add the automated case with the adapter implementation; do not substitute passing legacy tests for this coverage.

Starting points and differences to preserve are recorded in the [source comparison](../../README.md#legacy-behavior-and-current-api-evidence). Relevant inspected sources:


Mapped requirement: [REQ-RT-PAUSE-001](../../requirements/simulation/REQ-RT-PAUSE-001.md).
