<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-PREVIEW-002
maps_to: REQ-COMP-PREVIEW-001
type: integration
---

## Coverage status

Native CPU integration executed in [test_preview.py](../../../tests_native/test_preview.py). Covers
[REQ-COMP-PREVIEW-001](../../requirements/live/REQ-COMP-PREVIEW-001.md)
AC-1, AC-5 and AC-6.

## Scenario

Run Inspector and playback with independently pending native intervals and
reject an incompatible preview candidate without disturbing playback.

## Given

- Playback A and preview B use distinct ovstages in one loaded runtime, with
  identical body/scene paths, different initial positions, gravity and clocks.
- The companion bridge resolves both exact attachment handles. Per-owner
  callback, query, debug-frame and allocation recorders are available.
- Compatible alternate thread-count options, an incompatible tolerance-scale
  candidate C and a recording-provider conflict candidate are prepared.
- A fixture barrier can retain pending native work; supported GPU/device cases
  use an explicit backend matrix rather than assuming any device is available.

## When

- Attach B through the preview service, submit A and B before fetching either,
  and fetch them independently; repeat on independent native client threads.
- Attempt conflicting mutation of pending B and lifecycle mutation from an
  application callback. Read permitted immutable metadata during pending work.
- Read each owner's state, query hits, event stream and completed debug frame.
- Create incompatible candidates and supported owner-local resource variants;
  compare A's configuration, objects and output before and after each attempt.

## Then

- B is created through the companion runtime bridge alongside independent public
  attachments, and A remains attached to the original stage. (REQ AC-1)
- Each body follows only its own gravity/time; queries, callbacks and output
  identify the selected owner despite equal paths. Both native intervals can
  remain pending; conflicting same-owner work and callback reentry return Busy
  without consuming peer results or blocking. (REQ AC-5)
- Compatible copied resource choices are retained per attachment. Incompatible
  tolerance/provider candidates publish no usable handle, release their own
  resources and leave A unchanged; global defaults and hard CPU-only policy are
  preserved. Unsupported device combinations are reported explicitly. (REQ AC-6)

## Remaining coverage

GPU/device execution is not qualified or advertised by the CPU host adapter.
Owner-local queries/events/views and recording lifetimes are covered by the
inspection, events, debug, OVD and interaction suites. Customer-USD output and
Play/cancel transitions are covered by TEST-COMP-PREVIEW-001.

## Executable coverage

- [Executed subset cases](../../../tests_native/test_preview.py). Remaining scenario obligations are listed above.
