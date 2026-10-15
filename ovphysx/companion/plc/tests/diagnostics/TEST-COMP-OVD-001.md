<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-OVD-001
maps_to: REQ-COMP-OVD-001
type: integration
---

## Coverage status

Executable native recording/decoding and customer-USD conversion. Covers [REQ-COMP-OVD-001](../../requirements/diagnostics/REQ-COMP-OVD-001.md) AC-1 through AC-4.

## Scenario

Record a short simulation, decode it and convert frames to customer USD without Kit.

## Given

- A core recording with object create/move/delete over three known frames and a truncated malformed variant.
- Scoped core recording capability enabled before the first successful create.
- A customer USD stage and no viewer package; a companion preview attachment with a pending native interval during recording-transition probes.

## When

- Capture through the shared coordinator, try transitions while preview is pending, fetch/retire preview and check capture lifetime; decode both inputs and apply valid frames with Python conversion.
- Verify create/attach stay dormant, then start explicitly before the first step.
  Stop and attach another preview before another explicit start.

## Then

- One coordinated process capture remains coherent; pending-owner transitions report Busy, idle retry works, and preview retirement does not finalize the unrelated capture (REQ AC-1).
- Neither core nor preview attachment starts or restarts capture. Explicit start
  before the first step records it (REQ AC-1).
- Valid records preserve object lifetime/time; malformed input reports a diagnostic with no native USD dependency (REQ AC-2).
- The selected layer has expected paths, transforms, visibility and time samples (REQ AC-3).
- Conversion runs without Kit and core recording works with viewer absent (REQ AC-4).

## Executable coverage

[Native tests](../../../tests_native/test_ovd.py) exercise actual core recording
with independent preview start/stop admission, preview retirement, decode after
runtime shutdown, full attribute paths, schema/frame records, every truncated
prefix, bounded lengths, unknown versions/commands and C table/output canaries.
`test_preview_attach_does_not_start_or_restart_recording` covers dormant
attachment and explicit restart.

[USD tests](../../../tests/test_ovd.py) check selected-layer transforms/units,
visibility, handle reuse, typed array/list values, raw frame markers, explicit
timeline validation, source revision checks and atomic undo/redo. The real native
capture is converted in the customer-USD process without Kit. All four customer-USD cases pass, including explicit flag-width and unmatched
retirement diagnostics.
