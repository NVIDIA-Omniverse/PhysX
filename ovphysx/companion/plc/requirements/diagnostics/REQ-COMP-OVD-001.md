<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-COMP-OVD-001
title: "Recording reuse and Python OVD-to-USD conversion"
status: implemented
owner: ovphysx
---

## Description

Core recording remains the source of OmniPVD output. Optional decoding/viewer functionality returns native data and authors the customer stage in Python.

## Acceptance Criteria

- AC-1: **Recording.** The companion uses one process recording coordinator integrated with the core recording capability. Set the global capability before the first successful core create; create/attach/reattach remain dormant until explicit FILE/TCP start. Start before the first simulation step to capture it. Recording concerns the shared SDK, not an isolated capture per attachment. Native recording transitions respect all live owners' admission and report Busy while required owners are pending; preview teardown cannot finalize an unrelated capture. Current facade detach/finalization behavior must be integrated before advertising mixed core/companion recording.
- AC-2: **Decoding.** Native OVD decoding returns USD-independent records with object identity, lifetime and frame/time information; malformed data yields bounded diagnostics without native USD creation.
- AC-3: **Python conversion.** Python maps decoded records to prims, visibility and time samples in the caller-selected customer USD stage/layer; handle-to-prim mapping has explicit lifetime.
- AC-4: **Viewer separation.** Playback controls and presentation are host concerns; conversion is headless-capable and absence of the optional viewer does not disable core recording.

## Test References

- [TEST-COMP-OVD-001](../../tests/diagnostics/TEST-COMP-OVD-001.md) - actual native recording and customer-USD roundtrip are executable.

## Code References

- [Separate decoder C table](../../../include/ovphysx_companion/ovd.h)
- [Decoder ownership bridge](../../../src/Ovd.cpp)
- [Recording coordinator and copied records](../../../python/ovphysx_companion/ovd.py)
- [Customer-USD conversion](../../../python/ovphysx_companion/usd/ovd.py)
- [Native decoding/recording tests](../../../tests_native/test_ovd.py)
- [Customer-USD conversion tests](../../../tests/test_ovd.py)
- [Recording/import sample](../../../samples/ovd_recording.py)

## Implementation scope

The process coordinator delegates recording to core; pending preview intervals
block start/stop, and preview retirement preserves an unrelated capture. The
stopped decoder supports little-endian OVD 0.1.42 through 0.4.0 with explicit
input/record/string/payload bounds and byte-offset diagnostics. Native records
own full attribute-handle paths, schema identifiers, messages and raw scene ticks.
No USD or simulation initialization is needed for decoding.

Python conversion authors each object lifetime as a distinct Xform, typed data
attributes, recognized rigid global poses and visibility samples. The selected
layer gets schema/frame/message metadata and diagnostics for unmatched destruction.
Recorded handles resolve against explicit playback time; handle reuse cannot alias
an older prim. Default time codes use command indices, preserving stream ordering
without pretending scene-local ticks are seconds. Host-provided time conversion
must be monotone. Explicit length units scale through a reset capture root; target
stage units stay unchanged. Generic data conversion does not generate collider
meshes or supply viewer UI; those presentation policies consume the retained data.

## Dependencies

- [REQ-COMP-USD-EDIT-001](../python/REQ-COMP-USD-EDIT-001.md)
- [REQ-COMP-BUFFERS-001](../exchange/REQ-COMP-BUFFERS-001.md)
- [Companion ADR-0004](../../adr/ADR-0004-customer-usd-python.md)
