<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-READ-001
maps_to: REQ-CAPI-READ-001
type: integration
---

## Scenario

A read through the public C API on a stream of the scene's CUDA context hands over its
completion as an event, is refused under capture until a released read leaves its plan, then
is captured and replayed against the poses current at each replay (AC-2, AC-4).

## Given

- A DirectGPU instance (`/physics/suppressReadback`) and
  `boxes_falling_on_groundplane_gpu.usda`, stepped once.
- A non-blocking stream the test creates in the scene's CUDA context, and the CUDA driver's
  graph calls, loaded at run time.
- A held rigid-body query.

## When

- A `position` read with `{stream, 0}` is attempted between `cuStreamBeginCapture` and
  `cuStreamEndCapture`, before any read over the query.
- A `position` read with `{stream, 0}` is made outside a capture and released.
- The same read is captured and the graph instantiated. Twice, the scene steps five times,
  the graph is launched and the stream synchronized, and the captured column is compared with
  a read made with `{0, 0}`.

## Then

- The first read fails and the capture ends with a valid graph (REQ AC-4).
- The read outside the capture returns a device group whose `cuda_sync` is `{0, event}` with
  a nonzero event (REQ AC-2).
- The captured read succeeds, gathers into the column the read before it left on the query,
  and its group names no event; after each replay its column holds the poses the fresh read
  returns (REQ AC-2, AC-3, AC-4).

## Code References

- `ovphysx/tests/c_unittests/test_read_capture.cpp`:
  `ReadCaptureGpuTest.CapturedReadReplaysTheCurrentPoses`.

The case needs a CUDA device and runs in the GPU pass. The other refusals of AC-4 are covered
through the Python surface by [TEST-PYTHON-READ-002](../python/TEST-PYTHON-READ-002.md). AC-1's
device ordering and AC-3's release without a host wait are covered only in the runtime, by
[TEST-READ-DEVICE-003](../../../ovruntime/plc/tests/output/TEST-READ-DEVICE-003.md).
