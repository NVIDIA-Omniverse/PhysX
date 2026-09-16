<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-CUDA-002
maps_to: REQ-CAPI-CUDA-002
type: unit
---

`test_output_read_device_stream.py` exercises the public Warp output-read handoff against a
real non-default CUDA stream, and the entry point's own contract directly.

## Scenario

The public output-read adapter orders the native completion event onto Warp's current device
stream before returning a CUDA array. A column whose `wait_event` is `0` has nothing to order
against.

## Given

- A DirectGPU scene read through `PhysX.read()` while a non-default `wp.ScopedStream` is
  current.
- A recorder substituted for `ovphysx_cuda_stream_wait_event`, and direct calls to that
  entry point for the cases that are about the C API itself.

## When

- A read is taken under that stream, repeated, and taken again eight steps later.
- The entry point is called directly with queued device work outstanding, with no CUDA in
  the process, and with `event == 0`.

## Then

- The read returns the device column directly as `warp.array`, with no manual wait and no
  host sync before the return. Correctness is checked three ways, because
  finite-and-in-range passes for a stale column too: the same column copied to the host
  twice is identical, a read taken eight steps later differs, and every box is lower than
  before (REQ-PYTHON-READ-001 AC-4).
- The ordering bridge is intercepted during construction. Its arguments are the raw current
  Warp `stream.cuda_stream` handle and the group's `cuda_wait_event`, exactly once per CUDA
  group with a non-zero event in the read: one wait per group, not per column
  (REQ-PYTHON-READ-001 AC-4).
- With queued device work outstanding, the entry point returns while the stream is still
  busy, so it enqueued rather than drained (AC-1).
- With no CUDA in the process, the call returns an error carrying a retrievable message
  (AC-3). Skipped where CUDA is present: a bogus `CUevent` given to a live driver segfaults
  inside `cuStreamWaitEvent`, and no wrapper can validate an arbitrary handle.
- `event == 0` succeeds without contacting the driver (AC-2).
- A failed wait raises before any array is returned (REQ-PYTHON-READ-001 AC-4); that
  assertion is exercised by the Python read suite under TEST-PYTHON-READ-001.

## Notes

A borrowed device column or downstream view must remain referenced until queued work that
uses its storage has completed. The public read tests bind returned arrays to names across
their copies and synchronization points.
