<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-OVSTAGE-UPDATE-004
maps_to: REQ-CAPI-OVSTAGE-UPDATE-001
type: integration
---

## Scenario

An update submitted inside a CUDA graph capture is refused, because a graph replay does not repeat it,
and the capture stays valid (AC-6).

## Given

- A CUDA device and an instance with no stage attached.
- A blocking stream in the primary context, which is current on the test thread.

## When

- Begin a capture of the stream in global mode, submit an update of ordinal 1, and end the capture.
- Submit the same update again outside the capture and collect its task.

## Then

- The update inside the capture is rejected with `OVPHYSX_API_INVALID_STATE`, without a task, and the last
  error names the CUDA graph capture (AC-6).
- Ending the capture succeeds (AC-6).
- The update outside the capture is accepted, and its task fails because no stage is attached (AC-3).

## Test locations

- `ovphysx/tests/c_unittests/test_worker_lifecycle.cpp`: `UpdateFromOvstageInsideACaptureIsRefusedGpuTest`.
- `ovphysx/tests/python_tests/test_write_capture_gpu.py`: `test_update_from_ovstage_refused_inside_a_capture`, the
  same refusal inside a Warp capture, where Python raises `RuntimeError` and the captured graph still replays.
