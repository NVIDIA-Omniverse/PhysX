<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-READ-001
maps_to: REQ-PYTHON-READ-001
type: integration
---

## Scenario

The high-level output-read API is exercised on CPU and CUDA scenes through the
real native runtime. Focused tests cover scalar dtype and lane mapping.

## Given

- CPU and DirectGPU scenes with fixed-width and array-valued output columns.
- Native DLTensor descriptors covering supported and unsupported dtype, device,
  lane, and empty combinations.
- A real non-default Warp CUDA stream when a CUDA device is available.

## When

- `PhysX.read()` and `read_tokens()` drain tensors and index maps.
- A result is closed while returned arrays and Warp slices remain referenced.
- A CUDA result is created under a non-default current Warp stream.
- The converter encounters an unsupported dtype or device, or a failed stream wait.
- Runtime dependencies and public type stubs are inspected.

## Then

- CPU and CUDA tensors and maps use the Warp types, dtype, shape, pointer, and
  device required by REQ AC-1 and AC-2.
- Non-empty arrays share the native pointer and remain usable after result close;
  empty arrays do not retain a native lease (REQ AC-3).
- The real or recorded current Warp stream waits once on the matching producer
  event without a host-side synchronization, and wait failure raises before a
  result is returned (REQ AC-4).
- Explicit close and context exit release group metadata and due session cleanup
  consistently (REQ AC-5). **Foreign-thread finalization is not covered here**: the two
  threaded cases drive `release_borrow` directly against a stand-in holder rather than
  letting a real Warp deleter run on another thread, so the deferral path itself is
  asserted only on the owning thread.
- Package metadata and public stubs require and expose the bounded Warp frontend
  without a direct NumPy read-result declaration (REQ AC-6).

## Test location

`ovphysx/tests/python_tests/test_output_read.py` (AC-1..AC-3, AC-6),
`test_output_read_device.py` and `test_output_read_device_stream.py` (AC-4, the CUDA
event handoff), `test_output_read_lifetime.py` (AC-5, borrow and session lifetime), and
`test_type_stubs.py` for the stub surface AC-6 names.
