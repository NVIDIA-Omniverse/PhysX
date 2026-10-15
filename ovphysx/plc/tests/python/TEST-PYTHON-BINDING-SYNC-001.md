<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-BINDING-SYNC-001
maps_to: REQ-PYTHON-BINDING-SYNC-001
type: integration
---

## Scenario

A delayed producer fills CUDA buffers while a synchronous binding read or write
begins, covering direct Warp arrays and the DLPack stream handoff on repeated calls.

## Given

- A GPU rigid-body velocity binding, CUDA Warp buffers and a non-default Warp stream.
- The buffers passed directly or through DLPack producers that forward stream
  handoffs to Warp, with uncached and cached-provider variants and no additional
  array framework dependency.
- Separate-stream DLPack producers whose stream is not current at the binding call.
- A delayed kernel followed by a buffer fill; an event confirms the fill is still pending.

## When

- Read into the pending destination twice using the same array.
- Write pending source data twice with full, indexed and masked selection.

## Then

- The destination contains physics values after the producer finishes, never the
  producer's zero fill (REQ AC-1).
- Every selected row contains the newly produced input value (REQ AC-2).
- Both the first and reused-buffer calls succeed, including cached Warp and
  DLPack descriptors (REQ AC-3).
- A wait that returns without completing the handoff event raises before native access
  (REQ AC-1).
- Work enqueued after the handoff event does not cause a spurious wait failure
  (REQ AC-1).

## Test Location

- `ovphysx/tests/python_tests/test_tensor_warp_smoke_gpu.py`:
  `test_read_waits_for_warp_stream_with_cached_buffer`,
  `test_write_waits_for_warp_stream_with_cached_buffer`,
  `test_producer_wait_must_complete`,
  `test_binding_handoff_from_separate_producer_stream`, and
  `test_producer_wait_ignores_later_stream_work`.
