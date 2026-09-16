<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-CUDA-002
title: Consumer-Stream Ordering for Device Read Columns
status: implemented
owner: ovphysx
---

## Description

A device (`kDLCUDA`) read column is handed over before its producing work has necessarily
completed; `ovstage_read_group_t.data.cuda_sync.wait_event` is the event that work signals.
A consumer reading on its own stream must order that stream after the event, or it may
observe a partially written column.

Honouring that contract previously required the consumer to call CUDA directly, which a
language binding may not be able to do. `ovphysx_cuda_stream_wait_event()` supplies the
single ordering primitive through the CUDA driver shim ovphysx already loads. The public
Python output-read adapter uses it to order the producer event onto Warp's current device
stream before returning an array.

This requirement covers the C API contract only; the Python half of the handoff is
REQ-PYTHON-READ-001 AC-4. Neither covers exposing an application-owned stream as the
execution stream for PhysX or `PxDirectGPUAPI`, which remains out of scope.

## Acceptance Criteria

- AC-1: `ovphysx_cuda_stream_wait_event(stream, event)` enqueues a wait on `stream` for
  `event` and returns without synchronizing the host.
- AC-2: `event == 0` returns success without contacting CUDA, so a caller with no producer
  work to await needs no special case and a CPU-only process never reaches the driver.
- AC-3: When CUDA is unavailable in the process, the call returns `OVPHYSX_API_ERROR` with a
  message retrievable through `ovphysx_get_last_error()`, rather than failing hard.

The Python obligation this entry point exists to serve — enqueue a CUDA group's completion
event onto the current Warp stream once per group, and raise before any array is returned if
that wait fails — is [REQ-PYTHON-READ-001](../python/REQ-PYTHON-READ-001.md) AC-4. It is
stated there and not restated here.

## Test References

- TEST-CAPI-CUDA-002

## Code References

- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_cuda_stream_wait_event`)
- ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp (`ovphysx_cuda_stream_wait_event`)
- ovphysx/tests/python_tests/test_output_read_device_stream.py

## Dependencies

- [ADR-0008](../../../ovruntime/plc/adr/ADR-0008-tensor-backend-sourced-output-read.md) -
  device read columns and their readiness contract
- [ADR-0023](../../../ovruntime/plc/adr/ADR-0023-warp-python-output-read-frontend.md) -
  public Python Warp frontend
- [REQ-PYTHON-READ-001](../python/REQ-PYTHON-READ-001.md) - public return contract
