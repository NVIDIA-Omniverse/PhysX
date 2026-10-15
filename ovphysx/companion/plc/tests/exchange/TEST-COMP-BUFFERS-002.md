<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-BUFFERS-002
maps_to: REQ-COMP-BUFFERS-001
type: integration
---

## Scenario

Copy CPU records and reject device-backed input before conversion. This covers
the copied CPU surface. GPU execution and zero-copy companion views are not advertised;
explicit device-to-host transport is covered by TEST-COMP-DEFORM-WRITE-001.

## Given

- A built SDK, immutable shape records, mutable caller sequences and a device-like
  object that fails if iterated or deep-copied.

## When

- Submit native mass/matching work, change caller sequences, and attempt device input.

## Then

- Results have documented scalar types, counts, SI or shared-frame coordinates (REQ AC-1).
- Captured inputs and copied results remain independent of caller mutation (REQ AC-2).
- Device input fails before conversion/transfer; there is no implicit synchronization (REQ AC-3).

## Executable coverage

- [tests_native/test_jobs.py](../../../tests_native/test_jobs.py):
  `test_real_mass_job_retains_input_and_terminal_result`, `test_job_rejects_device_input_before_copy`.
- [tests_native/test_points.py](../../../tests_native/test_points.py):
  `test_device_data_requires_explicit_transfer`, `test_invalid_and_failure_atomicity`.
