<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-PYTHON-OMNIPVD-SAMPLE-001
maps_to: REQ-PYTHON-OMNIPVD-SAMPLE-001
type: unit
---

## Scenario

The shipped OmniPVD sample reports an existing recording even when its output
directory contains literal brackets.

## Given

- Separate ordinary and bracketed output directories.
- A recording dependency that writes a 2048-byte `capture.ovd` to the sample's
  exact FILE destination, without requiring the native runtime.

## When

- Execute `tests/python_samples/omnipvd_recording.py` for each directory and
  capture its printed summary.

## Then

- Each output directory contains a nonempty `capture.ovd` (REQ AC-1).
- Both summaries list one OVD file by name and report its 2.0 KB size, and
  neither prints the missing-file warning (REQ AC-1).

## Executable coverage

- `tests/python_tests/cpu_tests/test_omnipvd_sample.py::OmniPvdSampleTests::test_recording_summary_handles_literal_brackets`
