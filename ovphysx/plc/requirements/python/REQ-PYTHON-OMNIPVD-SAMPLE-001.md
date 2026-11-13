<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-PYTHON-OMNIPVD-SAMPLE-001
title: OmniPVD Python Sample Recording Summary
status: implemented
owner: ovphysx
---

## Description

The shipped OmniPVD Python sample reports the recording produced in its output
directory so users can locate the file for inspection.

## Acceptance Criteria

- AC-1: **Literal output directory.** When a nonempty `.ovd` file exists in the
  sample's output directory, the final summary lists its name and size without
  a missing-file warning. This holds for ordinary directory paths and paths
  containing literal pattern characters such as `[1]`.

## Test References

- TEST-PYTHON-OMNIPVD-SAMPLE-001

## Code References

- ovphysx/tests/python_samples/omnipvd_recording.py
- ovphysx/tests/python_tests/cpu_tests/test_omnipvd_sample.py
- ovphysx/docs/tutorials/omnipvd_recording.md

## Dependencies

- REQ-PYTHON-OMNIPVD-LATE-001
