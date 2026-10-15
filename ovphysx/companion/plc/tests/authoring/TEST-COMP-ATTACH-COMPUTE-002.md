<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-ATTACH-COMPUTE-002
maps_to: REQ-COMP-ATTACH-COMPUTE-001
type: integration
---

## Scenario

Match points natively, transform captured mesh frames/units, and publish returned pairs through customer USD in a separate process.

This is executed subset coverage, not the entire requirement. See its Implementation scope.

## Given

- A built/staged CPU SDK and the companion package in the locked Python environment.
- Customer-USD tests run in a separate process from native fixtures.

## When

- Execute the cases listed below through the sample guide test commands.

## Then

- Matching works without a live attachment; frame and unit conversion is explicit (REQ AC-1).
- Results preserve endpoints/revisions and deterministic copied int32 indices (REQ AC-2).
- Empty inputs, ties, signed zero, invalid geometry and no-match diagnostics have defined outcomes (REQ AC-3).
- Failure does not overwrite output; the native worker never receives USD pointers and late application is rejected (REQ AC-4).

## Executable coverage

- [tests_native/test_points.py](../../../tests_native/test_points.py).
- [tests/test_usd_helpers.py](../../../tests/test_usd_helpers.py): `test_attachment_matching_frames_units_and_invalid_results`, `test_extension_samples_run`, `test_extension_sample_cli`.
