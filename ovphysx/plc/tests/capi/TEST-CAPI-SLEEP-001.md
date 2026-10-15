<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-SLEEP-001
maps_to: REQ-CAPI-SLEEP-001
type: integration
---

## Scenario

C classification and Python reads expose sleeping state with the existing ownership contract.

## Given

- The C and Python writability APIs, which require no attached scene.
- Dynamic rigid bodies and the normal Python session API on DirectGPU.
- A separate CPU-only test process with dynamic rigid bodies and working wake writes.

## When

- Ask writability for `isSleeping` and `wake` on rigid bodies and articulation links.
- Attempt an `isSleeping` write through Python.
- Read `isSleeping` by name before and after the first DirectGPU step, then by token.
- On CPU, put bodies to sleep and retain the named read's array after leaving its
  context. Wake them and read by token, then step again.

## Then

- `isSleeping` is READ_ONLY on both types; existing wake classifications are unchanged (REQ AC-1).
- Whole-articulation `isSleeping` remains UNCLASSIFIED, and Python rejects a rigid-body
  `isSleeping` write with an attribute-specific diagnostic (REQ AC-1).
- DirectGPU emits no pre-step groups, then a fixed host uint8 scalar per returned
  rigid-body row, all zero; the token read preserves the attribute and values (REQ AC-2).
- On CPU, the named read returns ones and the later token read returns zeros. The
  retained array still contains ones after the wake write, new read, and next step
  (REQ AC-2).

## Automated coverage

- `tests/c_unittests/test_writability.cpp`: `Writability.RigidBody`,
  `Writability.ArticulationLink`, `Writability.UnknownAndCrossTypeAreUnclassified`,
  and `Writability.EveryDeclaredAttributeIsClassifiedSomewhere` (AC-1).
- `tests/python_tests/test_writability.py`:
  `test_writability_matches_the_write_api` sleeping-state and wake cases (AC-1).
- `tests/python_tests/test_write_api.py`: `test_unwritable_attribute_raises` (AC-1).
- `tests/python_tests/test_output_read.py`:
  `test_sleeping_state_is_host_uint8_and_accepts_tokens` (AC-2).
- `tests/python_tests/cpu_tests/test_sleep_state.py`:
  `test_sleeping_state_changes_leave_retained_python_snapshot_unchanged` (AC-2).
- The Python reads exercise the C transport through the binding. This focused
  frontend coverage does not independently recheck articulation-link state or
  disabled-row selection; those are runtime behaviors.
