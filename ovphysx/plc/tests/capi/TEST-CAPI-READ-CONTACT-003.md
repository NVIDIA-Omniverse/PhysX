<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-READ-CONTACT-003
maps_to: REQ-CAPI-READ-CONTACT-002
type: integration
---

## Scenario

A contact query is opened through the C API and through `PhysX.query` with `contacts=` and the explicit `PhysX.read_sync` convenience: malformed
arguments are refused up front, a filter the runtime cannot pair fails with the runtime's message, and a
valid query reads its sensors in selection order on both residencies.

## Given

- C: an ovphysx instance, then `tests/data/boxes_falling_on_groundplane.usda` attached (eleven cubes
  carrying `PhysxContactReportAPI`, and a static `/World/BigBase`).
- Python: the `write_contact_sensor_usda` scene (`/World/Light` 1 kg and `/World/Heavy` 2 kg resting on
  `/World/Ground` with the API, `/World/Quiet` without it) on a CPU instance and on a DirectGPU instance,
  stepped 60 times at 1/60 s.

## When

- C, before any stage: `ovphysx_query_contacts` with a NULL `out_query`, NULL `selections`, a zero count,
  21846 filters, NULL filters for one filter per sensor, and a sensor or filter selection that is empty,
  has a count without handles, holds `OVX_INVALID_PRIMPATH`, or has a pattern with an embedded NUL or no
  leading `/`.
- C, with the stage: sensors `/World/Cube*` with the filter `/World/*`, then with the filter
  `/World/BigBase`, requesting an optional query summary.
- Python: `contacts=[/World/Heavy, /World/Quiet, /World/Light]`, each with the filter `/World/Ground`;
  `/World/Heavy` with the filter `/World/*`; and malformed `contacts=` arguments.
- Python: `/World/Heavy` with filters (`/World/Ground`, `/World/Light`) and `/World/Light` with filters
  (`/World/Ground`, `/World/Heavy`), reading `normalContactForceMatrix` and `frictionContactForceMatrix`.
- Python: the same two sensors and filters, reading `filteredContactCount`, `filteredContactForce`,
  `filteredContactNormal` and `filteredContactPoint`.
- Python, DirectGPU, with `/World/Light` starting at z = 3 instead of resting: four reads of
  `normalContactForceMatrix` with `contacts=[/World/Light, /World/Heavy]`, each with the filter
  `/World/Ground`, after 10, 150, 151 and 152 steps, each read's array kept after its result closes.
- `ovphysx_writability(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, ...)` for both matrix names and the eight filtered
  detail names, from C and Python.

## Then

- Every malformed C call returns `OVPHYSX_API_INVALID_ARGUMENT` at once with no task issued (`op_index` 0),
  and `out_query` stays 0 (AC-1).
- `/World/*` for eleven cubes fails the task with `OVPHYSX_API_ERROR`, and the task error contains "one
  object per sensor" and the query handle and summary remain empty; the shared
  `/World/BigBase` opens and releases, with the summary's reserved `query_result_id` and
  `all_handle` remaining zero (AC-1, AC-2).
- The Python read returns `/World/Heavy` then `/World/Light` (`/World/Quiet` reports nothing), each at its
  weight within 5%, as CPU and as CUDA columns (AC-3).
- `/World/*` raises a `RuntimeError` naming the rule; each malformed `contacts=` raises `ValueError` before
  a native call (AC-3).
- Both matrices are `(2, 6)` arrays (CPU, and CUDA on DirectGPU) with rows `/World/Heavy` then
  `/World/Light`: each sensor's ground column holds its weight upward within 5%, its other-sensor column is
  ~0, and friction is ~0. Both names classify `OVPHYSX_WRITABILITY_READ_ONLY` (AC-4).
- The count group is a `(2, 2)` int32 array (CPU, and CUDA on DirectGPU): each sensor's ground lane is
  positive and its other-sensor lane 0. Each detail group is a list of one array per sensor, shape `(n,)`
  for the force and `(n, 3)` for the normal and point, with n its ground count; the sum of force × normal z
  is its weight within 5%, and each point lies on the ground's top face (|z| < 0.05). All eight detail
  names classify `OVPHYSX_WRITABILITY_READ_ONLY` (AC-5).
- Each repeated read returns a CUDA `(2, 3)` array with rows `/World/Light` then `/World/Heavy`. The first
  reads exactly 0 for the falling `/World/Light`, the later ones each sensor's weight within 5%, and after
  the last read every kept array still holds the values it was read with (AC-3).

## Test locations

- ovphysx/tests/c_unittests/test_query_contacts.cpp (`QueryContactsTest.*`)
- ovphysx/tests/python_tests/cpu_tests/test_contact_query_cpu.py
- ovphysx/tests/python_tests/test_contact_query.py
- Contact-binding ports (ADR-0044): CPU `cpu_tests/test_contact_query_cpu.py`
  (`test_cpu_contact_query_friction_detail_after_sliding`, `test_cpu_filtered_attributes_absent_on_an_unfiltered_query`,
  `test_cpu_contact_separation_is_finite`) and DirectGPU `test_contact_query.py`
  (`test_contact_query_friction_detail_after_sliding_device`).
- ovphysx/tests/c_unittests/test_writability.cpp (`Writability.ContactSensor`) and
  ovphysx/tests/python_tests/test_writability.py (the contact-sensor cases)
