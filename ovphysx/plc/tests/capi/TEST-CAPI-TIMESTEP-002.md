<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-TIMESTEP-002
maps_to: REQ-CAPI-TIMESTEP-002
type: integration
---

## Scenario

A zero-dt step changes neither the contact-binding reads nor the
`CONTACT_SENSOR` read of the same resting body.

## Given

- `test_contact_reads_unchanged_across_zero_dt_step` in
  `ovphysx/tests/python_tests/cpu_tests/test_tensor_bindings_api.py`,
  parametrized over the sync and async step paths, with a contact binding on
  `/World/Cube1` filtered against `/World/BigBase` in
  `boxes_falling_on_groundplane.usda`.
- `test_cpu_contact_force_survives_a_zero_dt_step` in
  `ovphysx/tests/python_tests/cpu_tests/test_output_read_contacts_cpu.py`, with
  the `PhysxContactReportAPI` stage whose `/World/Heavy` (2 kg) and
  `/World/Light` (1 kg) rest on the ground.
- Both settle at `dt = 1/240 s`, far from the `1.0` the defect substituted, so a
  clobbered divisor shows as a 240x error rather than rounding.

## When

- Settle the bodies with positive-dt steps and read the net normal force, the
  normal force matrix, the detailed normal contact forces, and the
  `netNormalContactForce` column.
- Take one zero-dt step — `step_sync(0.0)` on one path, `step(0.0)` plus
  `wait_all()` on the other — and read everything again.
- Take one further positive-dt step at the settle dt and read a third time.

## Then

- Every binding read after the zero-dt step equals its value from before that
  step to within 1e-5 (REQ AC-1, AC-2); before the step the baseline is
  asserted non-zero, so the comparison cannot pass vacuously.
- Those post-step values stay within 5% of the reads taken after the following
  positive-dt step, so "unchanged" cannot be satisfied by both reads moving to
  the same wrong value (REQ AC-1).
- `netNormalContactForce` stays within 5% of each body's weight across a zero-dt
  step on both step paths, which is the same value the binding reports, so the
  two public reads agree (REQ AC-3).
- All reads remain finite (REQ AC-4).
