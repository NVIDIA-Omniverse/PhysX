<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-TIMESTEP-001
maps_to: REQ-CAPI-TIMESTEP-001
type: integration
---

## Scenario

Synchronous and asynchronous steps accept the caller's timestep consistently.

## Given

- `test_step_variants` and `test_step_large_dt_advances_full_time` in
  `ovphysx/tests/python_tests/test_simulation.py`, reused by
  `ovphysx/tests/python_tests/cpu_tests/test_step_boundary_conditions.py`.
- Separate CPU and GPU processes; the motion scene has one body with velocity
  (1, 0, 0), no gravity, and no damping.

## When

- Exercise both stepping methods with zero, ordinary, and above-one-second
  timesteps, and with negative, NaN, and infinite timesteps.
- Read the body's position after a 1.001-second or 2-second step completes.

## Then

- Finite, nonnegative test timesteps succeed; the body's displacement is
  (dt, 0, 0), demonstrating that the supplied timestep is not clamped (REQ AC-1).
- Negative and non-finite timesteps raise an error (REQ AC-2).
