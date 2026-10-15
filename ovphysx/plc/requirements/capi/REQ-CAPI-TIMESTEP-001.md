<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-TIMESTEP-001
title: Consistent caller-provided simulation timesteps
status: implemented
owner: ovphysx
---

## Description

Synchronous and asynchronous stepping use the same timestep domain. The caller
chooses the timestep appropriate for the simulation.

## Acceptance Criteria

- **AC-1: Pass-through.** For an attached scene, `ovphysx_step` and
  `ovphysx_step_sync` pass finite, nonnegative timesteps to the runtime without
  clamping, substepping, or a one-second upper bound. The accumulated simulation
  time must remain finite.
- **AC-2: Invalid input.** Both calls reject negative and non-finite timesteps
  with `OVPHYSX_API_INVALID_ARGUMENT` before advancing simulation.

## Test References

- [TEST-CAPI-TIMESTEP-001](../../tests/capi/TEST-CAPI-TIMESTEP-001.md)

## Code References

- `ovphysx/src/ovphysx/ovphysx.cpp`: `ovphysx_step`, `ovphysx_step_sync`,
  `omni_sdk_physx_simulate_instance`.

## Dependencies

None.
