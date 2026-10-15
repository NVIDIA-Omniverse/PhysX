<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-TIMESTEP-002
title: A zero-dt step leaves contact reads unchanged
status: implemented
owner: ovphysx
---

## Description

`ovphysx_step` and `ovphysx_step_sync` accept a timestep of `0`
(REQ-CAPI-TIMESTEP-001 AC-1 and AC-2 make only negative and non-finite
timesteps invalid). The runtime treats such a step as a no-op:
`physXUpdateNonRenderDispatch()` guards its work on `elapsedSecs > 0`, so a
zero-dt step neither produces nor clears contacts and leaves
`PhysXScene::mCurrentTimeStep` — the divisor the `CONTACT_SENSOR` read uses —
untouched.

The deprecated contact binding converts impulse to force with its own cached
divisor, `InstanceData::last_step_dt`. That cache must follow the same rule as
the scene's timestep, or the two public reads of the same contact disagree.
Bootstrapped from NVBugs 6904332, where both step calls wrote
`last_step_dt = (step_dt > 0) ? step_dt : 1.0f`: after a zero-dt step the
binding divided the preceding step's unchanged impulse by `1.0` instead of that
step's dt, reporting a force `1/dt` too small (0.8175 N against the
`CONTACT_SENSOR` read's 98.1 N at 120 Hz) with no error on either call.

## Acceptance Criteria

- **AC-1: The divisor survives a zero-dt step.** Neither `ovphysx_step` nor
  `ovphysx_step_sync` writes `InstanceData::last_step_dt` when `step_dt == 0`.
  The cache holds the dt of the most recent step that actually simulated, and is
  `1.0f` only until the first such step.
- **AC-2: Reads are identical across a zero-dt step.** On both step paths, and
  on CPU and GPU scenes, every contact-binding force read
  (`ovphysx_read_contact_net_normal_forces`,
  `ovphysx_read_contact_net_friction_forces`,
  `ovphysx_read_contact_normal_force_matrix`,
  `ovphysx_read_contact_friction_force_matrix`, and the detailed
  `ovphysx_read_normal_contact_data` / `ovphysx_read_friction_contact_data`
  magnitudes) returns the value it returned immediately before that step.
- **AC-3: The two public reads agree.** After a zero-dt step the binding's
  force reads stay within 5% of the `CONTACT_SENSOR` `netNormalContactForce`
  read of the same body, which the zero-dt step also leaves unchanged.
- **AC-4: No division by zero.** `last_step_dt` is never `0`, so the
  impulse-to-force conversion cannot produce a non-finite force, whatever
  sequence of zero-dt and positive-dt steps precedes a read.
- **AC-5: Documented.** The public documentation of the dt rule — the
  contact-binding block and per-function comments in `ovphysx_legacy.h`, the
  `ContactBinding` read docstrings and `PhysX.step` / `PhysX.step_sync` in
  `api.py`, the contact-binding tutorial and the developer guide — states that
  the dt comes from the last step that simulated and says what a read returns
  after a zero-dt step. No comment claims a zero-dt step produces zero impulses.

## Test References

- [TEST-CAPI-TIMESTEP-002](../../tests/capi/TEST-CAPI-TIMESTEP-002.md)

## Code References

- `ovphysx/src/ovphysx/ovphysx.cpp`: `ovphysx_step`, `ovphysx_step_sync`
  (`last_step_dt` write, AC-1).
- `ovphysx/src/include/internal/sdk/ovphysxSDK.hpp`: `InstanceData::last_step_dt`
  (AC-1, AC-4).
- `ovphysx/src/ovphysx/ovphysxContactBinding.cpp`: the five reads that load the
  divisor (AC-2, AC-4).
- `ovphysx/include/ovphysx/ovphysx_legacy.h`,
  `ovphysx/python/ovphysx/api.py`,
  `ovphysx/docs/tutorials/contact_binding.md`,
  `ovphysx/docs/developer_guide.md`: the documented dt rule (AC-5).
- `ovphysx/ovruntime/source/omni.physx/plugins/PhysXUpdate.cpp`:
  `physXUpdateNonRenderDispatch`, the `elapsedSecs > 0` guard this requirement
  mirrors — a narrowly-scoped reference across the interface, not shared
  ownership of the contract.

## Dependencies

- REQ-CAPI-TIMESTEP-001 (zero is a valid timestep for both step calls).
