<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-COMP-INTERACTION-001
maps_to: REQ-COMP-INTERACTION-001
type: integration
---

## Coverage status

Implemented and executed for the advertised CPU surface. Covers [REQ-COMP-INTERACTION-001](../../requirements/live/REQ-COMP-INTERACTION-001.md) AC-1 through AC-4.

## Scenario

Drag a body under collision constraints and cancel while its object is deleted.

## Given

- A rigid body with saved axis locks, CCD flags and velocities, plus a fixture-defined impulse-at-point expected response.
- A scoped native lease and a forbidden reentrant wait probe.

## When

- Apply the impulse, begin drag, update its target, cancel/delete, and try old native access and reentrant waiting.

## Then

- The lease exposes only the correct typed object and stale access after deletion is rejected (REQ AC-1).
- The impulse produces the expected linear/angular change; unsupported backend modes fail explicitly (REQ AC-2).
- Temporary constraints are gone, settings restore and velocities follow the documented end policy (REQ AC-3).
- Mutation order matches core tasks and the reentrant wait reports an error without blocking (REQ AC-4).

## Executable coverage

- [C++ access](../../../../tests/c_unittests/test_companion_access.cpp): `CompanionNativeAccess.BuildNegotiationPreservesRejectedOutput`, `CompanionAccess.TypedBorrowRejectsRecursiveWaitAndStaleRecord`; incompatible SDK/type, stale records, callback admission, wait rejection and the compiled native consumer sample (AC-1/4).
- [Python integration](../../../tests_native/test_interaction.py): `test_impulse_at_point_and_exact_instance`, `test_drag_collision_end_velocity_and_abi`, `test_drag_pending_peer_and_owner_retirement`, `test_interaction_extension_sample` (AC-2/3/4).

The native shard and all four Python cases pass. CPU-only support is explicit;
GPU/DirectGPU interaction is not advertised. No allocation-failure injector is
used: rollback is exercised through invalid/duplicate/stale admission and the
native implementation also guards later construction with RAII.

## Review regression

`test_joint_drag_wakes_sleeping_body_on_begin_and_move` lets a body sleep, begins
an explicit D6 grab, lets the grab settle, moves its target, and verifies wake-up
and actual displacement. Force grabbing remains the default and retains its
existing substep/anchor/damping regressions.

`test_force_ray_endpoint_rejects_joint_handle_without_mutation` calls the force
ray endpoint with a D6 handle, verifies InvalidArgument, and checks both the output
canary and joint target remain unchanged.
