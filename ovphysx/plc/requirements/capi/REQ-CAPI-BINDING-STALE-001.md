<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-BINDING-STALE-001
title: Tensor Binding Metadata Getters Reject Bindings Invalidated by Detach/Reattach
status: implemented
owner: ovphysx
---

## Description

A `TensorBindingState` created by `ovphysx_create_tensor_binding` records
the `AttachHandle` that was live at creation time
(`binding.attachHandle = instance->attachHandle`). Detaching and
reattaching a stage (`ovphysx_detach_ovstage` / `ovphysx_attach_ovstage`)
advances `instance->attachHandle` to a new value while leaving any
still-undestroyed binding in `instance->tensor_bindings` — the map entry,
and the `TensorAPI` views it wraps, are stale once the attach they were
created against is gone.

Every entry point that reads through a binding's `TensorAPI` views must
detect this staleness by comparing `instance->attachHandle` against
`binding.attachHandle` after the map lookup, and reject the call with
`OVPHYSX_API_NOT_FOUND` before touching the (possibly dangling) views —
the same contract `ovphysx_read_tensor_binding`,
`ovphysx_write_tensor_binding`, `ovphysx_write_tensor_binding_masked`,
`ovphysx_articulation_update_kinematic`, `ovphysx_rigid_body_view_wake_up`,
`ovphysx_rigid_body_view_sleep`, and
`ovphysx_tensor_binding_get_prim_paths` already honor. This requirement
extends that contract to the binding metadata surface, so a caller that
retains a binding handle across detach/reattach gets a consistent
"recreate binding" rejection from every entry point, not silent stale data
from some and a rejection from others.

## Acceptance Criteria

- AC-1: `ovphysx_get_tensor_binding_spec` returns `OVPHYSX_API_NOT_FOUND`
  when `instance->attachHandle != binding.attachHandle`, checked after the
  binding lookup and before computing the tensor spec.
- AC-2: `ovphysx_get_articulation_metadata` returns the same
  `OVPHYSX_API_NOT_FOUND` rejection for a stale binding, checked before the
  `!binding.artiView` "not an articulation binding" check so a stale
  binding is reported as stale rather than as the wrong binding kind.
- AC-3: `ovphysx_articulation_get_dof_names` rejects a stale binding under
  the same ordering as AC-2.
- AC-4: `ovphysx_articulation_get_body_names` rejects a stale binding under
  the same ordering as AC-2.
- AC-5: `ovphysx_articulation_get_joint_names` rejects a stale binding
  under the same ordering as AC-2.
- AC-6: `ovphysx_get_tensor_binding_native_device` rejects a stale binding
  before reading the binding's simulation view.

## Test References

- TEST-CAPI-BINDING-STALE-001

## Code References

- ovphysx/src/ovphysx/ovphysxTensorBinding.cpp (`ovphysx_get_tensor_binding_spec`,
  `ovphysx_get_tensor_binding_native_device`,
  `ovphysx_get_articulation_metadata`, `ovphysx_articulation_get_dof_names`,
  `ovphysx_articulation_get_body_names`, `ovphysx_articulation_get_joint_names`)

## Dependencies

- None
