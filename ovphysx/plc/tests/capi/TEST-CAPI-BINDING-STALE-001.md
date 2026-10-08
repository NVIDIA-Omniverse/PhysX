<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-BINDING-STALE-001
maps_to: REQ-CAPI-BINDING-STALE-001
type: unit
---

## Scenario

`ArticulationMetadataTest.MetadataGettersRejectBindingStaleAfterReattach`
(`ovphysx/tests/c_unittests/test_articulation_metadata.cpp`) creates an
articulation tensor binding, detaches and reattaches the same USD stage
without destroying the binding, and asserts every metadata getter rejects
the now-stale binding handle.

## Given

A `TensorBindingCpuTest`/`ArticulationMetadataTest`-style fixture attaches
`tests/data/links_chain_sample.usda` and creates an
`OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32` binding on
`/World/articulation`. Pre-fix, `ovphysx_get_tensor_binding_spec`,
`ovphysx_get_articulation_metadata`, `ovphysx_articulation_get_dof_names`,
`ovphysx_articulation_get_body_names`, and
`ovphysx_articulation_get_joint_names` looked up the binding by handle and
read through `it->second` / `binding.artiView` unconditionally, without
ever comparing `instance->attachHandle` to `binding.attachHandle` — unlike
the read/write/prim-path entry points, which already carried that guard. The
new native-device getter follows the same stale-binding contract.

## When

The test calls all six getters once before detach (establishing a
non-trivial baseline: `dof_count > 0`, `SUCCESS` from every getter), then
calls `ovphysx_detach_ovstage` (via `destroy_ovstage_test_attachments`) and
reattaches the same USD via `attach_usd_with_ovstage` — advancing
`instance->attachHandle` to a new value while the original
`binding_handle` remains in `instance->tensor_bindings` (never destroyed).
The same six getters are then called again with the identical stale
`binding_handle`.

## Then

- All six getters return `.status == OVPHYSX_API_NOT_FOUND` on the stale
  binding after reattach. Before the stale-metadata fix, the five pre-existing
  getters returned `OVPHYSX_API_SUCCESS`
  with the pre-detach shape/dof_count/body_count/joint_count/names; the
  fix under test (the `instance->attachHandle != binding.attachHandle`
  guard, matching the pattern already used at `ovphysx_read_tensor_binding`
  and the other guarded call sites) makes the assertion pass.
- The already-guarded sibling `ovphysx_tensor_binding_get_prim_paths` also
  rejects the same stale handle, confirming the getters are now consistent
  with the existing data-path contract.
- A fresh binding created on the same pattern after the reattach succeeds
  on `ovphysx_get_articulation_metadata` with the same `dof_count` as the
  pre-detach baseline, proving the guard is attach-specific and not a
  global regression.
