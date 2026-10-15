<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-SELECT-001
maps_to: REQ-CAPI-SELECT-001
type: integration
---

## Coverage status

Implemented. The C precedence case lives in `tests/c_unittests/test_output_read.cpp`
(`QuerySelectionPrimPathsWinsOverPattern`): with both a `prim_paths` entry and a glob
that matches every cube, `total_prim_count` is 1 — `prim_paths` wins, the pattern is
ignored. `QuerySelectionRejectsInvalidSelectionsOnTheReceipt` calls
`ovphysx_query_selection` with each malformed selection and a NULL one, and checks the
receipt itself — `OVPHYSX_API_INVALID_ARGUMENT`, no op index, no query — so a failure moved
into the task would fail it. Its NULL-`prim_paths`-with-a-count case carries a valid
pattern, which must not be used as a silent fallback. The Python cases live in
`tests/python_tests/test_output_read.py` (subset, precedence, empty-match, the
`scope=ACTIVE` rejection, write) and its CPU mirror `cpu_tests/test_output_read_cpu.py`,
including byte-identical parity against a deprecated tensor binding over the same
selection. The selected kinematic update of AC-5 is covered on CPU by the
`SelectedKinematicUpdateCpuTest` cases in `tests/c_unittests/test_articulation_kinematic_isolation.cpp`
and on DirectGPU by `test_articulation_kinematic_selection_gpu.py`, through the
`update_articulations_kinematic` kwargs of AC-6, whose rejections
`cpu_tests/test_articulation_kinematic_selection.py` checks.
`cpu_tests/test_selection_object_types.py` covers AC-7 over
`tests/data/articulation_selection.usda`: articulation selections against a fixed table of the
expected articulations and an articulation tensor binding over the same pattern or path, and
link, joint and rigid-body selections
that stay on their own objects.
The public session Python surface adds DirectGPU selected-write isolation: `test_session_deformable_gpu.py`
writes a `prim_paths` subset and leaves the other bodies/materials unchanged within 1e-4 (AC-5, AC-6), and
`test_session_properties_gpu.py` runs pattern-selected DOF/body/wrench writes, including link and joint
selections (AC-5, AC-6).

## Scenario

The public `ovphysx_query_selection` entry point selects a prim subset for both read and
write with `prim_paths`-wins precedence, rejects malformed selections, treats a no-match
selection as a valid empty query, and is mirrored by the Python `prim_paths=` / `pattern=`
kwargs — producing values byte-identical to a deprecated tensor binding over the same
selection — and selects an articulation through any of its links or joints, as the
articulation binding does (REQ-CAPI-SELECT-001 AC-1..AC-7).

## Given

- An ovphysx instance attached to a fixture with several addressable prims
  (`two_articulations.usda` and a multi-rigid stage), stepped.
- The deprecated tensor-binding read/write over the same pattern/path list as the
  byte-identical parity oracle.

## When

- Call `ovphysx_query_selection` with an explicit `prim_paths` subset; read; open a
  write over the same query, scatter, commit, step.
- Call it with a glob `pattern`; read.
- Call it with **both** `prim_paths` and `pattern`.
- Call it with a NULL `selection`, with an empty descriptor (no pattern, zero
  `prim_paths`), with an oversized-component / embedded-NUL `pattern`, with a
  `prim_paths` list containing `OVX_INVALID_PRIMPATH`, with a NULL `prim_paths` and a
  non-zero count beside a valid `pattern`, and with a `pattern` that does not start with `/`;
  inspect the returned receipt.
- Call it with a selection that matches nothing.
- From Python, call `read_sync` / `write_sync` with `prim_paths=` and with `pattern=`, and
  with neither; call them with a selection and `scope=ACTIVE`, and `read_sync` with
  `scope=ACTIVE, prim_paths=[]` (no pattern).
- Over two articulations with every joint position written, call
  `update_articulations_kinematic` with a selection naming one articulation (by its root, a
  link or a joint), naming both, with none, and with a selection and `scope=ACTIVE`, on CPU
  and on DirectGPU.
- Over two two-link articulations (one rooted on a transform above its links with a fixed
  base, one rooted on its root link and floating) and a standalone box, select articulations
  with patterns naming roots, links, joints, the fixed-base anchor joint and globs over them,
  and with one-path lists naming each link and joint; compare with an articulation tensor
  binding over the same selection. Select links, joints and rigid bodies with patterns and
  paths naming other members of the articulations.

## Then

- `ovphysx_query_selection` is a distinct entry point taking no scope; `ovphysx_query`
  keeps its whole-set behavior, and both take the optional creation result. A NULL
  `selection` returns `OVPHYSX_API_INVALID_ARGUMENT` (AC-1).
- With both fields set, `prim_paths` is used and `pattern` ignored; an empty descriptor
  returns `OVPHYSX_API_INVALID_ARGUMENT` (AC-2).
- A `pattern` with an oversized component, an embedded NUL or no leading `/`, a
  `prim_paths` list holding `OVX_INVALID_PRIMPATH`, a NULL `prim_paths` with a non-zero
  count (even beside a valid pattern), an empty descriptor or a NULL `selection` is
  rejected on the receipt: `OVPHYSX_API_INVALID_ARGUMENT`, no op index, and no query (the
  oversized case names the limit in the error message) (AC-1, AC-3).
- The no-match selection's creation result reports `total_prim_count == 0`, and its first
  read fetch completes successfully with a NULL group, not an error (AC-4).
- The selected read returns only the selected rows and the selected write leaves
  unselected superset rows bit-identical after commit + step — both byte-identical to the
  tensor-binding oracle over the same selection (AC-5).
- The selected kinematic update moves only the selected articulations' links; the other
  articulation's links stay bit-identical, on CPU and DirectGPU (AC-5).
- The Python `prim_paths=` / `pattern=` kwargs of `query` and its waiting adapters route
  to `ovphysx_query_selection`,
  honor `prim_paths`-wins, and reject empty/oversized selections; passing neither is the
  whole-set query. A selection with `scope=ACTIVE` raises `ValueError` for read and
  write, while `scope=ACTIVE, prim_paths=[]` is not a selection and opens the whole-set
  ACTIVE query. `update_articulations_kinematic` takes the same kwargs and raises the same
  `ValueError` for a selection with `scope=ACTIVE` (AC-6).
- Every articulation selection selects the same articulations as the binding, and each
  one-path list naming a link or joint selects exactly its articulation. A link query
  selects only the named links, and nothing for an articulation root or a joint; a joint
  query selects only the named joints, and nothing for a link; and a rigid-body query over
  the whole stage selects only the standalone box (AC-7).

## Implementation notes

Precedence is resolved at the C boundary (`ovphysx_query_selection` hands the runtime the
native handle list with the pattern suppressed), so the C case asserts it there via
`total_prim_count` rather than resolving any string; it interns one cube's path to an
`ovx_primpath_t` through the attached Stage's dictionary — the native handle the API takes.
The byte-identical parity against a deprecated tensor binding over
the same selection is the Python `test_read_pattern_matches_tensor_binding` /
`test_cpu_read_pattern_matches_tensor_binding` oracle.
