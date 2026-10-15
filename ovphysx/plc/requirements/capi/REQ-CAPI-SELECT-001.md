<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-SELECT-001
title: Prim-set Selection on the Session Query
status: implemented
owner: ovphysx
---

## Description

`ovphysx_query` selects every object of a type within a scope, with no way to name a
subset of prims — the gap that blocks removing the deprecated tensor-binding read/write
([REQ-CAPI-WRITE-001](REQ-CAPI-WRITE-001.md) AC-14(a)). Add
`ovphysx_query_selection`, which carries a required **selection descriptor** — an
explicit list of native `ovx_primpath_t` handles OR a glob pattern. Only the
descriptor *shape* and precedence follow the deprecated `ovphysx_tensor_binding_desc_t`;
the explicit list is native handles, not the binding's path strings — the caller has
already resolved paths to tokens (per
[ADR-0025](../../../ovruntime/plc/adr/ADR-0025-query-prim-set-selection.md)).

C has no function overloading, so the selection is a **new sibling entry point**, not a
selector parameter on `ovphysx_query`: `ovphysx_query` stays the whole set within scope.
Both query forms take the same optional creation result
([REQ-CAPI-SESSION-001](REQ-CAPI-SESSION-001.md)). This REQ owns
the public boundary — the entry point, the descriptor, precedence, validation, and the
Python mirror. The runtime enumeration filter it feeds (resolve, ∩ type, no scope,
cache isolation, write inheritance) is
[REQ-READ-SELECT-001](../../../ovruntime/plc/requirements/output/REQ-READ-SELECT-001.md).

## Acceptance Criteria

- AC-1: **Sibling entry point.** `ovphysx_query_selection(handle, object_type, selection,
  out_query, out_summary)` takes a **required, non-NULL** `const ovphysx_query_selection_t*`
  — a `pattern` glob string, a `prim_paths` list of native `ovx_primpath_t` handles, and
  `prim_paths_count` — and the optional creation result both query forms share. It takes
  **no scope**: a selected query covers every selected object of the type, sleeping ones
  included (REQ-READ-SELECT-001 AC-6); `ovphysx_query` with `OVPHYSX_SCOPE_ACTIVE` stays the
  way to read only the objects that moved. A NULL `selection` returns
  `OVPHYSX_API_INVALID_ARGUMENT` — the whole set is the plain `ovphysx_query(handle,
  object_type, scope, out_query, out_summary)`, not a NULL overload.

- AC-2: **Precedence.** When both are supplied, `prim_paths` wins and `pattern` is
  ignored; they are never combined — the same rule as `ovphysx_tensor_binding_desc_t`.
  A descriptor with neither a `pattern` nor any `prim_paths` entry returns
  `OVPHYSX_API_INVALID_ARGUMENT`.

- AC-3: **Validation by kind.** A `pattern` with an oversized path component or an
  embedded NUL is rejected with `OVPHYSX_API_INVALID_ARGUMENT` and a message naming the
  limit, via the shared `hasOversizedPathComponent` / `hasEmbeddedNul` checks — the same
  rejection REQ-CAPI-PATTERN-001 applies to tensor bindings. A `pattern` that does not
  start with `/` is rejected with `OVPHYSX_API_INVALID_ARGUMENT`: every pattern-taking
  entry point requires an absolute pattern (REQ-CAPI-PATTERN-002 AC-1). The `prim_paths` handles are
  validated as handles instead: a `prim_paths` array that is NULL with a non-zero count,
  or that contains `OVX_INVALID_PRIMPATH`, is rejected with `OVPHYSX_API_INVALID_ARGUMENT`.
  Handles are passed
  through by pointer — nothing is copied or string-checked, since the caller already
  resolved the paths. Every one of these rejections, and a NULL `selection` (AC-1), is
  returned on the receipt: no task is created and `out_query` stays 0.

- AC-4: **Valid empty match.** A syntactically valid selection that resolves to no
  object of the queried type yields a valid empty query: its optional creation result
  reports `total_prim_count == 0`, and the first `ovphysx_read` fetch completes
  successfully with a NULL group, not an error. The membership is resolved once at
  creation and is not re-resolved by later sessions. (Resolution and `∩ type` are owned
  by REQ-READ-SELECT-001.)

- AC-5: **Read and write narrow together.** A read over a selected query returns only
  the selected rows; a write opened over the *same* selected query scatters
  only those rows and leaves every unselected superset row bit-identical after commit +
  step. Narrowing the query narrows both directions with no separate write-side surface
  (backed by REQ-READ-SELECT-001 AC-4). `ovphysx_update_articulations_kinematic` over a
  selected articulation query likewise refreshes only the selected articulations and leaves
  every other articulation unchanged (REQ-TENSOR-FK-001 AC-1, AC-4).

- AC-6: **Python mirror.** `PhysX.query`, the waiting adapters `PhysX.read_sync`,
  `PhysX.read_tokens_sync` and `PhysX.write_sync`, and
  `PhysX.update_articulations_kinematic` accept keyword-only `prim_paths=None` (a list of
  native `ovx_primpath_t` handles, e.g. from a read group's `prim_list` or
  `ovstage.PathDictionary.intern_path`) and `pattern=None`,
  marshalled into `ovphysx_query_selection_t` and routed to `ovphysx_query_selection`;
  `prim_paths`-wins precedence and the pattern's empty / oversized-component rejection
  mirror the C behavior. Passing neither is the whole-set query and routes to
  `ovphysx_query`, honouring `scope`. Because the C selection entry has no scope, a
  selection combined with `scope=ACTIVE` raises `ValueError` rather than being widened to
  `ALL`; an empty `prim_paths` list with no `pattern` is not a selection and keeps its
  scope.

- AC-7: **What a selection selects, per type.** A selection names prims, and a query keeps
  the objects of its type whose prim is named. An `OVPHYSX_QUERY_TARGET_ARTICULATION` query also
  selects an articulation when the selection names one of its links or joints, so it selects
  the same articulations a deprecated articulation tensor binding does for the same pattern
  or paths. Every other type selects only the objects whose own prim is named: an
  `OVPHYSX_QUERY_TARGET_RIGID_BODY` query does not cover articulation links, which are
  `OVPHYSX_QUERY_TARGET_ARTICULATION_LINK` objects, and a link, joint or tendon query is not
  widened to the rest of an articulation. (Backed by REQ-READ-SELECT-001 AC-3 and AC-7.)

## Test References

- TEST-CAPI-SELECT-001 — implemented. The C precedence case
  (`QuerySelectionPrimPathsWinsOverPattern` in `tests/c_unittests/test_output_read.cpp`)
  proves `prim_paths` wins over `pattern` at the C boundary via `total_prim_count`, and
  `QuerySelectionRejectsInvalidSelectionsOnTheReceipt` proves every malformed selection is
  rejected on the receipt, with no task and no query (AC-1, AC-3). The
  Python cases (`test_output_read.py`: subset, precedence, empty-match, the
  `scope=ACTIVE` rejection, write;
  `cpu_tests/test_output_read_cpu.py`: the CPU mirror) cover the kwargs and the
  legacy-binding parity; `cpu_tests/test_selection_object_types.py` covers AC-7 against a fixed
  table of the expected articulations and an articulation tensor binding. The selected kinematic update of AC-5 is covered by
  `SelectedKinematicUpdateCpuTest` in `tests/c_unittests/test_articulation_kinematic_isolation.cpp`
  (CPU) and `test_articulation_kinematic_selection_gpu.py` (DirectGPU, through the Python
  kwargs of AC-6); `cpu_tests/test_articulation_kinematic_selection.py` checks the kwargs'
  rejections.

## Code References

- ovphysx/include/ovphysx/ovphysx_types.h (`ovphysx_query_selection_t`, `prim_paths` a
  `const ovx_primpath_t*` handle list — AC-1, AC-2)
- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_query_selection` declaration + doc; the
  whole-set `ovphysx_query` and the optional creation result both share — AC-1; the selected kinematic update in the
  `ovphysx_update_articulations_kinematic` doc — AC-5; which objects each type keeps — AC-7)
- ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp (`ovphysx_query_selection` impl: precedence,
  the pattern's string validation, the `prim_paths` NULL-with-count and `OVX_INVALID_PRIMPATH`
  checks, the relative-pattern rejection, and handle-list passthrough across the sidecar
  boundary — AC-1, AC-2, AC-3)
- ovphysx/src/include/internal/sdk/ovphysxSDK.hpp (`hasOversizedPathComponent`,
  `hasEmbeddedNul`, `oversizedPathComponentMessage` — the shared pattern validation — AC-3)
- ovphysx/src/ovphysxInternal/ovphysxInternalInterop.cpp + sidecar loader (the
  `_output_query_selected` export/accessor binding `ovxQuerySelected`, now
  `(attachHandle, type, primPaths, primPathCount, pattern, outSummary)` — AC-1)
- ovphysx/python/ovphysx/api.py + _bindings.py (`query`, `read_sync`, `read_tokens_sync`,
  `write_sync` and `update_articulations_kinematic` `prim_paths=` / `pattern=` kwargs, and
  the selection-with-`ACTIVE` `ValueError` in `PhysX.query` — AC-6)
- ovphysx/ovruntime/source/omni.physx/plugins/OvxPhysicsRead.cpp (`ovxQuerySelected` adds
  the articulation of each selected link or joint to an articulation query's selection — AC-7)
- ovphysx/ovruntime/source/omni.physx/plugins/tensors/base/BaseSimulationView.cpp
  (`resolveOwningArticulation`, shared with the articulation tensor binding — AC-7)
- ovphysx/docs/read_write/index.md and ovphysx/docs/read_write/path_patterns.md (the
  selection's per-type rule — AC-7)
- ovphysx/docs/changelog.md (the new selection surface)

## Dependencies

- [ADR-0025](../../../ovruntime/plc/adr/ADR-0025-query-prim-set-selection.md) (the decision this implements)
- [REQ-READ-SELECT-001](../../../ovruntime/plc/requirements/output/REQ-READ-SELECT-001.md) (the runtime enumeration filter this surface feeds)
- [REQ-CAPI-PATTERN-001](REQ-CAPI-PATTERN-001.md) (the shared path-component validation reused here)
- [REQ-CAPI-PATTERN-002](REQ-CAPI-PATTERN-002.md) (the grammar of the `pattern` glob and its documentation)
- [REQ-CAPI-WRITE-001](REQ-CAPI-WRITE-001.md) (the tensor-binding deprecation whose gap (a) this closes)
