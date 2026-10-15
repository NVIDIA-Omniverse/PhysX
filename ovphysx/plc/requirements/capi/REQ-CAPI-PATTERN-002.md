<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: REQ-CAPI-PATTERN-002
title: Path Pattern Grammar
status: implemented
owner: ovphysx
---

## Description

Every ovphysx entry point that takes a physics-object path pattern — the session query's prim-set
selection (`ovphysx_query_selection`'s `pattern`, the Python `query` / `read_sync` /
`read_tokens_sync` / `write_sync` `pattern=` argument) and the deprecated tensor-binding, SDF-view
and contact-binding entry points —
matches it against prims with the one ovruntime matcher, so a pattern matches the same prims
everywhere. That grammar is not a shell glob: each component is a glob translated to a POSIX
extended regular expression, `**` spans levels, and the last component is searched at any depth.
These are the rules callers most often get wrong (`*` against `**`, a last component that reaches
deeper than expected, `[N]` read as an index placeholder), so this requirement fixes the grammar as
the ovphysx user contract and requires it to be documented once, in full, where every
pattern-taking entry point points.

The behaviour is implemented by the ovruntime matcher
([REQ-PUBLICAPI-001](../../../ovruntime/plc/requirements/api/REQ-PUBLICAPI-001.md) AC-32, AC-33,
AC-34). The last-component rule (AC-4) is that matcher's default, the runtime setting
`/physics/tensors/recursiveLeafPatternMatch` = `true`; ovphysx does not expose the setting, so the
default is the contract. The component-length limit is owned by
[REQ-CAPI-PATTERN-001](REQ-CAPI-PATTERN-001.md).

## Acceptance Criteria

- AC-1: **Components.** A pattern is absolute: it starts with `/`. `ovphysx_query_selection`
  rejects a relative pattern with `OVPHYSX_API_INVALID_ARGUMENT` (owned by
  [REQ-CAPI-SELECT-001](REQ-CAPI-SELECT-001.md) AC-3); the deprecated entry points log an error and
  select nothing. The pattern is split on `/` into components; a trailing `/` is ignored, and a `/`
  inside a balanced `( ... )` group does not split. Each component is matched against the name of
  one prim, so a component never matches across levels.
- AC-2: **Component syntax.** A component is a glob translated to a POSIX extended regular
  expression that must match the whole name: `*` is any run of characters (including none), `?` is
  one character, and `.` is a literal dot. Character classes (`[0-9]`), alternation (`a|b`,
  `(a|b)`, each alternative matching the whole name) and repetition (`+`, `{m,n}`) pass through.
  `[N]` is a class matching the letter `N`, not an index placeholder. A component that is not a
  valid expression matches nothing, and this is not reported as an error.
- AC-3: **`**`.** `**` as a whole component matches zero or more levels. Inside a longer component
  it is not special.
- AC-4: **Last component.** The last component, when it is not a bare `*` and the pattern contains
  no `**`, is searched at any depth below the objects the earlier components matched; the earlier
  components match one level each. A name nested inside a match of the same name is not selected
  again, even when the outer match is not an object of the query's type. A bare `*` last component
  matches direct children only, and so does any last component of a pattern that contains `**`.
- AC-5: **Literal paths.** A pattern with none of `*`, `?`, `(`, `)`, `|`, `[`, `]`, `{`, `}` or
  whitespace selects exactly the prim at that path when one exists; when none does, the text is
  matched as a pattern under AC-1 to AC-4.
- AC-6: **What is selected.** A pattern selects prims, of which the query or view keeps the objects
  of its type; for the session query, which objects of each type a selected prim selects is owned by
  [REQ-CAPI-SELECT-001](REQ-CAPI-SELECT-001.md) AC-7, and the Path patterns page states it.
  Physics-only objects (replicator clones with no authored prim) are matched by the same rules. A
  pattern that matches no object is an empty selection, not an error.
- AC-7: **Limit.** An alternative that contains a `/`, such as `arm/link_0` in
  `(base_link|arm/link_0)`, never matches, because each component is matched against a single name;
  the other alternatives still match.
- AC-8: **Documentation.** The grammar above is documented once, with worked examples, on the
  "Path patterns" page (`docs/read_write/path_patterns.md`), and the documentation of every
  pattern-taking entry point points to that page: `ovphysx_query_selection` and
  `ovphysx_query_selection_t.pattern`; the deprecated tensor-binding pattern-matching block and
  `ovphysx_tensor_binding_desc_t.pattern`; and the Python `query`, `read_sync`,
  `read_tokens_sync`, `write_sync`, `update_articulations_kinematic`, `create_tensor_binding`,
  `create_sdf_view` and
  `create_contact_binding` docstrings. No example
  presents `[N]` as an index placeholder.

## Test References

- [TEST-CAPI-PATTERN-002](../../tests/capi/TEST-CAPI-PATTERN-002.md)
- [TEST-CAPI-SELECT-001](../../tests/capi/TEST-CAPI-SELECT-001.md) — the relative-pattern rejection
  of AC-1

## Code References

- ovphysx/docs/read_write/path_patterns.md (the canonical page — AC-8)
- ovphysx/docs/read_write/index.md (the prim-set selection subsection that links the page — AC-8)
- ovphysx/include/ovphysx/ovphysx.h (`ovphysx_query_selection` doc — AC-8)
- ovphysx/include/ovphysx/ovphysx_types.h (`ovphysx_query_selection_t.pattern` — AC-8)
- ovphysx/include/ovphysx/ovphysx_legacy.h (the tensor-binding pattern-matching block — AC-8)
- ovphysx/include/ovphysx/ovphysx_legacy_types.h (`ovphysx_tensor_binding_desc_t` — AC-8)
- ovphysx/python/ovphysx/api.py (the pattern docstrings — AC-8)
- ovphysx/src/ovphysx/ovphysxPhysXInterop.cpp (`ovphysx_query_selection`'s relative-pattern
  rejection — AC-1)
- ovphysx/tests/data/path_patterns.usda (the documented example hierarchy)
- ovphysx/tests/python_tests/cpu_tests/test_path_patterns.py
- ovphysx/ovruntime/source/omni.physx/plugins/tensors/base/PathPatternMatcher.{h,cpp} (the matcher —
  AC-1 to AC-5 and AC-7, owned by REQ-PUBLICAPI-001)
- ovphysx/ovruntime/source/omni.physx/plugins/tensors/base/BaseSimulationView.cpp
  (`isRecursiveLeafPatternMatchEnabled`, and the internal-registry pass that matches physics-only
  objects — AC-4, AC-6)
- ovphysx/ovruntime/source/omni.physx/plugins/tensors/GlobalsAreBad.h
  (`kDefaultRecursiveLeafPatternMatch` — AC-4)

## Dependencies

- [REQ-PUBLICAPI-001](../../../ovruntime/plc/requirements/api/REQ-PUBLICAPI-001.md) AC-32, AC-33,
  AC-34 — the matcher that implements this grammar
- [REQ-CAPI-PATTERN-001](REQ-CAPI-PATTERN-001.md) — the component-length limit
- [REQ-CAPI-SELECT-001](REQ-CAPI-SELECT-001.md) — the prim-set selection that accepts a pattern
