<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-PATTERN-002
maps_to: REQ-CAPI-PATTERN-002
type: integration
---

## Coverage status

Implemented in `tests/python_tests/cpu_tests/test_path_patterns.py` over
`tests/data/path_patterns.usda`, the example hierarchy of the Path patterns page plus two
same-name nestings:

- `test_documented_patterns_select_documented_bodies` resolves every row of the page's examples
  table, and the further rules of AC-1 to AC-5 and AC-7, through the session read's `pattern=`
  selection and through a deprecated rigid-body tensor binding, and requires both to select exactly
  the listed bodies.
- `test_relative_pattern_is_refused` covers the absolute-path rule of AC-1 on both routes.
- `test_physics_only_clones_follow_the_same_rules` covers the replicator clones of AC-6.

AC-8 is verified by the ovphysx documentation build, run with warnings as errors so that a
broken link fails it, and by inspection of the listed docstrings.

## Scenario

A path pattern selects the objects the Path patterns page says it selects, through the prim-set
selection and the deprecated tensor binding alike, and every pattern-taking entry point's
documentation points to that page (REQ-CAPI-PATTERN-002 AC-1..AC-8).

## Given

- A stage with rigid bodies at `/envs/env0/Robot/base_link`, `/envs/env0/Robot/arm/link_0`,
  `/envs/env0/Group/Robot/base_link`, `/envs/env1/Robot/base_link`, `/envs/env1/Robot/arm/link_0`
  and `/envs/envA/Robot/base_link`; a `base_link` body nested inside a `base_link` body; and a
  `base_link` body below a `base_link` transform. The other prims are transforms. The stage is
  attached through ovstage on CPU and stepped once.

## When

- Select rigid bodies with each pattern of the examples table, and with patterns exercising a
  trailing `/`, repetition (`env[0-9]+`), a parenthesized alternation in an earlier component, `**`
  inside a longer component, `**` over zero levels, a literal path to an existing prim, and
  same-name nesting under `*` and under `*/**`, through `PhysX.read_sync(pattern=...)` and through
  `create_tensor_binding(pattern=...)`.
- Select with a relative pattern through both routes.
- On a fresh attach, clone `/envs/envA` to `/envs/envB` before the first step, then select
  `/envs/env[AB]/Robot/base_link` through both routes.

## Then

- Each pattern selects exactly its listed bodies on both routes: one level per component (AC-1);
  the glob, class, alternation and repetition syntax, `.` as a literal, and an invalid expression
  selecting nothing without an error (AC-2); `**` over zero or more levels and not special inside a
  longer component (AC-3); the last-component rule with its bare-`*`, `**` and same-name exceptions,
  including an outer transform hiding a nested body (AC-4); a literal path selecting its prim and
  falling back to a pattern when no prim is there (AC-5); an empty match as an empty selection
  (AC-6); an alternative containing a `/` never matching while the others do (AC-7).
- The relative pattern raises from the read and yields an empty tensor binding (AC-1).
- The clone is selected alongside its source by the same rules (AC-6).
- The documentation build succeeds with the page linked from the read/write overview and the
  tensor-bindings tutorial (AC-8).
