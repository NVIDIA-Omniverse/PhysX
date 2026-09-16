<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

---
id: TEST-CAPI-PATTERN-001
maps_to: REQ-CAPI-PATTERN-001
type: integration
---

## Scenario

An oversized path pattern component is rejected at the public boundary with an
argument error on every pattern-taking entry point, a component at exactly the
limit still works, and the runtime backstop never matches instead of crashing.

## Given

- The CPU-mode `cpu_tests` PhysX session with `boxes_falling_on_groundplane.usda`
  populated through ovstage.
- Patterns whose last component is 65536 `a` characters or 65536 `?`
  characters, the literal one also as a `prim_paths` entry and as a contact
  sensor or filter pattern, a pattern whose last component is exactly 4096
  characters, a parenthesized group whose two `/`-separated halves are each
  under the limit but total more than 4096 characters, and a short group with a
  `/` inside it.

## When

- `create_tensor_binding(pattern=...)`, `create_tensor_binding(prim_paths=[...])`,
  `create_sdf_view(pattern=...)`, `create_contact_binding([sensor])` and
  `create_contact_binding([sensor], [filter])` are called with the oversized
  input.
- `create_tensor_binding(pattern=...)` is called with the at-limit component,
  with the oversized parenthesized group, and with the short group containing a
  `/`.

## Then

- Every oversized call raises `RuntimeError` whose message names the
  4096-character limit, and no binding or view is created (REQ AC-1, AC-2).
- The at-limit pattern is accepted and yields an empty binding; the oversized
  group is rejected although no `/`-split piece exceeds the limit, and the
  short group with a `/` inside still binds its named body (REQ AC-3).
- The runtime backstop is proven by ovruntime's `TestTensorSimulationView.cpp`
  glob token cases, TEST-PUBLICAPI-001 increment 14 (REQ AC-4).

## Known gap

- The C entry points are exercised through the Python frontend only; there is
  no `c_unittests` case for the rejection.
