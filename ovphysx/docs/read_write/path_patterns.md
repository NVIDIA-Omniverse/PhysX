<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Path patterns

A path pattern selects physics objects by prim path. It is accepted by the prim-set selection of
the session read/write API (`ovphysx_query_selection` and its `pattern` field in C, the `pattern=`
argument of `PhysX.query` / `read_sync` / `read_tokens_sync` / `write_sync` / `evaluate_sdf_shapes`
in Python) and by the deprecated tensor-binding, SDF-view and contact-binding entry points. All of
them match a pattern against prims with the same matcher, so a pattern matches the same prims
everywhere; which objects each entry point then keeps is described in
[What a pattern selects](#what-a-pattern-selects).

A pattern is matched **one path level at a time**, and its last component follows a rule a shell
glob does not have. Read [The last component](#the-last-component) before relying on `*`, `**`,
or a named last component.

## Summary

| Syntax | Meaning |
|---|---|
| `/` | Starts the pattern, which is always absolute, and separates components. Each component matches one prim **name**, at one level. |
| `*` | Any run of characters within one name, including none. Never crosses a `/`. |
| `?` | Exactly one character. |
| `[0-9]`, `[abc]`, `[1-2][1-5]` | One character from the class. |
| `a\|b`, `(a\|b)` | Either alternative, each matched as a whole name. |
| `**` | Zero or more levels, when it is a whole component. |
| `.` | A literal dot. |
| last component | Searched at **any depth** below what the earlier components matched, unless it is a bare `*` or the pattern contains `**`. |

## Components

A pattern must be **absolute**: it starts with `/`. The session read/write API rejects a relative
pattern with an argument error; the deprecated entry points log an error and select nothing. A
trailing `/` is ignored, so `/World/robot/` means `/World/robot`.

The pattern is split on `/` into components. Each component is matched against the **name** of a
single prim — the text between two slashes of its path — so a component never matches across
levels. A `/` inside a balanced `( ... )` group does not split the pattern; see [Limits](#limits).

## Component syntax

Each component is a glob that is translated to a POSIX extended regular expression, which must
match the **whole** name:

- `*` means "any run of characters" and `?` means "any one character".
- `.` is always a literal dot. To match any single character, use `?`.
- The rest of the POSIX extended syntax passes through unchanged:
  - Character classes: `env[0-9]` matches `env0` to `env9`; `env[1-2][1-5]` matches `env11` to
    `env25`.
  - Alternation: `base_link|link_0` or `(base_link|link_0)` matches either name. Each alternative
    must match the whole name, not a prefix or a suffix of it.
  - Repetition: `+` and `{m,n}` apply to the item before them.
- `[N]` is **not** an index placeholder. It is a class that matches the letter `N`. For "any
  index", use `[0-9]` for one digit or `*` for any run.
- A component that is not a valid regular expression, such as one with an unclosed `[`, matches
  nothing. This is **not** reported as an error: a typo yields an empty selection.
- A component may be at most 4096 characters long. A longer one is rejected with an argument
  error.

## `**` — any depth

`**` as a **whole** component matches zero or more levels. `/envs/*/Robot/**` selects `Robot` and
everything below it. `/envs/*/**/base_link` selects every `base_link` below an env, at any depth,
including one directly below it. Inside a longer component (`a**b`), `**` is not special: it is
two `*`.

## The last component

The last component is where a pattern differs from a shell glob:

- **A named or glob last component is searched at any depth** below the objects the earlier
  components matched. `/envs/*/Robot/link_0` finds a `link_0` directly below `Robot` **and** one
  nested deeper, such as `/envs/env0/Robot/arm/link_0`.
- **The earlier components still match exactly one level each.** `/envs/*/Robot/base_link` does
  **not** find `/envs/env0/Group/Robot/base_link`, because `Robot` must be a direct child of an env.
  Spell the extra depth with `**`: `/envs/*/**/Robot/base_link`.
- **A bare `*` last component matches direct children only.** `/envs/*/Robot/*` selects the
  children of `Robot`, not everything below it. Use `**` for the whole subtree.
- **A name nested inside a match of the same name is not selected again.** With a `base_link`
  below another `base_link`, `/envs/*/Robot/base_link` returns only the shallower one, while
  `/envs/*/Robot/**/base_link` returns both. This holds even when the outer match is not an object
  of the query's type: a rigid body `base_link` below a plain transform `base_link` is hidden too.
- **When the pattern contains `**`**, the last component matches direct children, like every
  other component.

## Literal paths

A pattern with none of `*`, `?`, `(`, `)`, `|`, `[`, `]`, `{`, `}` or whitespace is first looked
up as an exact path. When a prim exists there, the pattern selects exactly that prim. When none
does, the text is matched as a pattern, so by the last-component rule it can select a same-named
object deeper down: `/envs/env0/link_0` selects `/envs/env0/Robot/arm/link_0` when there is no
prim at `/envs/env0/link_0`. To select objects by identity with no such fallback, use the
explicit `prim_paths` list of native prim handles instead of a pattern.

## What a pattern selects

A pattern selects prims; the query then keeps those that are objects of its type, so
`/envs/*/Robot/*` over `RIGID_BODY` returns only the children of `Robot` that are rigid bodies.

- `RIGID_BODY` covers standalone rigid bodies. Articulation links are `ARTICULATION_LINK`
  objects, so a pattern over `RIGID_BODY` never returns a link.
- `RIGID_BODY` also covers point-instancer instances, which are selected through the instancer's
  path: `/World/instA` selects all of `instA`'s instances, as one array row. A pattern that matches
  only the prototype, such as `/World/instA/proto` or `/World/instA/*`, selects nothing.
- An `ARTICULATION` query also keeps an articulation when the pattern matches any of its links
  or joints, as the articulation tensor bindings do. `/envs/*/Robot/base_link` selects the
  articulation `base_link` belongs to, whether the articulation root is `Robot` or `base_link`.
- Link, joint and tendon queries keep only the members whose own prim is matched. To read every
  link of an articulation, match the links, for example with `/envs/*/Robot/**`.
- The deprecated rigid-body tensor binding differs: it also covers articulation links, and a
  matched articulation root selects its root link. For a point instancer it selects nothing through
  the instancer's path, and a pattern matching the prototype returns a single instance, under the
  prototype's path. When migrating, select an instancer by its own path.
- An `SDF_SHAPE` query keeps the matched prims that are SDF collision shapes, by the SDF view's rule,
  and warns about the others.

Physics-only objects, such as replicator clones with no authored prim, are matched by the same
rules. A pattern that matches no object is not an error: the selection is empty.

Rows come in engine order, which is neither path order nor the tensor-binding order. Identify each
row through its group's prim list (row *i* is prim *i*), not by position. SDF shape evaluation
returns no prim list, so its rows follow the pattern's match order instead, as the SDF view's did.

## Limits

- An alternative that contains a `/`, as `arm/link_0` in `(base_link|arm/link_0)`, never matches,
  because every component is matched against a single name. The other alternatives still match.
  Use `**` or an explicit `prim_paths` list instead.

## Examples

For this hierarchy, where every leaf is a rigid body and the other prims are transforms:

```text
/envs/env0/Robot/base_link
/envs/env0/Robot/arm/link_0
/envs/env0/Group/Robot/base_link
/envs/env1/Robot/base_link
/envs/env1/Robot/arm/link_0
/envs/envA/Robot/base_link
```

| Pattern | Selects |
|---|---|
| `/envs/*/Robot/base_link` | `/envs/env0/Robot/base_link`, `/envs/env1/Robot/base_link`, `/envs/envA/Robot/base_link` — not `/envs/env0/Group/Robot/base_link` |
| `/envs/*/**/Robot/base_link` | The three above, and `/envs/env0/Group/Robot/base_link` |
| `/envs/*/Robot/link_0` | `/envs/env0/Robot/arm/link_0`, `/envs/env1/Robot/arm/link_0` |
| `/envs/*/Robot/*` | `/envs/env0/Robot/base_link`, `/envs/env1/Robot/base_link`, `/envs/envA/Robot/base_link` — `arm` is not a body, and `arm/link_0` is two levels down |
| `/envs/*/Robot/**` | `/envs/env0/Robot/base_link`, `/envs/env0/Robot/arm/link_0`, `/envs/env1/Robot/base_link`, `/envs/env1/Robot/arm/link_0`, `/envs/envA/Robot/base_link` |
| `/envs/env[0-9]/Robot/base_link` | `/envs/env0/Robot/base_link`, `/envs/env1/Robot/base_link` |
| `/envs/env?/Robot/base_link` | `/envs/env0/Robot/base_link`, `/envs/env1/Robot/base_link`, `/envs/envA/Robot/base_link` |
| `/envs/env0/Robot/base_link\|link_0` | `/envs/env0/Robot/base_link`, `/envs/env0/Robot/arm/link_0` |
| `/envs/env0/Robot/base\|link_0` | `/envs/env0/Robot/arm/link_0` — `base` does not select `base_link`: an alternative must match the whole name |
| `/envs/env.*/Robot/base_link` | Nothing: `.` is a literal dot, so `env.*` matches only names that start with `env.` |
| `/envs/env0/link_0` | `/envs/env0/Robot/arm/link_0` — there is no object at that exact path, so it is matched as a pattern |
| `/envs/env[0/Robot/base_link` | Nothing: `env[0` is not a valid expression |
| `/envs/env0/Robot/(base_link\|arm/link_0)` | `/envs/env0/Robot/base_link` — the alternative containing a `/` never matches |
