<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Cloning -- Replicate Environments

This tutorial shows how to use the clone API to replicate sub-sections of a USD scene. Cloning creates copies in the internal physics representation (not USD prims), optimized for large-scale parallel simulation.

## Prerequisites

- Complete the [Hello World](hello_world.md) tutorial.
- A USD scene with a hierarchy suitable for cloning (for example `/World/envs/env0`).
  This tutorial uses `basic_simulation.usda`, which ships with every package under
  `ovphysx/samples/data/` in the wheel, `<sdk-root>/samples/data/` in the C/C++ SDK,
  and `tests/data/` in a repository checkout.
  The source can be any prim subtree that carries physics: an `Xform` environment root
  such as `/World/envs/env0`, or a single rigid-body prim with no wrapper (for example
  a `Cube` that itself carries `PhysicsRigidBodyAPI`). Clones are runtime physics only, so
  no USD prim has to exist at the target paths.

## Key Concepts

**Stage and physics copies are separate.**

- **Direct ovphysx API** (`PhysX.clone()` / `ovphysx_clone()`) — the approach used in this tutorial. After an ovstage-backed scene is attached and drained, `clone()` copies a source subtree in the internal physics representation. No USD prims are created, so it stays fast and memory-light at large environment counts.
- **ovstage copies** (`Stage.clone()` / `ovstage_clone`) duplicate the populated
  stage data for other consumers. When using both APIs, complete the attach
  before cloning the stage. Clone the same source and targets separately in
  ovphysx, before warmup or the first step. Use a dedicated ordinal for the
  stage clone, seal it, and exclude it from every
  `update_from_ovstage()` range. Forwarding that ordinal can ingest the copied
  physics data as new scene objects; it does not invoke the PhysX replicator.

Refer to [Scene Cloning](../developer_guide.md#scene-cloning) for ordinal ordering
and the effect of later structural changes.

Tensor-binding path patterns include direct ovphysx runtime clones even when an
intermediate target path has no authored USD prim. For example, after cloning
`/World/envs/env0/robot` to `/World/envs/env1/robot`, the pattern
`/World/envs/env*/robot` resolves both objects.

**Do not use authored-ancestor deletion to remove runtime clones.** `PhysX.clone()`
and `ovphysx_clone()` copies exist only in the physics runtime. They have no authored
prim. In a sibling-environment layout, cloning `/World/envs/env0` to
`/World/envs/env1` and then deleting `/World/envs` through ovstage removes the
authored source bodies but leaves the runtime clone addressable. This is not a
general guarantee that clones survive ancestor deletion in other layouts.

**Clone before warmup.** Call `clone()` before warmup and before the first
`step()`. A later clone fails because changing the physics layout would corrupt
initialized solver state. Python reports this as `TaskError` when you wait.
To clone later, detach, wait, reload or reattach the source Stage, then clone.
See [Warmup and Determinism](../developer_guide.md#warmup-and-determinism).

**Grouping copies with `env_ids`.** A single `clone()` call numbers its copies automatically. One logical environment is sometimes assembled from **several** `clone()` calls — for example, first every environment's robot, then every environment's object. In that case, pass the optional per-target `env_ids` with the same ids in every call. Copies sharing an id land in the same runtime environment and can collide.

> **Mixing automatic and explicit ids can join environments unintentionally.**
> The automatic numbering base belongs to the attachment; explicit logical ids
> always map to runtime id `id + 1`. With environment-id filtering active on a
> fresh attachment, a one-target automatic call uses runtime id 1. A later call
> with `env_ids=[0]` also uses runtime id 1, so those copies can collide. In the
> reverse order, the explicit call raises the base and the automatic call uses
> runtime id 2. There is no warning or error for this overlap. Choose one ID
> strategy for an attachment; use explicit ids consistently when you need to
> control grouping across calls.

Environment ids provide cross-environment collision isolation only when the
scene uses GPU dynamics and GPU broadphase. They do not isolate CPU clones.
In CPU mode, give every clone a spatially disjoint `anchor_transforms` pose;
co-located clones share one collision space and can push each other apart.

When env ids are requested but the scene runs CPU dynamics or a CPU broadphase,
the runtime logs `EnvIds requested but gpu dynamic is disabled` and `EnvIds
requested but gpu broadphase is not set`. Those records go to the Carbonite log,
not to Python's `warnings` module or `sys.stderr`, so a caller who wants to
observe, assert on, or escalate them has to attach a log consumer:
`ovphysx.enable_python_logging()` routes them to the `ovphysx` Python logger, and
`ovphysx_set_log_callback()` delivers them to a C callback. Refer to
[Logging](../developer_guide.md#logging).

## Code Language

### Python

This complete sample attaches `basic_simulation.usda`, clones `/World/envs/env0`
into three spatially disjoint targets before the first step, runs 10 steps, and
reads back one rigid-body position per environment:

```{literalinclude} ../../tests/python_samples/clone.py
:language: python
```

### C

The C sample performs the same three-target clone, passing `NULL` for `env_ids`
so the call numbers its copies automatically:

```{literalinclude} ../../tests/c_samples/clone_c/main.c
:language: c
```

## Result

After this tutorial, you can replicate environments through the clone API and simulate all copies together.
