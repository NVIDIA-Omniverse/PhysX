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

## Key Concepts

**Two ways to replicate.** ovphysx offers two cloning paths:

- **Direct ovphysx API** (`PhysX.clone()` / `ovphysx_clone()`) — the approach used in this tutorial. After an ovstage-backed scene is attached and drained, `clone()` copies a source subtree in the internal physics representation. No USD prims are created, so it stays fast and memory-light at large environment counts.
- **Up-front ovstage duplication** (`ovstage_clone`) — applications that own the ovstage `Stage` can duplicate the source subtree on the stage before attaching it. This keeps every scene edit on the producer-owned stream and avoids the warmup ordering constraint described in [Warmup and Determinism](../developer_guide.md#warmup-and-determinism).

Refer to [Scene Cloning](../developer_guide.md#scene-cloning) for the full comparison.

Tensor-binding path patterns include direct ovphysx runtime clones even when an
intermediate target path has no authored USD prim. For example, after cloning
`/World/envs/env0/robot` to `/World/envs/env1/robot`, the pattern
`/World/envs/env*/robot` resolves both objects.

**Clone before warmup.** All `clone()` calls must happen **before** warmup and before the first `step()`. Cloning after `warmup()` or the first step reallocates physics structures and would corrupt already-initialized solver state, so the runtime rejects it with `OVPHYSX_API_INVALID_ARGUMENT` (surfaced in Python as `RuntimeError`). If you must clone later, call `reset_stage()`, wait for it to complete, then reload the source scene or reattach its ovstage before cloning again. Refer to [Warmup and Determinism](../developer_guide.md#warmup-and-determinism).

**Grouping copies with `env_ids`.** A single `clone()` call numbers its copies automatically. One logical environment is sometimes assembled from **several** `clone()` calls — for example, first every environment's robot, then every environment's object. In that case, pass the optional per-target `env_ids` with the same ids in every call. Copies sharing an id land in the same runtime environment and can collide, and without `env_ids`, objects cloned by different calls never share an environment.

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
