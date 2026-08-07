# Cloning: Replicate Environments

This tutorial shows how to use the clone API to replicate sub-sections of a USD scene. Cloning creates copies in the internal physics representation (not USD prims), optimized for large-scale parallel simulation.

## Prerequisites

- Complete the [Hello World](hello_world.md) tutorial.
- Use a USD scene with a hierarchy suitable for cloning (for example `/World/envs/env0`).

## Key Concepts

**Two ways to replicate.** ovphysx offers two cloning paths:

- **Direct ovphysx API** (`PhysX.clone()` / `ovphysx_clone()`) — the approach used in this tutorial and the supported 0.5 path for tensor-based multi-environment workloads. After an ovstage-backed scene is populated and attached, `clone()` copies a source subtree in the internal physics representation. No USD prims are created, so it stays fast and memory-light at large environment counts.
- **ovstage duplication** (`ovstage.Stage.clone()` / `ovstage_clone()`) — apps that own the ovstage `Stage` can duplicate the source subtree and drain the clone delta with `update_from_ovstage()`. This creates the physics objects, but in ovphysx 0.5 TensorBindingsAPI does not discover objects created by the clone delta. Complete TensorBindingsAPI support for this path is scheduled for ovstage after 0.5.

Refer to [Scene Cloning](../developer_guide.md#scene-cloning) for the full comparison.

**Tensor binding limitation in 0.5.** For clones that need tensor bindings, use
the direct `clone()` API and complete cloning before `warmup_gpu()` or the first
simulation step. Direct cloning invalidates existing tensor and contact
bindings; destroy and recreate them before use. Do not also drain a duplicate
ovstage clone delta for the same target paths.

**Clone before warmup.** All `clone()` calls must happen **before** GPU warmup and before the first `step()`. Cloning after `warmup_gpu()` or the first step reallocates DirectGPU buffers and would corrupt already-initialized solver state, so the runtime rejects it with `OVPHYSX_API_INVALID_ARGUMENT` (surfaced in Python as `RuntimeError`). If you must clone later, call `reset_stage()` first. Refer to [GPU Warmup and Determinism](../developer_guide.md#gpu-warmup-and-determinism).

**Grouping copies with `env_ids`.** A single `clone()` call numbers its copies automatically. When one logical environment is assembled from **several** `clone()` calls (for example, first every environment's robot, then every environment's object), pass the optional per-target `env_ids` with the same ids in every call, so copies sharing an id land in the same runtime environment and can collide. Without `env_ids`, objects cloned by different calls never share an environment.

## Code Language

### Python

```{literalinclude} ../../tests/python_samples/clone.py
:language: python
```

### C

```{literalinclude} ../../tests/c_samples/clone_c/main.c
:language: c
```

## Result

After this tutorial, you can replicate environments through the clone API and simulate all copies together.
