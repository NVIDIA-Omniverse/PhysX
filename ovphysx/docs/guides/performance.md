<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# Simulation Performance Guide

Best practices for optimizing simulation performance with ovphysx. Most tuning is
either a per-scene USD attribute on the codeless `PhysxSceneAPI` (refer to
[Physics Scene](../simulation_setup/physics_scene.md) and
[Physics Schemas](../physics_schemas.md)) or a `PhysXConfig` option on the
instance.

## Solver Iterations

Lowering solver iterations to the smallest count that still gives acceptable
fidelity helps a lot — clamp the per-scene iteration counts. PGS is typically
faster per iteration; TGS often reaches better stability with fewer iterations.
For the attributes and how to author them, refer to
[Physics Solver](../simulation_setup/physics_scene.md#physics-solver).

## CPU vs GPU by Scene Size

For only a few rigid bodies and articulations, CPU simulation is often faster; for
large scenes (for example RL with many environments), use GPU simulation and
access state through the batch [tensor API](../tutorials/tensor_bindings.md). How
to select CPU vs GPU (per-scene `physxScene:enableGPUDynamics`, process-wide
`set_cpu_mode`, and GPU selection) is covered in
[CPU vs GPU Simulation](../simulation_setup/physics_scene.md#cpu-vs-gpu-simulation).

## Reading State Efficiently

Unlike Kit-based Omni PhysX, ovphysx does not write simulation output back to
USD, so there is no per-step USD write-back cost. Read state in bulk through
[tensor bindings](../tutorials/tensor_bindings.md) (DLPack, zero-copy on device)
or the ovstage [output read](../ovstage_integration.md) API rather than per-prim.

## Physics Thread Count

Adjust the number of CPU simulation threads. For small scenes, single-threaded
(`num_threads=0`) often gives the best performance:

```python
from ovphysx import PhysX, PhysXConfig
physx = PhysX(config=PhysXConfig(num_threads=0))
```

## Stepping Cadence

ovphysx steps explicitly through `step()` / `step_sync()`, so you control how often
physics runs relative to any rendering or control loop — there is no Kit-style
asynchronous scene-update mode to configure. Keep any per-step Python work light,
since it runs inline with your loop and can become the bottleneck.

## Avoid Unintended Collision Overlap in RL Scenes

When building many parallel environments, do **not** duplicate infinite collision
geometry such as ground planes. A ground plane is infinite, so each duplicate
touches every environment, producing large numbers of expensive inter-environment
overlaps. Author one shared ground, or use finite ground colliders positioned per
environment. [OmniPVD recording](../tutorials/omnipvd_recording.md) is useful to
inspect the scene and confirm there are no unexpected overlaps.

## Disable Scene-Query Support

If you do not use scene queries (raycast/sweep/overlap), disable support to save
performance:

```usda
def PhysicsScene "physicsScene" (
    prepend apiSchemas = ["PhysxSceneAPI"]
)
{
    bool physxScene:enableSceneQuerySupport = false
}
```

## Multi-GPU

On multi-GPU machines, pass a supported multi-GPU pattern through
`active_cuda_gpus` for a potential speedup. Alternatively, when
`active_cuda_gpus` is empty, use `PhysXConfig(scene_multi_gpu_mode=...)`. Refer to
[CPU vs GPU Simulation](../simulation_setup/physics_scene.md#cpu-vs-gpu-simulation).

## Collision Geometry Choice

Simpler collision geometry is faster: an SDF mesh collider is far more expensive
than a sphere. Prefer primitives, then convex hulls, before triangle/SDF meshes
(refer to [Colliders](../simulation_setup/collision.md)).

In GPU RL workloads, watch for warnings that a `convexHull` mesh failed to cook a
GPU-compatible hull (common for long, thin, high-aspect-ratio inputs). Such a
mesh falls back to CPU contact generation, which can badly hurt performance and
prevents interaction with GPU-only features. Workarounds: use a bounding-cube
approximation, or a static triangle mesh if the geometry is not on a dynamic body.

## Deformables and Particles

Deformables and particles are considerably more expensive than rigid bodies.
Where feasible, approximate a deformable with rigid bodies plus compliant
contacts, or rigid bodies connected by joints.
