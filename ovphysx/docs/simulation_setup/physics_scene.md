<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# Physics Scene and Simulation Configuration

Every stage that ovphysx simulates needs a **physics scene**: a `PhysicsScene`
prim that defines gravity and the PhysX solver configuration. This page explains
how to author the scene and configure global simulation behavior (solver,
iteration counts, GPU buffers, sleeping, stabilization) for scenes that ovphysx
loads and steps.

This is the foundation the other Simulation Setup pages build on
([Collision](collision.md), [Rigid Bodies](rigid_bodies.md),
[Joints](joints.md), [Articulations](articulations.md),
[Deformables](deformables.md), [Particles](particles.md)).

## How ovphysx Simulates a Scene

ovphysx consumes **pre-authored USD** owned by the application through ovstage.
Once a stage is attached, a step proceeds roughly as follows:

1. After population, seal the ordinal with `advance_write_floor()`, then call
   [`attach_ovstage()`](../ovphysx_overview.md). Attach parses the sealed ordinal
   and creates PhysX objects for the prims that carry physics schemas.
2. Colliders that need cooking (convex hull, convex decomposition, SDF, triangle
   mesh) are cooked. Results are cached on disk so recomputation is usually
   avoided — refer to the cooked-collider cache (UJITSO) in the
   [Developer Guide](../developer_guide.md).
3. `step(dt)` advances the simulation by `dt`. **`step()` is asynchronous**: it
   enqueues the step and returns an `op_index` immediately. Subsequent in-stream
   ovphysx calls (such as tensor `read()` / `write()`) automatically wait for it,
   but to consume results **outside** the ovphysx stream you must synchronize —
   `wait_op()` on the returned index (or `wait_all()`). `step_sync(dt)` is a
   convenience that steps and waits in one call (the typical choice for RL and
   control loops). Refer to the [Execution Model](../developer_guide.md#execution-model).
4. Results are read back explicitly by the application — either with
   [tensor bindings](../tutorials/tensor_bindings.md) or with the ovstage
   [output read](../ovstage_integration.md) API. Unlike Omni PhysX in Kit,
   **ovphysx does not write simulation output back to the attached USD stage**;
   the application owns writing state back to ovstage.

The rest of this page is about authoring the `PhysicsScene` prim and the
attributes that configure this pipeline.

## Setting Up a USD Stage and a Physics Scene

To simulate physics, a stage must have at least one `PhysicsScene` prim. The
scene is authored with the core `UsdPhysics` schema (part of stock `usd-core`).
PhysX-specific solver settings live on the codeless `PhysxSceneAPI` schema (refer to
[Physics Schemas](../physics_schemas.md)).

### `.usda`

```usda
#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
    kilogramsPerUnit = 1
)

def Xform "World"
{
    def PhysicsScene "physicsScene"
    {
        vector3f physics:gravityDirection = (0, 0, -1)
        float physics:gravityMagnitude = 9.81
    }
}
```

### Python

```python
import ovphysx
from pxr import Plug, Usd, UsdGeom, UsdPhysics, Gf

# Register the bundled codeless PhysX schemas first: USD builds its schema
# registry on first access and never rebuilds it, so this must precede the
# first stage. Refer to Physics Schemas.
Plug.Registry().RegisterPlugins([str(p) for p in ovphysx.codeless_schema_paths()])

stage = Usd.Stage.CreateNew("scene.usda")

# Units and up axis (see below).
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)

world = UsdGeom.Xform.Define(stage, "/World")

scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
scene.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr().Set(9.81)
```

For a complete, CI-tested minimal scene (physics scene + static ground + dynamic
bodies), refer to the `simple_physics_scene.usda` reference used by
[Hello World](../tutorials/hello_world.md).

## Units and Up Axis

USD supports scaling of units through layer metadata. For correct simulation
results, the units and up axis of your stage must match your content.

- `metersPerUnit` sets the length unit (`1` = meters). Keep all sizes,
  positions, and velocities in that unit.
- `upAxis` is `"Y"` or `"Z"`. Point `physics:gravityDirection` down the same
  axis (up axis `Z` -> gravity `(0, 0, -1)`).
- `kilogramsPerUnit` (set through `UsdPhysics.SetStageKilogramsPerUnit`) sets the
  mass unit.
- If no gravity is authored, ovphysx scales default and auto-computed values by
  the layer unit metadata. For example, at `metersPerUnit = 0.01` (centimeter
  scale) the default gravity magnitude is `981.0 cm/s^2`.

> **Angles are always in degrees.** Every rotation-related physics parameter is
> in degrees, including drive stiffness (units of torque/degree for an angular
> drive).

> **Keep units consistent across all layers.** Automatic reconciliation of unit
> scales between layers is not supported; author every layer and asset at the
> same scale.

## Multiple Physics Scenes

A stage may contain multiple `PhysicsScene` prims. `PhysicsRigidBodyAPI` and
`PhysicsCollisionAPI` have a `physics:simulationOwner` relationship that assigns
a body or collider to a specific scene. If it is not set, the first
`PhysicsScene` found during traversal is used. Objects in separate physics
scenes do not collide with each other.

```usda
def PhysicsScene "physicsScene0"
{
}

def PhysicsScene "physicsScene1"
{
}

def Cube "cube0" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsCollisionAPI"]
)
{
    rel physics:simulationOwner = </physicsScene0>
}
```

> **ovphysx steps all scenes together.** Multiple `PhysicsScene` prims in one
> stage become separate PhysX scenes, but they all advance together on each
> `step()`; per-scene stepping is not surfaced. For isolated stages or truly
> independent parallel sims, use separate subprocesses (refer to the threading and
> single-attached-stage notes in the [Developer Guide](../developer_guide.md)).

## Physics Solver

The solver iteratively resolves constraints (joints, contacts) to reach a
physically plausible state without overlaps or disconnected joints. Two
strategies are available, selected on `PhysxSceneAPI`:

- **TGS** (Temporal Gauss-Seidel, the default): converges faster; can overshoot
  (objects gaining excess velocity) in complex collision scenarios. TGS benefits
  from `physxScene:enableExternalForcesEveryIteration` and needs only very few
  (around 1) velocity iterations.
- **PGS** (Projected Gauss-Seidel): converges more slowly but is less prone to
  overshooting.

Iteration counts control accuracy vs. cost. Position iterations keep bodies from
overlapping; velocity iterations prevent bodies from picking up artificial
velocity on interaction. Both are clamped per-scene with min/max attributes.

### `.usda`

```usda
def PhysicsScene "physicsScene" (
    prepend apiSchemas = ["PhysxSceneAPI"]
)
{
    vector3f physics:gravityDirection = (0, 0, -1)
    float physics:gravityMagnitude = 9.81
    uniform token physxScene:solverType = "TGS"
    uniform uint physxScene:minPositionIterationCount = 4
    uniform uint physxScene:maxPositionIterationCount = 16
    uniform uint physxScene:minVelocityIterationCount = 1
    uniform uint physxScene:maxVelocityIterationCount = 1
}
```

### Python (codeless PhysX schema)

PhysX-specific attributes live on codeless schemas that have no typed Python
class. Apply them by identifier and author the attributes generically. This
continues the script above, whose registration call already ran before the
stage was created:

```python
from pxr import Sdf

scene_prim = scene.GetPrim()
scene_prim.ApplyAPI("PhysxSceneAPI")
scene_prim.CreateAttribute("physxScene:solverType", Sdf.ValueTypeNames.Token).Set("TGS")
scene_prim.CreateAttribute("physxScene:maxPositionIterationCount", Sdf.ValueTypeNames.UInt).Set(16)
```

Refer to [Physics Schemas](../physics_schemas.md) for the full codeless-schema
registration story, and [Performance](../guides/performance.md) for guidance on
clamping iteration counts in bulk/RL scenes.

## PhysX GPU Memory Buffers

When simulating on the GPU, PhysX allocates internal buffers at initialization.
If a buffer overflows, the runtime reports an error and you must increase its
capacity on `PhysxSceneAPI`. The most common one to tune is the found/lost pairs
capacity:

```usda
def PhysicsScene "physicsScene" (
    prepend apiSchemas = ["PhysxSceneAPI"]
)
{
    uint physxScene:gpuFoundLostPairsCapacity = 10240
}
```

## CPU vs GPU Simulation

GPU dynamics are enabled per scene in USD with `physxScene:enableGPUDynamics`
(and a GPU broadphase through `physxScene:broadphaseType = "GPU"`):

```usda
def PhysicsScene "physicsScene" (
    prepend apiSchemas = ["PhysxSceneAPI"]
)
{
    bool physxScene:enableGPUDynamics = true
    uniform token physxScene:broadphaseType = "GPU"
}
```

ovphysx also exposes process- and instance-level controls that complement the
per-scene USD settings:

- `ovphysx.PhysX.set_cpu_mode(True)` forces a process-wide CPU-only mode (must be
  called before any instance is created; it cannot be reverted in that process).
- `PhysX(active_cuda_gpus="0,1")` selects explicit CUDA ordinals and their
  supported scene-distribution mode. Any non-empty value takes precedence over
  `PhysXConfig.scene_multi_gpu_mode`; one ordinal disables distribution.
- With `active_cuda_gpus` empty,
  `PhysXConfig(scene_multi_gpu_mode=...)` controls distribution of multiple
  `PhysicsScene` prims across GPUs.

GPU features (deformables, particles, SDF meshes, DirectGPU tensor workloads)
require GPU dynamics. Refer to the GPU/CPU and determinism sections of the
[Developer Guide](../developer_guide.md).

## Sleeping

Sleeping lets PhysX save computation when dynamic objects are at rest. An object
that stops moving for a short time goes to sleep until it is woken — by the
application (for example, applying a force or changing a joint drive target) or
by another object colliding with it.

Each dynamic object type exposes sleep-related properties: for rigid bodies,
`physxRigidBody:sleepThreshold` sets the mass-normalized kinetic-energy threshold
below which the body may sleep (set it to `0` to keep a body always awake).

From the runtime, rigid-body tensor bindings expose explicit
[`wake_up()` / `sleep()`](../tutorials/tensor_bindings.md) controls.

> **Sleeping and effective iteration counts (GPU).** In a GPU simulation it is
> efficient to apply the same iteration count to every awake object — the maximum
> over all awake objects in the scene. If a high-iteration object goes to sleep,
> the effective iteration count for the remaining awake objects can drop, which
> can change their behavior (notably for deformables). Keep this in mind when
> mixing objects with very different iteration settings.

## Stabilization

Stabilization improves the settling behavior of rigid bodies in large-pile or
high-interaction scenes by applying extra damping to slow-moving objects. When a
body's mass-normalized kinetic energy falls below its stabilization threshold,
damping is applied so it settles faster.

Stabilization reduces momentum and can cause unphysical effects, so it is **not**
recommended for robotics, industrial applications, or any simulation that needs
precise interactions.

Enable it at the scene level, then set the threshold per rigid body:

```usda
def PhysicsScene "physicsScene" (
    prepend apiSchemas = ["PhysxSceneAPI"]
)
{
    bool physxScene:enableStabilization = true
}

def Cube "box" (
    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysxRigidBodyAPI", "PhysicsCollisionAPI"]
)
{
    float physxRigidBody:stabilizationThreshold = 0.0001
}
```

## Reading Simulation Output

ovphysx does not write results back to the attached stage. To observe simulated
state, read it explicitly:

- [Tensor bindings](../tutorials/tensor_bindings.md) — bulk, DLPack-friendly
  read/write of poses, velocities, joint state, and more.
- [ovstage output read](../ovstage_integration.md) — ovstage-native read of
  positions, orientations, velocities, and joint state.
