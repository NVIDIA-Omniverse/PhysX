<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Simulation Performance Guide

Best practices for optimizing simulation performance with ovphysx. Most tuning is
either a per-scene USD attribute on the codeless `PhysxSceneAPI` (refer to
[Physics Scene](../simulation_setup/physics_scene.md) and
[Physics Schemas](../physics_schemas.md)) or a process-global config key set
before instance creation.

## Measure First

Before applying anything below, find out where the time actually goes. Setting
`OVPHYSX_PROFILER=trace` makes a run write a chrome trace JSON covering each
ovphysx call, the host-side stage load and collision cooking work, and most of the
PhysX SDK's own zones — no external tool needed, and the result is parseable from a
script. Setting `OVPHYSX_PROFILER=nvtx` instead publishes to an NVIDIA Nsight
Systems capture, correlated with CUDA activity, which is what lets you attribute
host-device transfers. Neither needs a rebuild and no backend is selected by
default. Refer to [Profiling](../tutorials/profiling.md).

Note that `ovphysx_step` only enqueues work: in a capture the simulation cost shows
up under the matching wait, not the step.

## Solver Iterations

Lowering solver iterations to the smallest count that still gives acceptable
fidelity helps a lot — clamp the per-scene iteration counts. PGS is typically
faster per iteration; TGS often reaches better stability with fewer iterations.
For the attributes and how to author them, refer to
[Physics Solver](../simulation_setup/physics_scene.md#physics-solver).

## CPU vs GPU by Scene Size

For only a few rigid bodies and articulations, CPU simulation is often faster; for
large scenes (for example RL with many environments), use GPU simulation and
access state in bulk through the session read/write API (`ovphysx_read` / `ovphysx_write`). How
to select CPU vs GPU (per-scene `physxScene:enableGPUDynamics`, process-wide
`set_cpu_mode`, and GPU selection) is covered in
[CPU and GPU Simulation](../simulation_setup/physics_scene.md#cpu-and-gpu-simulation).

## Reading State Efficiently

Unlike Kit-based Omni PhysX, ovphysx does not write simulation output back to
USD, so there is no per-step USD write-back cost. Read state in bulk through
the session read API (`ovphysx_read` / `PhysX.read`, backed by the ovstage
[output read](../ovstage_integration.md); device-native `warp.array` columns) rather than
per-prim. The older [tensor bindings](../tutorials/tensor_bindings.md) (DLPack, zero-copy on
device) are deprecated. Request the optional query and session results only when you use them:
passing NULL in C, or `metadata=False` in Python (the default for `read()` and `write()`), skips
their discovery work.

## Physics Thread Count

Three independent settings control CPU workers. They are not aliases.

`/physics/physxDispatcher` (default `false`) chooses which pool runs PhysX
CPU dispatcher tasks. Unset or `false` uses ovruntime's Carb dispatcher, which
submits those tasks into the process-wide `carb.tasking` pool. `true` creates
PhysX's own `PxDefaultCpuDispatcher` with a dedicated worker set, and PhysX
CPU tasks no longer run on Carb workers. Other Carbonite work still uses the
tasking pool either way. Hosts may set this before create through the legacy
`PhysXConfig.carbonite_overrides` / `ovphysx_create_args` carbonite path; there
is no scoped `GlobalConfigKey` for it.

`num_threads` (`/persistent/physics/numThreads`, default `8`) is the PhysX CPU
dispatcher limit, not the Carb pool size. With the default Carb dispatcher it
caps how many PhysX tasks may run at once on that pool (effective concurrency
is also bounded by `carb.tasking` `threadCount`). With `physxDispatcher=true`
it is the size of PhysX's own dispatcher. `num_threads=0` means no extra
PhysX dispatcher workers: those tasks run on the calling thread. For small
scenes that is often faster. Set it before constructing the instance; it is
not authored in USD:

```python
from ovphysx import GlobalConfigKey, PhysX, set_global_config

set_global_config(GlobalConfigKey.NUM_THREADS, 0)
physx = PhysX()
```

`/plugins/carb.tasking.plugin/threadCount` and
`/plugins/omni.tbb.globalcontrol/maxThreadCount` size the process-wide Carb
and TBB pools. ovphysx treats these as startup-only settings and writes no
count of its own, so an unconfigured pool sizes itself to the machine. TBB is
not the PhysX CPU dispatcher. Run each pool-size experiment in a fresh process
and pass the overrides to the first constructor through the legacy
`PhysXConfig` carbonite path:

```python
from ovphysx import PhysX, PhysXConfig

physx = PhysX(
    config=PhysXConfig(
        carbonite_overrides={
            "/plugins/carb.tasking.plugin/threadCount": 8,
            "/plugins/omni.tbb.globalcontrol/maxThreadCount": 8,
            "/physics/physxDispatcher": False,
        }
    )
)
```

A Carbonite host that started tasking before ovphysx remains the owner of its
pool size. Under ovphysx's startup-only configuration policy, worker-pool
changes requested after tasking starts are rejected. Carbonite provides a
separate runtime tasking reconfiguration API, but ovphysx does not expose it.

Raising `threadCount` does not raise the PhysX dispatcher cap: that is still
`num_threads`. Raising `num_threads` above the live tasking pool does not
create extra Carb workers. Smaller is not automatically faster for the
tasking pool. On a GPU-heavy scene with thousands of replicated environments,
a tasking pool well above the logical CPU count measured faster than one
sized at or below it, so treat that count as something to measure for your
own workload rather than a value to minimize.

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
performance. This is a per-scene USD attribute, so add it to the `PhysicsScene`
prim in the scene's `.usda` layer rather than setting it at runtime. The example
shows the prim as it appears with the attribute applied; extend your existing
`PhysicsScene` definition rather than adding a second one:

```usda
def PhysicsScene "physicsScene" (
    prepend apiSchemas = ["PhysxSceneAPI"]
)
{
    bool physxScene:enableSceneQuerySupport = false
}
```

## Multi-GPU

On multi-GPU machines, set `ConfigKey.ACTIVE_CUDA_GPUS` after create and before
attach for a potential speedup. Alternatively, when that key is unset, set
`GlobalConfigKey.SCENE_MULTI_GPU_MODE` before create. Refer to
[CPU and GPU Simulation](../simulation_setup/physics_scene.md#cpu-and-gpu-simulation).

## Collision Geometry Choice

Simpler collision geometry is faster: an SDF mesh collider is far more expensive
than a sphere. Prefer primitives, then convex hulls, before triangle/SDF meshes
(refer to [Colliders](../simulation_setup/collision.md)).

In GPU RL workloads, watch for this warning, which names the offending prim:

```text
ConvexMeshCookingTask: failed to cook GPU-compatible mesh, collision detection will fall back to CPU. Collisions with particles and deformables will not work with this mesh. Prim /World/envs/env_0/Rod
```

It is common for long, thin, high-aspect-ratio inputs. Such a mesh falls back to
CPU contact generation, which can badly hurt performance and prevents
interaction with GPU-only features. The convex-decomposition cooker emits the
same sentence prefixed with `ConvexDecompositionTask:`. Workarounds: use a
bounding-cube approximation, or a static triangle mesh if the geometry is not on
a dynamic body.

## Deformables and Particles

Deformables and particles are considerably more expensive than rigid bodies.
Where feasible, approximate a deformable with rigid bodies plus compliant
contacts, or rigid bodies connected by joints.
