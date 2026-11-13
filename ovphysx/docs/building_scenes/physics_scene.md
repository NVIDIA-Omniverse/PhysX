<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# The Physics Scene

The `PhysicsScene` [prim](../reference/glossary.md#scenes-and-stages) holds the settings that apply to the whole simulation, such as gravity.
This page covers gravity, units, the time step, and the few scene settings worth knowing early.

![Two 1 m boxes start with their centers 2 m above the ground, one with Earth gravity and one with Moon gravity. A height label follows each box. The Earth box lands before 0.75 s, and the Moon box lands between 1.25 s and 1.5 s. Next to them, the program prints both heights every 0.25 s.](images/guide_physics_scene_gravity.png)

## Gravity

Two attributes of the `PhysicsScene` set gravity:

| Attribute | Meaning | Default |
| --- | --- | --- |
| `physics:gravityDirection` | The direction gravity pulls in. | Down the up axis, such as `(0, 0, -1)` when Z is up. |
| `physics:gravityMagnitude` | The strength, in distance per second squared. | 9.81 m/s², converted to the stage's units. |

A scene without any `PhysicsScene` gets the same defaults.

The following program, `physics_scene_gravity.py`, drops the same box twice: once with Earth gravity and once with Moon
gravity, 1.62 m/s².

```{literalinclude} code/physics_scene_gravity.py
:language: python
:start-after: [example-start]
:end-before: [example-end]
```

It prints:

```{literalinclude} code/physics_scene_gravity.out
:language: output
```

Gravity belongs to the scene, so each drop loads its own scene.

## Units and Up Axis

Three settings at the top of the USD file, called layer metadata, set the units and the up
direction:

| Metadata | Meaning | These Guides Use |
| --- | --- | --- |
| `metersPerUnit` | The length of one scene unit, in meters. | `1`, for meters |
| `kilogramsPerUnit` | The mass of one scene unit, in kilograms. | `1`, for kilograms |
| `upAxis` | The axis that points up, `"Y"` or `"Z"`. | `"Z"` |

ovphysx reads every length and mass in these units, scales its defaults to them, and returns
results in them. For example, a [stage](../reference/glossary.md#scenes-and-stages) in centimeters, with `metersPerUnit = 0.01`, gets a
default gravity of 981 cm/s².

## The Time Step

Your program sets the time step, not the scene. Each `step(dt)` or `step_sync(dt)` call moves
the simulation forward by `dt` seconds. These guides use 1/60 s.
[Stable Simulations](../common_tasks/stable_simulations.md#what-to-change) shows when smaller
steps help.

## Settings You Can Change

PhysX-specific scene settings live on `PhysxSceneAPI`. To use one, apply the API schema to the
`PhysicsScene` prim and set the value there. For example, this complete scene file, `cpu_scene.usda`, runs on the
CPU:

```usda
#usda 1.0
(
    metersPerUnit = 1
    kilogramsPerUnit = 1
    upAxis = "Z"
)

def Xform "World"
{
    def PhysicsScene "PhysicsScene" (
        prepend apiSchemas = ["PhysxSceneAPI"]
    )
    {
        bool physxScene:enableGPUDynamics = 0
        uniform token physxScene:broadphaseType = "MBP"
    }
}
```

These are the settings you are most likely to change:

| Setting | Default | What It Does |
| --- | --- | --- |
| `physxScene:enableGPUDynamics` | `true` | Simulates the scene on the GPU when one is available. To run the scene on the CPU, set it to `false` and set `physxScene:broadphaseType` to `"MBP"`, the CPU broad phase (multi-box pruning). |
| `physxScene:minPositionIterationCount` and `physxScene:maxPositionIterationCount` | `1` and `255` | The fewest and most passes the solver makes over each body's position in a step. The solver is the part of PhysX that works out how bodies move and push on each other. Refer to [Stable Simulations](../common_tasks/stable_simulations.md#what-to-change). |
| `physxScene:disableSleeping` | `false` | Sleeping stops simulating a body at rest until another body touches it or your program changes it. Set it to `true` to keep every body awake. |

To use the CPU for every scene in a process, refer to
[CPU and GPU](../getting_started/concepts.md#cpu-and-gpu).

## More Detail

- [Physics Scene and Simulation Configuration](../simulation_setup/physics_scene.md) covers the
  solver, GPU memory, multiple scenes, and stabilization.
- [PhysicsScene](../population/PhysicsScene.md) and [PhysxSceneAPI](../population/PhysxSceneAPI.md)
  list every scene attribute with its default.
- Next: [Rigid Bodies](rigid_bodies.md) adds objects that move.
