<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Simulation Setup

How each USD physics setting becomes part of the simulation, with every option and its
limits. The [Building Scenes](../building_scenes/loading_scenes.md) guides introduce the same
topics in short form.

| Page | What It Covers |
| --- | --- |
| [Physics Scene and Simulation Configuration](../simulation_setup/physics_scene.md) | Scene settings, units, solver, GPU memory, CPU and GPU simulation, sleeping and stabilization. |
| [Colliders](../simulation_setup/collision.md) | Collision shapes, mesh cooking, offsets, filtering and CPU and GPU compatibility. |
| [Rigid Bodies](../simulation_setup/rigid_bodies.md) | Body types, frames, materials, mass, continuous collision detection, velocities and forces. |
| [Kinematic Support Geometry](../simulation_setup/kinematic_support.md) | Moving platforms, conveyors and surface velocity. |
| [Joints](../simulation_setup/joints.md) | Joint frames, drives, limits, joint types and joint instancing. |
| [Articulations](../simulation_setup/articulations.md) | Robot structure, roots, drives, friction, closed loops, mimic joints, tendons and inverse dynamics. |
| [Deformable Bodies](../simulation_setup/deformables.md) | Volume and surface deformables, mesh generation, materials and attachments. |
| [Particles](../simulation_setup/particles.md) | Particle systems, particle sets, offsets and reading particle state. |

```{toctree}
:hidden:
:maxdepth: 1

/simulation_setup/physics_scene
/simulation_setup/collision
/simulation_setup/rigid_bodies
/simulation_setup/kinematic_support
/simulation_setup/joints
/simulation_setup/articulations
/simulation_setup/deformables
/simulation_setup/particles
```
