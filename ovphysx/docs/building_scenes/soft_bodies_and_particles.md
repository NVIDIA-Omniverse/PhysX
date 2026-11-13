<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Soft Bodies and Particles

Some objects change shape: a rubber toy squashes, cloth folds, and water flows. ovphysx
simulates them with soft bodies and particles. Both run only on an NVIDIA GPU. On the CPU,
ovphysx logs an error and leaves them out of the simulation.

![A soft green cube falls from 0.75 m, squashes to 0.363 m tall when it hits the ground, springs back, and settles at about 0.46 m tall. Next to it, the program's output lines appear one by one as simulated time passes.](images/guide_soft_bodies_cube.png)

## Soft Bodies

A soft body, also called a deformable body, is simulated on a mesh of points joined into small
shapes. When the body hits something, its points move apart or together. Forces between them then pull
the mesh back toward its rest shape, the shape it has when nothing pushes on it.

| Kind | Simulation Mesh | Use It For |
| --- | --- | --- |
| Volume | Tetrahedra, in a `TetMesh` | Solid objects that squash, such as rubber or flesh. |
| Surface | Triangles, in a `Mesh` | Thin objects that bend, such as cloth or paper. |

A scene marks a soft body with these API schemas:

| API Schema | Applied To | What It Does |
| --- | --- | --- |
| `OmniPhysicsDeformableBodyAPI` | The body | Makes the prim a soft body. |
| `OmniPhysicsVolumeDeformableSimAPI` or `OmniPhysicsSurfaceDeformableSimAPI` | The simulation mesh | Marks the mesh to simulate and holds its rest shape. |
| `PhysicsCollisionAPI` | The collision mesh | Marks the mesh the body collides with. A soft body needs one. |
| `OmniPhysicsDeformableMaterialAPI` | A material bound to the body | Sets how soft the body is. |

Assets often have no tetrahedral mesh. With `PhysxAutoDeformableBodyAPI`, ovphysx builds the
simulation mesh from a render mesh when it attaches the [stage](../reference/glossary.md#scenes-and-stages).

## A Soft Cube

The following program, `soft_bodies_cube.py`, drops a soft cube with 0.5 m sides and prints its height. Its simulation mesh is
also its collision mesh: the eight corners of the cube, joined into five tetrahedra. Real soft bodies
use many more points.

```{literalinclude} code/soft_bodies_cube.py
:language: python
:start-after: [example-start]
:end-before: [example-end]
```

It prints:

```{literalinclude} code/soft_bodies_cube.out
:language: output
```

Notice the following:

- The cube settles at about 0.46 m, pressed down by its own weight. A rigid box would print
  0.500 m on every line.
- The material sets the softness. `youngsModulus` is the stiffness in pascals, and a lower value
  is softer. `poissonsRatio` makes the cube bulge sideways when it is squashed, and
  `elasticityDamping` makes its wobbling die out.
- Here, the `points` arrays are in GPU memory, and `numpy()` copies them to the CPU.

## Particles

Particles simulate liquids and granular material such as sand. Each particle is a small sphere,
and many of them together flow, splash, and settle:

![Side view of 750 liquid particles. A block of particles falls from where the dashed outline shows it started and splashes on the ground in a low mound that spreads outward, as water does.](images/guide_soft_bodies_particles.png)

A scene needs two parts:

| Part | What It Does |
| --- | --- |
| A `PhysxParticleSystem` prim | Holds settings that its particles share, such as their size, which `particleContactOffset` sets. |
| A `Points` prim with `PhysxParticleSetAPI` | Makes each point a particle. Its `physxParticle:particleSystem` relationship names the particle system. |

On the particle set, set `physxParticle:fluid` to `1` for a liquid or to `0` for a granular
solid.

## Reading Their State

A read returns one array per object. Its `points` hold one row per mesh point or particle,
relative to the [prim](../reference/glossary.md#scenes-and-stages) that holds them.

| Object Type | One Array Per |
| --- | --- |
| `DEFORMABLE_VOLUME` | Volume soft body |
| `DEFORMABLE_SURFACE` | Surface soft body |
| `DEFORMABLE_SKIN_MESH` | Render mesh that follows a soft body. Its `points` are read-only. |
| `PARTICLE_SET` | Particle set |

## More Detail

- [Deformable Bodies](../simulation_setup/deformables.md) covers render meshes, automatic mesh
  generation, attachments, and limitations.
- [Particles (reference)](../simulation_setup/particles.md) explains particle size, spacing, mass, and
  density.
- [Deformable volume](../read_write/attributes/deformable_volume.md),
  [Deformable surface](../read_write/attributes/deformable_surface.md),
  [Particle set](../read_write/attributes/particle_set.md), and
  [Deformable material](../read_write/attributes/deformable_material.md) list every readable and
  writable attribute.
- [OmniPhysicsDeformableBodyAPI](../population/OmniPhysicsDeformableBodyAPI.md),
  [PhysxAutoDeformableBodyAPI](../population/PhysxAutoDeformableBodyAPI.md),
  [PhysxParticleSystem](../population/PhysxParticleSystem.md), and
  [PhysxParticleSetAPI](../population/PhysxParticleSetAPI.md) describe each scene setting.
- Next: [Selecting Objects](../simulation_data/selecting_objects.md) shows how to pick the
  objects a read or write works on.
