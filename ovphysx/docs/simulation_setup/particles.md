<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# Particles

ovphysx can simulate GPU-accelerated position-based-dynamics (PBD) particles for
fluids and granular media (sand, etc.). Particle sets interact with rigid bodies,
articulations, and deformables.

> **What ovphysx supports for particles.** You author the particle system and
> particle sets in USD, ovphysx simulates them, and you read the resulting
> particle positions and velocities back through the ovstage
> [output read](../ovstage_integration.md) API (object type `PARTICLE_SET`,
> attributes `points` / `velocities`). There is **no particle tensor-control
> API**, and fluid render post-processing (isosurface, anisotropy, smoothing) and
> the Kit particle-sampler UI are out of scope for ovphysx.

> **Particles require GPU simulation.** Enable GPU dynamics on the physics scene
> (`physxScene:enableGPUDynamics = true`, `physxScene:broadphaseType = "GPU"`) —
> refer to [Physics Scene](physics_scene.md). CPU simulation of particles is not
> supported. The particle schema is not finalized and may change.

## Simulation Components

A particle simulation has three parts:

1. **Particle sets** — groups of particles backed by a `UsdGeom.Points` or
   `UsdGeom.PointInstancer` prim with `PhysxParticleSetAPI` applied.
2. **A particle system** (`PhysxParticleSystem` prim) — holds simulation
   parameters shared by all its particle sets. Particle sets reference their
   system through the `physxParticle:particleSystem` relationship.
3. **A PBD particle material** — bound to the particle system, provides
   additional shared parameters (and density; refer to the section below).

Multiple particle systems per stage are supported, but particles in different
systems do not collide with each other.

## Authoring a Particle Set

```usda
def PhysxParticleSystem "particleSystem"
{
    float particleContactOffset = 0.05
    bool particleSystemEnabled = 1
    rel simulationOwner = </World/physicsScene>
}

def Points "particles" (
    prepend apiSchemas = ["PhysxParticleSetAPI"]
)
{
    point3f[] points = [(0, 0, 1), (0, 0, 2), (0, 0, 3)]
    float[] widths = [0.1, 0.1, 0.1]
    bool physxParticle:fluid = 0
    rel physxParticle:particleSystem = </World/particleSystem>
}
```

Set `physxParticle:fluid = 1` for a fluid set, or `0` for solid/granular. You can
populate the point positions programmatically (a grid, a sampled volume, etc.)
and set them on the `Points` prim's `points` attribute.

## Offsets

The particle system's offsets control interaction distances.

**Particle-particle.** Solid particle size is governed by the *Solid Rest
Offset*: two solid particles touch when their centers are `2 * solidRestOffset`
apart. The *Fluid Rest Offset* is not a radius; combined with particle mass it
sets the fluid's target (rest) density. Two particles are neighbors (and generate
solver constraints) when within `2 * particleContactOffset`; the neighborhood
size is capped by `maxNeighborhood` (default 96).

![Solid particle contact offsets](images/particles_offsets_solidParticles.png)

**Particle-collider.** Contacts are generated when a particle center is closer to
a collider than the sum of the particle system's and the collider's *Contact
Offset*, and contact occurs at the sum of their *Rest Offsets* (the solid rest
offset does not apply here).

![Particle-to-collider offsets](images/particles_offsets_collider.png)

**Autocomputation.** By default the offsets are derived from
`particleContactOffset` (default 0.05 m, scaled to scene units), so adjust it to
change particle resolution:

- Contact Offset = `1.0 * particleContactOffset`
- Rest Offset = `0.99 * particleContactOffset`
- Fluid Rest Offset = `0.99 * 0.6 * particleContactOffset`
- Solid Rest Offset = `0.99 * particleContactOffset`

## Mass and Density

Particle mass is resolved with this precedence (lowest to highest):

```
default 1000 kg/m^3  <  material density (scene)  <  material density (particle system)  <  mass-component density  <  mass-component mass
```

Bind a PBD particle material to the physics scene or the particle system to set
density, or apply a `UsdPhysics.MassAPI` (mass component) to a particle set to
override. When mass is set, each particle's mass is `mass / numParticles`. When
density is set, per-particle mass is derived from the density and the rest-offset
volume (solid or fluid rest offset, as appropriate).

## Reading Particle State

ovphysx does not write particle results back to the stage. Read simulated
particle positions and velocities through the ovstage
[output read](../ovstage_integration.md) API using the `PARTICLE_SET` object type
(`points`, `velocities`). Point-instancer-backed and points-backed sets are both
read this way.
