<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# Collision Behavior Guide

How to resolve common contact-quality issues. This complements
[Colliders](../simulation_setup/collision.md) (which covers authoring collider
types) and [Rigid Bodies](../simulation_setup/rigid_bodies.md) (CCD, mass,
materials). All PhysX-specific attributes below are on codeless schemas — refer to
[Physics Schemas](../physics_schemas.md).

## Settings That Affect Contact Quality

If contacts look wrong (penetration, jitter, tunneling), work through these,
roughly in order:

- **Increase the maximum contacts per frame (GPU).** With GPU dynamics the
  default GPU contact/patch buffers can be too small for complex scenes (SDF
  colliders generate many contacts). A warning is printed when a buffer
  overflows. Raise `physxScene:gpuMaxRigidContactCount` and
  `physxScene:gpuMaxRigidPatchCount` on the scene.
- **Increase the contact offset.** A larger `physxCollision:contactOffset` lets
  the solver react earlier. Too large slows collision detection — tune
  iteratively.
- **Enable speculative CCD.** `physxRigidBody:enableSpeculativeCCD` extends the
  contact offset by object velocity, improving response for fast movers (refer to
  [CCD](../simulation_setup/rigid_bodies.md#continuous-collision-detection)).
- **Make objects heavier / match masses.** Extremely light bodies, or large mass
  ratios between contacting bodies, make the solver converge poorly.
- **Reduce the maximum depenetration velocity.** A smaller
  `physxRigidBody:maxDepenetrationVelocity` slows the response and avoids
  overshoot.
- **Increase position iterations.** More
  `physxRigidBody:solverPositionIterationCount` improves convergence.
- **Increase friction** (within physically reasonable ranges — few materials
  exceed ~2.0) to help objects come to rest.
- **Reduce the timestep** (raise `physxScene:timeStepsPerSecond`) — only if the
  above are insufficient.

## Worked Example

A scene of SDF cones falling on a ground plane, tuned for contact quality. It
uses core `UsdPhysics` plus codeless PhysX attributes (register the codeless
schemas first — refer to [Physics Schemas](../physics_schemas.md)):

```python
import ovphysx
from pxr import Plug, Sdf, UsdGeom, UsdPhysics, Gf

Plug.Registry().RegisterPlugins([str(p) for p in ovphysx.codeless_schema_paths()])

UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)

scene = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
scene.CreateGravityMagnitudeAttr(9.81)
scene_prim = scene.GetPrim()
scene_prim.ApplyAPI("PhysxSceneAPI")
scene_prim.CreateAttribute("physxScene:gpuMaxRigidContactCount", Sdf.ValueTypeNames.UInt).Set(1000000)
scene_prim.CreateAttribute("physxScene:gpuMaxRigidPatchCount", Sdf.ValueTypeNames.UInt).Set(100000)
scene_prim.CreateAttribute("physxScene:timeStepsPerSecond", Sdf.ValueTypeNames.UInt).Set(120)

# A dynamic cone with an SDF collider, tuned for good contacts.
cone = UsdGeom.Mesh.Define(stage, "/World/Cone")  # provide cone mesh geometry
cone_prim = cone.GetPrim()
UsdPhysics.CollisionAPI.Apply(cone_prim)
UsdPhysics.MeshCollisionAPI.Apply(cone_prim).CreateApproximationAttr("sdf")
UsdPhysics.RigidBodyAPI.Apply(cone_prim)
UsdPhysics.MassAPI.Apply(cone_prim).CreateMassAttr(10.0)

cone_prim.ApplyAPI("PhysxCollisionAPI")
cone_prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(0.25)
cone_prim.ApplyAPI("PhysxSDFMeshCollisionAPI")
cone_prim.CreateAttribute("physxSDFMeshCollision:sdfResolution", Sdf.ValueTypeNames.Int).Set(200)
cone_prim.ApplyAPI("PhysxRigidBodyAPI")
cone_prim.CreateAttribute("physxRigidBody:solverPositionIterationCount", Sdf.ValueTypeNames.Int).Set(30)
cone_prim.CreateAttribute("physxRigidBody:maxDepenetrationVelocity", Sdf.ValueTypeNames.Float).Set(100.0)
cone_prim.CreateAttribute("physxRigidBody:enableSpeculativeCCD", Sdf.ValueTypeNames.Bool).Set(True)
```

## Improving SDF Collision Behavior

- **Tessellation.** Use watertight triangle meshes without self-intersections. If
  the mesh has holes, the cooker closes them with a surface it chooses. When two
  dynamic SDF meshes interact, at most one contact is generated per triangle, so
  remesh large faces into smaller triangles for objects of very different sizes.
- **SDF resolution.** Too low omits thin features; too high costs memory and
  speed. ~250 is a good default; rarely exceed 1000. For extreme cases, split the
  mesh into parts with separate SDFs. Thin shells below ~2x the grid spacing may
  not be captured. When an SDF collider meets a static triangle mesh, each static
  triangle is tested against the dynamic collider's SDF, so ensure the dynamic
  collider's SDF is high quality.

## Initial Collision Overlap

If collision shapes overlap at the start of simulation, the engine applies strong
repulsive forces to separate them, causing unstable, unrealistic motion. Position
objects so their collision shapes do not overlap at the first step. Use
[OmniPVD recording](../tutorials/omnipvd_recording.md) to inspect initial
contacts.
