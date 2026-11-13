<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Colliders and Materials

A collider gives an object a shape that other objects can hit. A material sets how that shape
behaves when it touches something: how much it grips and how much it bounces.

![Two cups made from the same U-shaped mesh stand on the ground, and a ball drops into each. On the left, the approximation is convexHull: one collision shape, drawn dashed, covers the cup's opening, so the ball stops on top. On the right, the approximation is convexDecomposition: three dashed hulls, two walls and a base, keep the hollow, so the ball falls in.](images/guide_colliders_and_materials_cup.png)

## Making an Object Solid

Apply `PhysicsCollisionAPI` to a shape [prim](../reference/glossary.md#scenes-and-stages), such as a `Cube` or a `Mesh`. On or below a rigid
body, the collider moves with the body. Anywhere else, it is static, like the ground.
[Rigid Bodies](rigid_bodies.md#three-kinds-of-objects) compares the kinds.

## Simple Shapes

The prim types `Cube`, `Sphere`, `Capsule`, `Cylinder`, and `Cone` collide with exactly their
own shape. Only a `Cube` can be stretched unevenly: for a box, scale a `Cube`. A `Plane` is an
infinite flat ground, and it cannot be part of a dynamic body. Simple shapes are the cheapest
and most stable choice, so use them when they fit.

## Meshes

A `Mesh` collider has an approximation: the shape that ovphysx builds from its triangles. To
choose it, apply `PhysicsMeshCollisionAPI` to the mesh and set `physics:approximation`:

| `physics:approximation` | Shape | Use It For |
| --- | --- | --- |
| `convexHull` | One convex hull: the tightest shape without dents that wraps the mesh. | Moving objects without hollows, such as a rock. |
| `convexDecomposition` | Several convex hulls that together follow the mesh. | Moving objects with hollows, such as a cup. |
| `none`, the default | The triangles themselves. | Static and kinematic objects only, such as a room. On a dynamic body, ovphysx uses a convex hull instead. |
| `sdf` | A signed distance field: a grid that stores how far each point is from the surface. | Detailed moving objects that touch closely. |
| `boundingCube`, `boundingSphere` | A box or a sphere around the mesh. | Rough shapes that cost little. |

## Turning Collisions Off

These settings make objects pass through each other:

| To Stop | Use |
| --- | --- |
| One collider from colliding at all | `physics:collisionEnabled = false` on the collider. |
| Two particular objects from colliding with each other | `PhysicsFilteredPairsAPI` on one of them, with the relationship `physics:filteredPairs` naming the other. |

## Materials

A physics material is a `Material` prim with `PhysicsMaterialAPI`. It holds these values:

| Attribute | Meaning |
| --- | --- |
| `physics:staticFriction` | How hard it is to start sliding. |
| `physics:dynamicFriction` | How strongly friction slows an object that already slides. |
| `physics:restitution` | How much an object bounces: 0 does not bounce, and 1 bounces back as fast as it hit. |
| `physics:density` | Mass per volume, such as kg/m³. [Rigid Bodies](rigid_bodies.md#mass) shows when ovphysx uses it. |

To give a collider a material, apply `MaterialBindingAPI` to it and point the relationship
`material:binding:physics` at the material. A binding on a body prim applies to every collider
below it. A collider without a material uses the default one: both frictions 0.5 and no bounce.

Set every value you need: a friction or restitution that the material leaves out is 0.

When two colliders touch, PhysX averages the values of their two materials, unless a material
sets another combine mode with `PhysxMaterialAPI`, as the program in [Three Bouncing Balls](#three-bouncing-balls) does.

## Three Bouncing Balls

The following program, `colliders_and_materials_bounce.py`, drops three balls whose materials differ only in restitution:

<details class="getting-started-code">
<summary>The imports and the scene, in USD</summary>

```{literalinclude} code/colliders_and_materials_bounce.py
:language: python
:start-after: [example-start]
:end-before: [scene-end]
```

</details>

```{literalinclude} code/colliders_and_materials_bounce.py
:language: python
:start-after: [scene-end]
:end-before: [example-end]
```

It prints how high each ball bounces back:

```{literalinclude} code/colliders_and_materials_bounce.out
:language: output
```

![Three balls with a restitution of 0.0, 0.5, and 0.9 fall from 1 m onto the ground. The first does not bounce. The second bounces back to 0.249 m and the third to 0.806 m, and a faint circle marks the top of each bounce.](images/guide_colliders_and_materials_bounce.png)

Notice the following:

- A ball bounces back to about its restitution squared times the drop height:
  0.5 × 0.5 × 1 m = 0.25 m, and 0.9 × 0.9 × 1 m = 0.81 m.
- The ground has no material, so its restitution is 0. Each ball's material sets
  `restitutionCombineMode = "max"`, so every contact uses the ball's restitution, not the average.

## More Detail

- [Colliders](../simulation_setup/collision.md) covers SDF settings, contact offsets, and which
  shapes work together on the GPU.
- [Collision Behavior Guide](../guides/collision_tuning.md) helps when contacts jitter or objects
  pass through each other.
- [PhysicsCollisionGroup](../population/PhysicsCollisionGroup.md) turns off collisions between
  whole sets of colliders.
- [PhysicsCollisionAPI](../population/PhysicsCollisionAPI.md),
  [PhysicsMeshCollisionAPI](../population/PhysicsMeshCollisionAPI.md),
  [PhysicsFilteredPairsAPI](../population/PhysicsFilteredPairsAPI.md),
  [PhysicsMaterialAPI](../population/PhysicsMaterialAPI.md),
  [PhysxMaterialAPI](../population/PhysxMaterialAPI.md), and
  [MaterialBindingAPI](../population/MaterialBindingAPI.md) list every setting.
- Next: [Joints](joints.md) connects bodies to each other.
