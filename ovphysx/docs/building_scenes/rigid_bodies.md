<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Rigid Bodies

A rigid body is an object that moves but never bends, such as a box or a wheel. Objects that keep
their shape come in three kinds, and each kind is moved by something different.

![Side view. A blue kinematic platform floats 1 m above the static ground and slides from x = 1 m to x = 2 m and back. A green dynamic box rides on it. A band at the bottom shows the write your program makes before every step, and the program prints the x position of the platform and the box every second.](images/guide_rigid_bodies_kinematic.png)

## Three Kinds of Objects

| Kind | In USD | What Moves It |
| --- | --- | --- |
| Static collider | `PhysicsCollisionAPI` only | Nothing. It never moves. |
| Dynamic body | `PhysicsRigidBodyAPI` and `PhysicsCollisionAPI` | Physics: gravity, contacts, joints, and forces. |
| Kinematic body | The same API schemas, plus `physics:kinematicEnabled = true` | Your program. Gravity and contacts do not move it, but it moves the dynamic bodies it touches. |

A rigid body moves together with every [prim](../reference/glossary.md#scenes-and-stages) below it in the scene. Its colliders give it a
shape: a body without a collider falls through everything.
[Colliders and Materials](colliders_and_materials.md) covers shapes.

## Mass

If the scene sets no mass, ovphysx computes a dynamic body's mass from the volume of its
colliders and a density. It uses the density of the body's
[material](colliders_and_materials.md#materials) if the material sets one, or else 1,000 kg/m³,
the density of water. To choose the mass yourself, apply `PhysicsMassAPI` to the body and set
`physics:density` or `physics:mass`:

| You Set | ovphysx Uses | Mass of a 0.5 m Cube |
| --- | --- | --- |
| Nothing, and no material density | Volume times 1,000 kg/m³ | 125 kg |
| `physics:density` | Volume times your density | 62.5 kg for a density of 500 |
| `physics:mass` | Your mass | 2 kg for a mass of 2 |

If both are set, ovphysx uses `physics:mass`. A dynamic body with no collider and no mass set gets a mass
of 1 in the scene's mass unit, 1 kg in these guides, and ovphysx logs a warning.

## Starting Velocity

To start a body moving, set `physics:velocity`, in distance per second, and
`physics:angularVelocity`, in degrees per second. Both use world coordinates.
[Reading Values](../simulation_data/reading_values.md#common-rigid-body-attributes) lists the
units that reads return.

## Moving a Kinematic Body

To move a kinematic body, write a kinematic target before each step: the position the body must
reach at the end of that step. ovphysx moves the body there during the step, at the speed that
takes, so the body carries what rests on it. The following program, `rigid_bodies_kinematic.py`, slides a platform back and
forth with a box on it:

<details class="getting-started-code">
<summary>The imports and the scene, in USD</summary>

```{literalinclude} code/rigid_bodies_kinematic.py
:language: python
:start-after: [example-start]
:end-before: [scene-end]
```

</details>

```{literalinclude} code/rigid_bodies_kinematic.py
:language: python
:start-after: [scene-end]
:end-before: [example-end]
```

It prints:

```{literalinclude} code/rigid_bodies_kinematic.out
:language: output
```

Notice the following:

- The write selects the platform alone with `pattern`.
  [Writing Values](../simulation_data/writing_values.md) explains writes.
- The box moves with the platform because friction carries it.
- Without a new target, the platform stops where it is. `kinematicTargetOrientation` turns a
  kinematic body in the same way.

Do not move a kinematic body by writing `position`. The body then jumps to the new place, so
nothing rides along:

![Two copies of the moving platform. On the left, the program writes kinematicTargetPosition and the box rides along. On the right, the program writes position, so the platform jumps to each new place without carrying the box, which stays where it was.](images/guide_rigid_bodies_teleport.png)

Kinematic targets work in the default mode and on the CPU, but not with DirectGPU. Refer to
[CPU and GPU](../getting_started/concepts.md#cpu-and-gpu).

## More Detail

- [Writing Values](../simulation_data/writing_values.md) shows how to push a dynamic body with a
  `force` or a new `linearVelocity`.
- [Rigid Bodies (reference)](../simulation_setup/rigid_bodies.md) covers mass details, fast-moving bodies,
  and instancing.
- [Kinematic Support Geometry](../simulation_setup/kinematic_support.md) covers conveyor belts
  and moving kinematic bodies through [stage](../reference/glossary.md#scenes-and-stages) edits.
- [Rigid body attributes](../read_write/attributes/rigid_body.md) lists every value you can read
  and write.
- [PhysicsRigidBodyAPI](../population/PhysicsRigidBodyAPI.md) and
  [PhysicsMassAPI](../population/PhysicsMassAPI.md) list the USD attributes and their defaults.
- Next: [Colliders and Materials](colliders_and_materials.md) gives bodies their shapes.
