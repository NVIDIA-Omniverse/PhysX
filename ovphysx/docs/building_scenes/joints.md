<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Joints

A joint connects two bodies and limits how they move relative to each other, like the hinge of a
door.

![Four panels, recorded together in one ovphysx run. A revolute joint lets an arm swing around a hinge. A prismatic joint lets a box slide down a tilted rail until it stops at the limit. A spherical joint lets a ball circle under its pivot. A distance joint lets a ball fall straight down until it is about 0.9 m from its anchor, then swing.](images/guide_joints_types.png)

## Joint Types

Each type is its own [prim](../reference/glossary.md#scenes-and-stages) type in USD:

| Type | Prim Type | The Second Body Can | Example |
| --- | --- | --- | --- |
| Fixed | `PhysicsFixedJoint` | Not move relative to the first. | Two parts welded together. |
| Revolute | `PhysicsRevoluteJoint` | Turn around one axis. | A door hinge or a wheel. |
| Prismatic | `PhysicsPrismaticJoint` | Slide along one axis. | A drawer. |
| Spherical | `PhysicsSphericalJoint` | Turn in any direction around one point. | A shoulder. |
| Distance | `PhysicsDistanceJoint` | Move freely within a minimum and maximum distance. | A rope. |
| D6 (six degrees of freedom) | `PhysicsJoint` | Slide along and turn around each of three axes, each free, limited, or locked. | Anything the other types cannot do. |

## Connecting Two Bodies

A joint prim names its bodies with the relationships `physics:body0` and `physics:body1`. Leave
one out to attach the other body to the world.

The joint holds a point of one body to a point of the other. You give each point, with its axes,
as a joint frame:

| Attribute | Meaning |
| --- | --- |
| `physics:localPos0`, `physics:localRot0` | Position and rotation of the joint frame on body 0, in body 0's own coordinates. When body 0 is the world, they are in world coordinates. |
| `physics:localPos1`, `physics:localRot1` | The same for body 1. |
| `physics:axis` | The frame axis, `"X"`, `"Y"`, or `"Z"`, that a revolute joint turns around or a prismatic joint slides along. |

Place the bodies so that the two frames meet at the start. If they do not, ovphysx logs a
warning and snaps the bodies into place in the first step.

The two bodies of a joint do not collide with each other. To let them collide, set the joint's
own `physics:collisionEnabled` to `true`.

## Limits

A limit stops a joint at a minimum or a maximum, like a door stop.

| Joint | Limit Attributes | Unit |
| --- | --- | --- |
| Revolute | `physics:lowerLimit`, `physics:upperLimit` | Degrees |
| Prismatic | `physics:lowerLimit`, `physics:upperLimit` | Distance |
| Spherical | `physics:coneAngle0Limit`, `physics:coneAngle1Limit`, which form a cone around the joint's axis | Degrees |
| Distance | `physics:minDistance`, `physics:maxDistance` | Distance |
| D6 | One `PhysicsLimitAPI` per axis, such as `PhysicsLimitAPI:rotX` | Degrees or distance |

## Drives

A drive is a motor on a joint. It pushes the joint toward a target position, a target velocity,
or both, like a spring with a damper:

```text
force = stiffness × (target position − position)
      + damping × (target velocity − velocity)
```

On a revolute joint, apply `PhysicsDriveAPI:angular` and set these attributes:

| Attribute | Meaning |
| --- | --- |
| `drive:angular:physics:targetPosition` | The angle the drive pulls toward, in degrees. |
| `drive:angular:physics:targetVelocity` | The speed the drive pushes toward, in degrees per second. |
| `drive:angular:physics:stiffness` | How strongly the drive pulls toward the target angle, as torque per degree. |
| `drive:angular:physics:damping` | How strongly the drive resists any speed other than the target velocity. |

A prismatic joint takes `PhysicsDriveAPI:linear` with the same attributes, in distances. A drive
that keeps a speed, such as a spinning wheel, sets the stiffness to 0 and uses damping with a
target velocity.

The following program, `joints_drive.py`, turns an arm on a hinge to 90 degrees:

<details class="getting-started-code">
<summary>The imports and the scene, in USD</summary>

```{literalinclude} code/joints_drive.py
:language: python
:start-after: [example-start]
:end-before: [scene-end]
```

</details>

```{literalinclude} code/joints_drive.py
:language: python
:start-after: [scene-end]
:end-before: [example-end]
```

It prints:

```{literalinclude} code/joints_drive.out
:language: output
```

Notice the following:

- ovphysx reads joint angles only for the joints of an articulation, which
  [Articulations](articulations.md) introduces. The program computes the angle from the arm's position instead.
- The following figure shows the same arm with less and with more damping:

![Three copies of the arm, seen from above, with a damping of 0.1, 0.6, and 3. A plot shows each arm's angle over 1.6 s. With 0.1, the arm swings past the 90-degree target to 140 degrees and back. With 0.6, it settles on the target. With 3, it moves slowly and reaches only about 57 degrees.](images/guide_joints_drive.png)

A drive acts like a spring, so a steady load, such as the weight of an arm held out sideways,
holds the joint short of its target. A higher stiffness makes the gap smaller.

To change a drive target while the simulation runs, edit the joint in the [stage](../reference/glossary.md#scenes-and-stages), as
[Changing the Scene](../common_tasks/changing_the_scene.md) shows.

## Turning a Joint Off

To leave a joint out of the simulation, set `physics:jointEnabled = false` on the joint prim.
Its bodies then move freely.

## More Detail

- [Joints (reference)](../simulation_setup/joints.md) covers soft limits, breaking joints, gear
  joints, and joint instancing.
- [Articulations (reference)](../simulation_setup/articulations.md) covers the joints of articulations.
- DirectGPU mode, described in [Data on the GPU](../simulation_data/gpu_data.md), rejects
  distance joints.
- [PhysicsJoint](../population/PhysicsJoint.md),
  [PhysicsRevoluteJoint](../population/PhysicsRevoluteJoint.md),
  [PhysicsPrismaticJoint](../population/PhysicsPrismaticJoint.md),
  [PhysicsSphericalJoint](../population/PhysicsSphericalJoint.md),
  [PhysicsDistanceJoint](../population/PhysicsDistanceJoint.md),
  [PhysicsFixedJoint](../population/PhysicsFixedJoint.md),
  [PhysicsDriveAPI](../population/PhysicsDriveAPI.md), and
  [PhysicsLimitAPI](../population/PhysicsLimitAPI.md) list every setting.
- Next: [Articulations](articulations.md) builds a robot arm from joints and drives.
