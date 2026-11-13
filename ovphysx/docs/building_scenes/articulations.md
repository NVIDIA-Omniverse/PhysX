<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Articulations

An **articulation** is a set of rigid parts, called links, connected by joints in a tree, such
as a robot arm, a gripper, or a walking robot. ovphysx moves the links together, so the joints
always hold exactly. Your program controls an articulation by setting target angles for its
joints.

![Top view of a robot arm on a fixed base. The program sets new targets for its two joints, drawn as a dashed outline, and the arm turns into that pose. A faint outline marks the starting pose. A plot shows the shoulder angle rising from 0 to 45 degrees and the elbow angle falling from 90 to 60 degrees, each settling on its dashed target line.](images/guide_articulations_arm.png)

## Links, Joints, and the Root

Each link is a [prim](../reference/glossary.md#scenes-and-stages) with `PhysicsRigidBodyAPI`, and joint prims connect the links. Apply
`PhysicsArticulationRootAPI` to the prim that holds the articulation: ovphysx follows the joints under it
and simulates the links as one articulation. The joints can branch but not close a loop. Without
the root, the links are separate rigid bodies, and their joints can stretch slightly under load.

The base is the link where the tree starts:

| Base | How the Scene Marks It | Example |
| --- | --- | --- |
| Fixed | A joint with only one body attaches the base to the world. | A factory arm bolted to the floor. |
| Floating | No joint attaches the articulation to the world, so it can move and fall. | A walking robot or a ragdoll. |

## Drives and the Starting Pose

Each motor of an articulation is a joint drive, which pulls its joint toward a target angle, as
[Joints](joints.md#drives) explains. Your program usually writes new targets while the
simulation runs.

Joints start at an angle of 0. To start a joint at another angle, apply
`PhysicsJointStateAPI:angular` to it and set `state:angular:physics:position` in degrees.
When ovphysx attaches the [stage](../reference/glossary.md#scenes-and-stages), it places the links to match these angles.

## An Arm That Moves to Its Targets

The following program, `articulations_arm.py`, builds a two-link arm that turns in a horizontal plane and writes targets of 45 and
60 degrees. The scene has these parts:

| Scene Part | What It Does |
| --- | --- |
| `Arm`, with `PhysicsArticulationRootAPI` | Makes the links and joints under it one articulation. |
| `BaseJoint` | A fixed joint with only one body, so `Base` is fixed to the world. |
| `Shoulder` and `Elbow` | Revolute joints that turn around the vertical z axis. `PhysicsDriveAPI:angular` gives each one a drive. |
| `PhysicsJointStateAPI:angular` on `Elbow` | Starts the elbow at 90 degrees. Its drive target is also 90 degrees, so the elbow holds that angle. |

<details class="getting-started-code">
<summary>The imports and the scene, in USD</summary>

```{literalinclude} code/articulations_arm.py
:language: python
:start-after: [example-start]
:end-before: [scene-end]
```

</details>

```{literalinclude} code/articulations_arm.py
:language: python
:start-after: [scene-end]
:end-before: [example-end]
```

It prints:

```{literalinclude} code/articulations_arm.out
:language: output
```

Notice the following:

- Joint angles are in degrees, both in the scene and in reads and writes.
- A target stays until your program writes a new one, so one write is enough. An articulation
  that has almost stopped moving falls asleep to save work, and a new target wakes it.

## Reading and Writing an Articulation

An articulation has data at three levels, each with its own object type:

| Object Type | One Entry Per | Examples of Attributes |
| --- | --- | --- |
| `ARTICULATION` | Articulation | `rootPosition`, `rootOrientation`, `rootLinearVelocity` |
| `ARTICULATION_LINK` | Link | `position`, `orientation`, `linearVelocity` |
| `ARTICULATION_JOINT` | Joint, with one row per axis | `jointPosition`, `jointVelocity`, `jointPositionTarget`, `jointVelocityTarget` |

Link poses are read-only, because the root and the joints decide where every link is. To move an
articulation, write its root state or its joint values.

Read-only attributes of `ARTICULATION` map its parts back to the scene. `linkPath` holds the prim
of each link, and `linkInboundJoint` holds the joint that leads into each link. `dofJointPath` and
`dofJointType` hold the joint and joint type of each axis that can move. Each path comes as a handle,
which `result.dictionary.path_to_string(ovstage.PrimPath(handle))` turns into text.

## More Detail

- [Articulations (reference)](../simulation_setup/articulations.md) covers where the root can go, closed
  loops, joint friction, and armature.
- [Joints (reference)](../simulation_setup/joints.md) lists the joint types an articulation
  supports.
- [Stable Simulations](../common_tasks/stable_simulations.md) and
  [Articulation Stability](../guides/articulation_stability.md) help when an articulation shakes or
  flies apart.
- [Articulation joint](../read_write/attributes/articulation_joint.md),
  [Articulation link](../read_write/attributes/articulation_link.md), and
  [Articulation](../read_write/attributes/articulation.md) list every readable and writable
  attribute.
- [PhysicsArticulationRootAPI](../population/PhysicsArticulationRootAPI.md),
  [PhysicsDriveAPI](../population/PhysicsDriveAPI.md), and
  [PhysicsJointStateAPI](../population/PhysicsJointStateAPI.md) describe each scene setting.
- Next: [Soft Bodies and Particles](soft_bodies_and_particles.md).
