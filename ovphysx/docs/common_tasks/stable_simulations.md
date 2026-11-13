<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Stable Simulations

A simulation is unstable when objects shake, sink into each other, fly apart, or pass through each
other. The changes on this page fix most cases.

![Two side views of the same stack of six boxes, in a scene that allows the solver one position iteration. On the left, with 1/60 s steps, the stack leans, buckles after about 3 s, and its top box ends on the ground 4.2 m away. On the right, with 1/240 s steps, the stack stands still and its top box moves 0.003 m.](images/guide_stable_simulations_stacks.png)

## What to Change

| You See | Change | Setting |
| --- | --- | --- |
| Stacks lean and fall, joints stretch, objects shake | Take several smaller steps for each step your program needs, as in the figure. ovphysx never splits a step on its own. | The `dt` that you pass to `step` or `step_sync` |
| Stacks sag, objects sink into each other, joints drift apart | Give the solver more iterations, the passes it makes in each step to correct positions. More iterations cost time. | `physxRigidBody:solverPositionIterationCount`, 16 by default, or `physxArticulation:solverPositionIterationCount` for an articulation, 32 by default. The scene's `physxScene:maxPositionIterationCount` caps both. |
| A heavy object sinks through light ones | Keep the masses of objects that touch close to each other. | `physics:mass`, on a body with `PhysicsMassAPI` |
| Objects jump apart at the first step | Place the shapes so that they do not touch at the start. | The positions in your scene |
| Joints jitter or shake, or a gripper pushes away what it holds | Use the lowest stiffness that reaches the target, enough damping that the joint does not swing past it, and a force limit. [Drives](../building_scenes/joints.md#drives) explains them. | The drive's `stiffness`, `damping`, and `maxForce`, such as `drive:angular:physics:maxForce` |
| A fast object passes through a wall or the ground | Turn on continuous collision detection (CCD), which looks for contacts between steps. Try speculative CCD first, because it is cheaper. If objects still pass through, also turn on sweep CCD. | Speculative: `physxRigidBody:enableSpeculativeCCD`. Sweep: `physxRigidBody:enableCCD` with `physxScene:enableCCD`. |
| A thin object, or a somewhat fast one, passes through another | Raise the contact offset of the moving shape, so that contacts start at that distance. Larger offsets cost time. | `physxCollision:contactOffset` |

A setting whose name starts with `physx`, such as `physxRigidBody:enableCCD`, works only when the
matching API schema, here `PhysxRigidBodyAPI`, is applied to the [prim](../reference/glossary.md#scenes-and-stages).

Tests with simple box scenes showed the following:

- A 1,000 kg box sank straight through a stack of 1 kg boxes, even with 64 iterations. A 100 kg box
  sank about 1 cm.
- A 10 cm box at 10 m/s passed through a 2 cm wall. A contact offset of 0.3 m stopped it. At
  30 m/s, only CCD stopped it.

## A Stack That Stands

The figure comes from this program, `stable_simulations_stack.py`. Its scene allows the solver one position iteration, which makes
the stack of six 0.5 m boxes fragile. It simulates 5 s with 1/60 s steps, then with 1/240 s steps.

```{literalinclude} code/stable_simulations_stack.py
:language: python
:start-after: [example-start]
:end-before: [example-end]
```

It prints:

```{literalinclude} code/stable_simulations_stack.out
:language: output
```

With 1/240 s steps, four times as many, the same scene stands. Smaller steps often let a scene keep
fewer iterations.

## More Detail

- [Drive Stability](../guides/articulation_stability.md#drive-stability) tunes drives in depth, and
  [Joint Armature](../guides/articulation_stability.md#joint-armature) steadies articulations with
  very light links.
- [Collision Behavior Guide](../guides/collision_tuning.md) covers contact quality.
- [Joint Parameter Tuning: Robotiq 2F-85](../guides/gripper_tuning.md) tunes a gripper.
- [Known Physics Limitations](../guides/limitations.md) lists cases without a full fix.
- [Physics Solver](../simulation_setup/physics_scene.md#physics-solver),
  [Continuous Collision Detection](../simulation_setup/rigid_bodies.md#continuous-collision-detection),
  and [Rest and Contact Offsets](../simulation_setup/collision.md#rest-and-contact-offsets) list
  the settings.
- Next: [Fast Simulations](fast_simulations.md).
