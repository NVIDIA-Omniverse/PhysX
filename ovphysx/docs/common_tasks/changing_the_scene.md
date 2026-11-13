<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Changing the Scene

Your program can change the scene while it runs: add an object, remove one, or change a setting
such as a body's mass. You make the change in the ovstage [stage](../reference/glossary.md#scenes-and-stages), then tell ovphysx to apply it.

![A timeline from 0 to 2 s starts with ordinal 1 sealed, and a box falls and lands on the ground. At 1 s, the simulation pauses while the program creates /World/Box2 directly in the stage at ordinal 2, seals ordinal 2, and calls update_from_ovstage(2, 2). Only then does the second box appear in the simulation and fall.](images/guide_changing_the_scene_timeline.png)

## Three Steps

With a scene attached, every change uses a new [ordinal](../reference/glossary.md#scenes-and-stages), the edit number from
[Concepts](../getting_started/concepts.md#stages-and-ordinals):

1. **Edit the stage** at an ordinal higher than any ordinal you sealed before.
2. **Seal** that ordinal with `stage.advance_write_floor`.
3. **Apply** the edits: wait for the [task](../reference/glossary.md#running-the-simulation) of `physx.update_from_ovstage(first, last)`, which takes
   the range of new ordinals.

Seal before you update. An update reads only sealed edits, and ovphysx does not read an ordinal
again after an update has covered it.

## Moving a Body Through the Stage

For this stage-edit workflow, use a CPU scene or GPU dynamics with CPU readback
(DirectGPU disabled). Edit the body's local transform, `omni:xform`, then seal and
apply the edit as above. `omni:resetXformStack` controls whether that transform
inherits its parents' transforms.

If `omni:fabric:worldMatrix` already exists, update it to the matching resolved
world pose at the same ordinal, including any affected descendants. Population
can create this column even when your application has never published it; leaving
it stale can keep the body at its old pose despite the local edit.

For DirectGPU pose control, use the `position` and `orientation` writes in
[Writing Values](../simulation_data/writing_values.md), after the first step.
These teleport the body. To drive a kinematic support that carries other bodies,
leave DirectGPU disabled and follow
[Kinematic Support Geometry](../simulation_setup/kinematic_support.md).

> **Changing only `omni:fabric:worldMatrix` does not move a body in physics.**
> That column holds a derived world pose; `update_from_ovstage` does not treat its changes
> as authored movement. See [Transform Inputs](../ovstage_integration.md#transform-inputs)
> for the distinction between triggering an update and reading its resolved pose.

## Add a Box While the Scene Runs

This program, `changing_the_scene_add_box.py`, drops a box, as in [Your First Simulation](../getting_started/first_simulation.md).
Halfway through its 2 s, it creates a second box directly in the stage at ordinal 2:

<details class="getting-started-code">
<summary>The imports and the starting scene, in USD</summary>

```{literalinclude} code/changing_the_scene_add_box.py
:language: python
:start-after: [example-start]
:end-before: [scene-end]
```

</details>

```{literalinclude} code/changing_the_scene_add_box.py
:language: python
:start-after: [scene-end]
:end-before: [example-end]
```

It prints:

```{literalinclude} code/changing_the_scene_add_box.out
:language: output
```

Notice the following:

- `create_rigid_body_cubes` from `ovphysx.population` writes the box straight into the stage at
  ordinal 2: its path, its size, and its starting pose. The pose is a 4 by 4 matrix whose last row holds
  the position. `ancestors=False` leaves the `/World` prim that is already in the stage as it is.
- The box needs no mass setting: ovphysx computes its mass from its size and a default density, which gives
  1,000 kg for this 1 m box. To set a mass, also pass `physics_mass_api={"mass": [2.0]}`.
- Right after the update, a read returns both boxes.

## Other Changes

Every change takes the same three steps. Only the edit differs:

| To | Edit the Stage With |
| --- | --- |
| Add an object | A helper from `ovphysx.population`, such as `create_rigid_body_cubes` |
| Change a value, such as `physics:mass` on a body with `PhysicsMassAPI` | `stage.write_attribute`, with an ovstage query for the object |
| Remove an object | `stage.delete_attributes`, with an ovstage query for the object and no attribute names |

A setting changes only when the object has the schema that defines it, such as `PhysicsMassAPI`
for `physics:mass`. A new mass, density, center of mass, or inertia takes effect at the next step,
not at the update.

Deleting only some attributes of an object that stays, by passing attribute names to `stage.delete_attributes`,
is not recommended: the value physics keeps for them is undefined (see
[Known limitations](../read_write/limitations.md)). Write the value you want instead.

## Changing the Scene or Writing Values

You can also change a running simulation by [writing values](../simulation_data/writing_values.md),
such as a velocity or a force:

| | Change the Scene | Write Values |
| --- | --- | --- |
| Use it to | Add or remove objects, or change their settings | Move, push, or steer objects, often at every step |
| What changes | The stage, then the simulation | The simulation only |
| When ovphysx applies it | At `update_from_ovstage`, but a mass change takes effect at the next step | When your program commits the write |
| Seen by other libraries that read the stage | Yes | No |

## More Detail

- [Deferred Mass Edits](../ovstage_integration.md#deferred-mass-edits-and-session-overrides) and
  [Changing a Prim's Type](../ovstage_integration.md#changing-a-prims-type-at-runtime) explain
  when mass edits take effect and how to change an object's type.
- [ovstage Physics Population Domain](../population/index.md) lists every prim type and setting
  that the `ovphysx.population` helpers write.

Next: [Contacts](contacts.md)
