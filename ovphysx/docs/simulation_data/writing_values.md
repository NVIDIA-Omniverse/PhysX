<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Writing Values

A write changes the simulation from your program. It can move a body, set its velocity, or push it
with a force. This page shows how a write works and how long each kind of value lasts.

## Open, Fill, Commit

With a scene attached to your `PhysX` object, a write takes three steps:

![Three panels show one write in order. Open: ovphysx returns a group with the object list /World/VelocityBox and an unfilled linearVelocity array. Fill: the program puts 1.000, 0.000, 0.000 into the array. Commit: a read now returns a velocity of 1.000 m/s, while the box is still at x = 0.000 m because no step has run.](images/guide_writing_values_open_fill_commit.png)

1. **Open** the write for one or more attributes. Like a read, it returns [groups](../reference/glossary.md#reading-and-writing), each with an
   array that has one row per selected object.
2. **Fill** every row of each array with the values you want.
3. **Commit** each group. ovphysx applies the values at once, before the next step.

`write_sync` opens the write and returns it with every group in its `groups` list. `commit_sync`
commits one group and waits. A loop that writes every step can keep a query and use `physx.write`
instead, as [Keeping a Query](selecting_objects.md#keeping-a-query) shows.

A commit writes every row of its group. The arrays do not start with the current values, so fill
every row. A group that you do not commit is discarded.

To write only some rows of a group, commit it with a mask: `session.commit_sync(group, mask=mask)`.
The mask holds one `bool` or `uint8` value per row, on the same device as the group's arrays
(`group.tensors[0].device`), and rows whose value is zero are not written. When you know the
objects in advance, a `pattern` is simpler, as in the program in [A Velocity and a Force](#a-velocity-and-a-force).

## A Velocity and a Force

The following program, `writing_values_push.py`, writes only once: a velocity of 1 m/s to one box
and a force of 30 N to another.
Then it steps for 1 s:

![Two 1 kg boxes start at rest with gravity off. A velocity write sets VelocityBox moving at 1 m/s at once. A 30 N force pushes ForceBox during step 1 only, after which its velocity stays at 0.5 m/s. After 30 steps, VelocityBox has moved 0.5 m and ForceBox 0.246 m.](images/guide_writing_values_force_vs_velocity.png)

<details class="getting-started-code">
<summary>The imports and the scene, in USD</summary>

```{literalinclude} code/writing_values_push.py
:language: python
:start-after: [example-start]
:end-before: [scene-end]
```

</details>

```{literalinclude} code/writing_values_push.py
:language: python
:start-after: [scene-end]
:end-before: [example-end]
```

It prints the x position and x velocity of both boxes, after the writes (step 0) and after some of
the steps:

```{literalinclude} code/writing_values_push.out
:language: output
```

Notice the following:

- VelocityBox moves at 1 m/s right after the commit, before the first step.
- ForceBox speeds up during step 1 only: a force of 30 N on 1 kg for 1/60 s adds 0.5 m/s. To keep
  pushing, write the force before every step.

## How Long Values Last

| Kind of Value | Examples | After the Commit |
| --- | --- | --- |
| State | `position`, `orientation`, `linearVelocity`, `angularVelocity` | Changes at once. The simulation goes on from the new value, so gravity and collisions keep changing it. |
| Settings and targets | `mass`, friction, joint drive targets | Stay until you write them again. |
| Forces | `force`, and `wrench`, a force and a torque together | Act during the next step only. |

Writing `position` keeps the body's orientation, and writing `orientation` keeps its position.

## What You Cannot Write

Some attributes are read-only, such as `linearAcceleration` and `isSleeping`. Opening a write for a
read-only or unknown name raises `TaskError` and writes nothing. Forces work the opposite way:
you can write them, but a read returns no group for them.

In [DirectGPU](../reference/glossary.md#cpu-and-gpu) mode, step once before you write values that live in GPU memory, such as positions and
velocities, as [Data on the GPU](gpu_data.md) explains.

## More Detail

- [Writable Data](../read_write/writable.md) lists every writable attribute, with the exact rules
  for timing, row masks, and what each value overwrites.
- Next: [Data on the GPU](gpu_data.md) keeps results in GPU memory for GPU libraries.
