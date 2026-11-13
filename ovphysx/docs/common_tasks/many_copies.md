<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Many Copies of a Scene

`clone` is a feature for robot training, which simulates many copies of one environment at the
same time. It makes the copies in the simulation only, and one read returns the state of every
copy.

![One environment, env0, holds a box above the shared ground plane. With one clone call, it becomes a 4 by 4 grid of 16 environments on that same ground. A read with the pattern /World/envs/*/Box returns 16 rows at 2.000 m. All 16 boxes then fall together, and a second read returns 0.500 m in every row.](images/guide_many_copies_grid.png)

## Environments and Copies

An environment is one self-contained setup, such as a robot and the objects it works with. Put
each environment under its own [prim](../reference/glossary.md#scenes-and-stages), such as `/World/envs/env0`. Keep everything that all
environments share, such as the ground plane, outside of them.

## Clone an Environment

This program, `many_copies_clone.py`, builds one environment with a falling box, clones it into a 4 by 4 grid, and reads
the box of every copy after 1 s:

```{literalinclude} code/many_copies_clone.py
:language: python
:start-after: [example-start]
:end-before: [example-end]
```

It prints:

```{literalinclude} code/many_copies_clone.out
:language: output
```

Notice the following:

- Each copy takes the target path you give it, and the objects inside keep their names below it,
  such as `/World/envs/env7/Box`.
- Each pose places the root of one copy, and `(0, 0, 0, 1)` as its rotation means no rotation.
  Objects inside the copy keep their places relative to the root.
- Clone after `attach_ovstage` and before the first step. ovphysx refuses a clone after the
  first step.
- The copies exist only in the simulation. ovphysx adds nothing to the [stage](../reference/glossary.md#scenes-and-stages), which still holds
  only `env0`.

## One Read for All Copies

`pattern="/World/envs/*/Box"` selects the box of every copy, because `*` stands for any one name,
from `env0` to `env15`.
[Selecting Objects](../simulation_data/selecting_objects.md#path-patterns) explains patterns, and
[Reading Values](../simulation_data/reading_values.md) shows how to tell the rows apart by path.

## Do Copies Collide?

| Simulation On | Objects of Different Copies |
| --- | --- |
| GPU, with the default GPU settings | Copies made by `clone` do not collide, even when they overlap |
| CPU | Collide when they overlap, and push each other away |

On the GPU, `clone` gives each copy its own environment ID, and objects with different IDs do not
collide. This holds while the scene keeps its default GPU settings.
[Scene Cloning](../developer_guide.md#scene-cloning) lists the conditions.

Either way, the copies still collide with the shared ground plane. Place the copies apart in any
case, so that your program also works on the CPU.

## More Detail

- [Cloning](../tutorials/cloning.md) has complete Python and C programs.
- [Scene Cloning](../developer_guide.md#scene-cloning) covers environments built from several
  clone calls, collisions between copies, and copies on the ovstage stage for a renderer.
- [Fast Simulations](fast_simulations.md) helps when many copies make each step slow.
