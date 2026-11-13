<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Showing the Simulation

ovphysx computes motion but draws nothing. To see the motion, your program passes the results to
a renderer, a library that turns a scene into pictures.

![Three panels joined by arrows. On the left, ovphysx steps the simulation. In the middle, the ovstage stage lists six boxes with the height from each box's world transform. The heights fall as the program writes each step and seals a new ordinal. On the right, a renderer shows the same six boxes falling onto a dark floor and coming to rest.](images/guide_showing_the_simulation_flow.png)

## Two Ways

| | Write Into the Stage | Hand the Arrays to Your Renderer |
| --- | --- | --- |
| After each step | Call `step_and_write_to_ovstage`, which also steps | Read `position` and `orientation` |
| Renderer | One that reads the ovstage stage, such as ovrtx, NVIDIA's RTX rendering library | Any renderer, such as your own engine |
| Shapes, lights, and cameras | Already in the stage | Your renderer loads or builds them |

## Write the Results Into the Stage

ovphysx never writes to the stage on its own, as [Concepts](../getting_started/concepts.md#stages-and-ordinals)
explains. With a scene attached, the helper `step_and_write_to_ovstage` from `ovphysx.utils` does it in one call:

1. It steps the simulation by `dt`.
2. It writes each body's new pose into the stage, where a renderer reads it.
3. It seals the [ordinal](../reference/glossary.md#scenes-and-stages) that you pass.

This program, `showing_the_simulation_write_back.py`, runs the falling box from [Your First Simulation](../getting_started/first_simulation.md),
writes its pose into the stage after every step, and reads it back from the stage:

```{literalinclude} code/showing_the_simulation_write_back.py
:language: python
:start-after: [example-start]
:end-before: [stage-read-start]
```

<details class="getting-started-code">
<summary>How the program reads the stage</summary>

```{literalinclude} code/showing_the_simulation_write_back.py
:language: python
:start-after: [stage-read-start]
:end-before: [stage-read-end]
```

</details>

```{literalinclude} code/showing_the_simulation_write_back.py
:language: python
:start-after: [stage-read-end]
:end-before: [example-end]
```

It prints:

```{literalinclude} code/showing_the_simulation_write_back.out
:language: output
```

Notice the following:

- The heights match those of [Your First Simulation](../getting_started/first_simulation.md), because the stage now holds the simulated pose.
- Each call writes at a new ordinal, here 2 to 61. Give every call an ordinal higher than the last.
- `physics_population_descs(rendering=True)` also loads the meshes, lights, materials, and cameras that a renderer
  needs.

The helper sets the transform of each body only. Prims nested under a body, such as the meshes of
a robot link, keep the transforms they had.

If your program also edits the scene, never pass these output ordinals to `update_from_ovstage`.
[The Key Principle](../ovstage_integration.md#the-key-principle-physics-must-not-get-its-own-changes)
explains why.

## Hand the Arrays to Your Own Renderer

After each step, read `position` and `orientation` of every body, as
[Reading Values](../simulation_data/reading_values.md) shows, and move the matching object in your
renderer. Keep these points in mind:

- `orientation` is a quaternion in x, y, z, w order. Check which order your renderer expects.
- The results hold no scale. Keep each object's size from your scene.

[Rendering Handoff](../tutorials/render_handoff.md) streams these arrays to the Rerun viewer. Its
sample is in the source repository, not in the Python package.

## More Detail

- [Python Utility](../ovstage_integration.md#python-utility-one-call-for-step--read--write-back)
  covers the helper in full.
- [C++ Utility](../ovstage_integration.md#c-utility-publish-fixed-world-transforms) covers a C++
  helper that writes the world transforms.
- [Signed Scale and Body Orientation](../read_write/data_model.md#signed-scale-and-body-orientation)
  combines poses with scaled shapes.
- Next: [Debugging](debugging.md) shows what to do when something goes wrong.
