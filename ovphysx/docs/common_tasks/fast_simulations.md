<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Fast Simulations

ovphysx can simulate very large scenes. In the following scene, it moves 100,000 boxes along
400 conveyor belts, with all of the work on the GPU:

```{eval-rst}
.. raw:: html

   <video class="conveyor-video" src="../_static/guide_fast_simulations_conveyors.mp4" poster="../_static/guide_fast_simulations_conveyors_poster.jpg" autoplay muted loop playsinline preload="auto" aria-label="The camera pulls back from a few conveyor belts carrying boxes to a factory floor of 400 conveyor belts, all moving 100,000 boxes at once."></video>
```

<div class="docs-portable-copy">

![A factory floor of conveyor belts, each carrying a line of boxes.](images/guide_fast_simulations_conveyors.png)

</div>

These tips make a simulation run faster. Before you change anything, time a few hundred steps of
your loop with `time.perf_counter()`, then time them again after each change.
[Profiling](../tutorials/profiling.md) shows which calls take the time. The numbers on this page
come from one computer with an NVIDIA RTX A6000. Yours will differ.

## Work While a Step Runs

`step_sync` waits for each step to finish. `step` queues the step and returns at once, so your
program can do its own work while PhysX simulates:

![Two timelines of four loops, measured with 4,000 boxes on the GPU. When the program waits for each step and then does 2 ms of its own work, the four loops take 18.2 ms. When it starts each step, does its work while the step runs, and then waits for the step, they take 10.2 ms, 8.0 ms sooner.](images/guide_fast_simulations_overlap.png)

Use that time for work that does not need the new results, such as preparing the next commands.
Then call `wait_task(task)` to report any error of the step. Calls on the same `PhysX` object, such
as reads, wait for the queued step anyway. Because `step` returns at once, time whole loops, not
the `step` call alone.

## Read Many Objects at Once

A read of 400 boxes takes about as long as a read of one box. This program, `fast_simulations_reads.py`, reads the positions of
400 boxes, first with one read, then with one read per box:

```{literalinclude} code/fast_simulations_reads.py
:language: python
:start-after: [example-start]
:end-before: [example-end]
```

It printed:

```{literalinclude} code/fast_simulations_reads.out
:language: output
```

One read of all the boxes was more than 100 times faster than 400 small reads. To read only some
objects, select them all with one pattern, as
[Selecting Objects](../simulation_data/selecting_objects.md#path-patterns) shows.

## More Tips

| Tip | Helps When | Read More |
| --- | --- | --- |
| Simulate on the GPU. | The scene has thousands of objects. With stacked boxes, the CPU was faster at 400 boxes and the GPU at 4,000. | [CPU and GPU](../getting_started/concepts.md#cpu-and-gpu) |
| Keep one query and read through it at every step. | You read the same objects at every step. Each read took about a third less time. | [Keeping a Query](../simulation_data/selecting_objects.md#keeping-a-query) |
| Keep the results in GPU memory with DirectGPU, and replay repeated GPU work with a CUDA graph. | Your code works on the results on the GPU. | [Data on the GPU](../simulation_data/gpu_data.md), [CUDA graphs](../simulation_data/gpu_data.md#replay-a-loop-with-a-cuda-graph) |
| Use spheres, capsules, and boxes before [convex hulls](../building_scenes/colliders_and_materials.md#meshes), and convex hulls before triangle and [signed distance field (SDF)](../building_scenes/colliders_and_materials.md#meshes) meshes. | Objects have detailed shapes. Simple shapes cost the least. | [Colliders](../simulation_setup/collision.md#performance-and-stability-tips) |
| Cap the solver iterations with `physxScene:maxPositionIterationCount`. | The motion still looks right with fewer. Rigid bodies use 16 by default, and a cap of 4 cut the step time of 4,000 boxes by a quarter to a third. | [Physics Solver](../simulation_setup/physics_scene.md#physics-solver) |
| Keep shapes from overlapping needlessly. | Every touching pair that can collide costs time. Shapes that overlap at the start also jump apart. | [What to Change](stable_simulations.md#what-to-change) |

## More Detail

- [Simulation Performance Guide](../guides/performance.md) covers threads, several GPUs, and
  deformables.
- [Execution Model](../developer_guide.md#execution-model) explains which calls queue their work.
