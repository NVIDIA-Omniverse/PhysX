<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Concepts

Every ovphysx program uses the same few ideas. This page explains them, with the program from
[Your First Simulation](first_simulation.md) as the example.

## How a Simulation Runs

A typical simulation follows this loop:

![Your program loads a USD scene into an ovstage stage and seals ordinal 1. ovphysx attaches to the stage, then steps again and again. After each step, your program reads positions as arrays, one row per object.](images/getting_started_simulation_loop.png)

1. Your program loads a USD scene into an ovstage stage and seals it, and ovphysx attaches to it.
2. Each step moves simulated time forward by an amount you choose, such as 1/60 s.
3. Between steps, your program reads values, such as positions, and writes values, such as forces.
4. To change the scene itself, your program edits the stage, seals a new ordinal, and calls
   `update_from_ovstage`, as [Changing the Scene](../common_tasks/changing_the_scene.md) shows.

## Stages and Ordinals

ovphysx reads the scene from an ovstage stage that your program creates and owns.

| Term | Meaning |
| --- | --- |
| Stage | A scene held in memory by ovstage, which your program loads and edits. |
| Ordinal | A number your program gives each batch of edits, higher for each new batch. |
| Seal | To mark every edit up to an ordinal as complete with `advance_write_floor`. ovphysx reads only sealed edits. |
| Prim | One element of a USD scene, such as a box or a joint, named by a path such as `/World/Box`. |
| API schema | A label on a prim that adds behavior, such as `PhysicsRigidBodyAPI`, which makes the prim a moving body. |

Ordinals put the edits in order, but the stage keeps no old copies of the scene.

ovphysx never writes to the stage on its own. For a renderer that reads the stage, write the
results into it, as [Showing the Simulation](../common_tasks/showing_the_simulation.md) shows.

## Steps and Tasks

Calls on one `PhysX` object run in the order your program makes them. Some calls queue their
work and return immediately:

| Call | What It Does |
| --- | --- |
| `step(dt)` | Queues one step of `dt` seconds and returns a task. |
| `wait_task(task)` | Waits for one task. It raises an error when the work failed. |
| `wait_all()` | Waits for every task that nothing has waited for yet. |
| `step_sync(dt)` | Steps and waits in one call. |

A **task** is a receipt for queued work. Steps, attaching, detaching, and scene updates queue
their work. Queries, reads, and writes run right away when nothing is queued before them.

Wait for each task only once. Use one `PhysX` object from one thread at a time.

## Reading and Writing State

A read asks for named values, called **attributes**, of a set of objects. For example,
reading `position` for these four boxes returns an array with one row per box:

![Four boxes fall from different heights. Next to them, a read of position returns four rows, one per box, and each row's z value changes as its box falls and lands.](images/getting_started_one_row_per_object.png)

A read has three parts:

1. A **query** picks the objects by type, such as `SimObjectType.RIGID_BODY`. It can also
   narrow them with a path pattern.
2. The read names the attributes, such as `position` and `linearVelocity`.
3. The result arrives in **groups**. Each group holds values for one attribute and the list
   of objects they belong to. An attribute can have several groups, so use all of them. With one
   `PhysicsScene`, as in these guides, each attribute arrives in one group, so short examples use
   `groups[0]`.

Use the group's list to tell which object each row belongs to. The order can differ from the
scene file.

`read_sync` does all three and waits. A write works the other way round: your program fills
arrays for the attributes it changes and commits them.
[Reading Values](../simulation_data/reading_values.md) and
[Writing Values](../simulation_data/writing_values.md) show both in more detail.

## CPU and GPU

By default, ovphysx uses a supported NVIDIA GPU when one is available, and otherwise uses the
CPU. The mode decides where most read results are:

| Mode | Where Physics Runs | Where Most Read Results Are |
| --- | --- | --- |
| Default, with an NVIDIA GPU | GPU | CPU memory |
| CPU | CPU | CPU memory |
| DirectGPU | GPU | GPU memory |

**DirectGPU** keeps results in GPU memory, which suits programs that process them on the GPU,
such as training with PyTorch. [Data on the GPU](../simulation_data/gpu_data.md) explains it.

To simulate on the CPU on a computer with an NVIDIA GPU, set the environment variable
`OVPHYSX_DISABLE_GPU=1` before your program starts.
[The Physics Scene](../building_scenes/physics_scene.md#settings-you-can-change) explains how a
scene can ask for the CPU.

### Faster GPU Loops

A step queued with `step` runs while your program does other work. A CUDA graph can replay
recorded GPU reads, your own GPU work, and writes with one call.
[Fast Simulations](../common_tasks/fast_simulations.md) and
[Data on the GPU](../simulation_data/gpu_data.md#replay-a-loop-with-a-cuda-graph) show how.

## Using the Results

In Python, every array that a read returns is a `warp.array` from NVIDIA Warp. For results in
CPU memory, `array.numpy()` gives a NumPy array.
[Pass Arrays On Without Copying](../simulation_data/gpu_data.md#pass-arrays-on-without-copying)
covers PyTorch and other libraries.

The arrays belong to the read. Use or copy them before its `with` block ends. `PhysX.destroy()`
fails while your program still holds a read's result or arrays, so read inside a function, as
`box_height` in [Your First Simulation](first_simulation.md) does, and return plain values.

## Next Steps

The guides build on these ideas, one topic per page, in
[Building Scenes](../building_scenes/loading_scenes.md),
[Simulation Data](../simulation_data/selecting_objects.md), and
[Common Tasks](../common_tasks/changing_the_scene.md).

The [Detailed Reference](../reference/index.md) describes every feature in full, and the
[Python API](../python_api.rst) and [C API](../api.md) references list every call.
