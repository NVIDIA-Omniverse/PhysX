<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Debugging

When something goes wrong, ovphysx tells you in two places: the exceptions that your program
receives, and the messages on the console. When the numbers alone do not explain a problem, draw
what PhysX sees, or record the simulation and replay it step by step.

![Three panels. On the left, ovphysx simulates the falling box while a REC label shows that recording is on. In the middle, each step adds to the timeline of the file falling_box.ovd, which holds 60 steps in 82 KB. On the right, a drawing of the replay: a playhead scrubs along the timeline, and the box appears at that step with its bounding box and its downward velocity.](images/guide_debugging_recording.png)

## Where Errors Appear

ovphysx reports errors as Python exceptions. When one appears depends on the call:

| Call | When the Error Appears | Exception |
| --- | --- | --- |
| A call that ovphysx rejects, such as one with a wrong argument | During the call | Usually `RuntimeError`, or `ValueError` or `TypeError` from checks in Python |
| A call that waits for its work, such as `step_sync` or `read_sync` | During the call | `RuntimeError` or `TaskError` |
| Queued work: `attach_ovstage`, `detach_ovstage`, `step`, and `update_from_ovstage` | When you wait for its task with `wait_task`, or at the next `wait_all()` if nothing waited for it | `TaskError` |

`TaskError` is a kind of `RuntimeError`, so `except RuntimeError` catches both. A `TaskError` lists
each failed [task](../reference/glossary.md#running-the-simulation) with its message in `error.errors`.

ovphysx also prints its warnings and errors on the console, in lines that start with `[Warning]`
or `[Error]`. They often explain the exception that follows.

## Common Mistakes

| Mistake | What You See | Fix |
| --- | --- | --- |
| Registering the PhysX schemas after loading a scene, or never | `OvstageError`: "USD schema definitions were already built", or `RuntimeError`: "the PhysX USD schema families were not both registered with ovstage" | Register the schemas before loading any scene, then restart the process. An attach that succeeds after the first error can still be missing physics settings. |
| Attaching before sealing the ordinal | `TaskError`: "Physics scan backend failed" | Call `stage.advance_write_floor(ordinal=1).wait()` before `attach_ovstage` |
| Misspelling an attribute name | `IndexError`: "list index out of range" at `result.groups[0]` | The read returns no groups and no error. Check the name in [Reading Values](../simulation_data/reading_values.md). |
| Keeping a read's arrays after its `with` block | `RuntimeError`: "Cannot destroy PhysX while output-read arrays or downstream views are alive" | Read inside a function and return plain numbers |
| Waiting for the same task twice | `RuntimeError`: "Task was never issued or was already collected" | Wait for each task once |

## Turn Up the Log Level

By default, ovphysx prints messages of level `WARNING` and above. To see more, set a more detailed
level at any point in your program:

```python
import ovphysx

ovphysx.set_global_config(ovphysx.GlobalConfigKey.LOG_LEVEL, ovphysx.LogLevel.INFO)
```

`LogLevel.VERBOSE` shows even more, and `LogLevel.NONE` silences the messages of ovphysx itself.
The level applies to the whole process.

## See What PhysX Sees

The debug view shows what PhysX works with, such as the collision shape of every object and every
contact. Use it when an object floats, sinks, or hits something you cannot see: the lines show the
shapes and contacts that PhysX actually uses. PhysX returns the lines, and your renderer draws them
over the scene:

![Two panels joined by an arrow. On the left, a read of DEBUG_RENDER returns lines grouped by color: 912 collision-shape lines and, after the box lands, 12 contact-point lines and 4 contact-normal lines. On the right, a renderer draws those lines in their colors: the falling box's magenta wireframe and a magenta circle of the ground plane. After the landing, it adds a red cross at each bottom corner of the box and four blue normals pointing up.](images/guide_debugging_debug_view.png)

To turn it on, write to the `DEBUG_RENDER` target: set `enabled` to 1, and each flag you want to a
value above 0. For some flags, such as `contactNormal`, the value is also the length of each line,
so a value of 1.5 draws 1.5 m normals. `scale`, 1 by default, multiplies every size. These flags help most:

| Flag | What It Draws |
| --- | --- |
| `collisionShapes` | The collision shape of every collider. A ground plane appears as circles. |
| `collisionAABBs` | The box, aligned with the world axes, that encloses each collider |
| `contactPoint` | A small cross at each point where two shapes touch |
| `contactNormal` | A line from each contact point in the direction that it pushes |
| `bodyAxes` | The x, y, and z axes of each rigid body |
| `bodyLinearVelocity` | A line along the velocity of each rigid body |
| `jointLocalFrames` | The local axes of each joint |

A read of the same target returns lines: `linePosition` holds the two ends of each line in world
space, and `lineColor` a `0xAARRGGBB` color for each end. Points and triangles come the same way.
This program, `debugging_debug_view.py`, turns on three flags for the falling box and counts the lines by color:

```{literalinclude} code/debugging_debug_view.py
:language: python
:start-after: [example-start]
:end-before: [example-end]
```

It prints:

```{literalinclude} code/debugging_debug_view.out
:language: output
```

Notice the following:

- The 912 magenta lines, `ffff00ff`, are collision shapes: 12 for the box and 900 for the ground
  plane. After the landing, 12 red lines, `ffff0000`, draw a cross at each of the four contact points,
  and 4 blue lines, `ff0000ff`, draw the 1.5 m contact normals.
- Contact lines appear only while the box is awake. Soon after it rests, it falls asleep, and they
  disappear.

[Debug Render](../read_write/attributes/debug_render.md) lists every flag. Its `scope` and
`cullingBox` settings limit drawing to some objects or to a region.

## Record and Replay With OmniPVD

OmniPVD, the PhysX Omniverse Visual Debugger, records what PhysX simulates at every step into a
`.ovd` file. A viewer replays the file, so you can stop at any step and look at shapes, contacts,
and velocities. This program, `debugging_recording.py`, records the falling box:

```{literalinclude} code/debugging_recording.py
:language: python
:start-after: [example-start]
:end-before: [example-end]
```

It prints:

```{literalinclude} code/debugging_recording.out
:language: output
```

Notice the following:

- `OVD_RECORDING_CAPABLE` records nothing by itself, but it costs some speed, so turn it on only
  when you record.
- Call `stop_recording` before you detach.
- A path that ends in `.ovd` names the file. Any other path names a folder, where each recording
  gets a new file named after the date and time.

To view the file, open an Omniverse Kit application such as Isaac Sim, enable its OmniPVD
extension (`omni.physx.pvd`), and import the `.ovd` file. The viewer must support the recording's
version, as [Capture Format and Compatibility](../tutorials/omnipvd_recording.md#capture-format-and-compatibility)
explains.

## More Detail

- [Error Handling](../developer_guide.md#error-handling) and
  [Task Receipts and Polling](../developer_guide.md#task-receipts-and-polling) cover errors in depth.
- [Logging](../developer_guide.md#logging) shows log callbacks.
- [Session Results](../read_write/index.md#session-results) shows how `read_sync` with `metadata=True`
  tells why an attribute returned no [group](../reference/glossary.md#reading-and-writing).
- [OmniPVD Recording](../tutorials/omnipvd_recording.md) covers live streaming and files.
- [Headless Capture and Investigation](../tutorials/omnipvd_recording.md#headless-capture-and-investigation)
  checks a recording without a viewer, using the installed `omnipvd_inspection.py` sample and the
  PhysX SDK's `ovd-inspect` tool.
- Next: [Stable Simulations](stable_simulations.md) fixes shaking and exploding objects.
