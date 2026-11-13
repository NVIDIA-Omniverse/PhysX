<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Reading Values

A read copies values out of the simulation, such as where each body is and how fast it moves. This
page reads several attributes at once and shows how to tell which row belongs to which object.

![Three boxes move for 0.5 s: BoxA rests on the ground, BoxB flies in an arc, and BoxC falls. A read_sync call for position and linearVelocity then returns two groups, one per attribute, and each row of both groups lights up together with its box.](images/guide_reading_values_groups.png)

## One Read, Several Attributes

Pass a list of attribute names to read them together. The following program,
`reading_values_three_boxes.py`, simulates three boxes
for 0.5 s, reads `position` and `linearVelocity` of every rigid body, and prints them by path:

<details class="getting-started-code">
<summary>The imports and the scene, in USD</summary>

```{literalinclude} code/reading_values_three_boxes.py
:language: python
:start-after: [example-start]
:end-before: [scene-end]
```

</details>

```{literalinclude} code/reading_values_three_boxes.py
:language: python
:start-after: [scene-end]
:end-before: [example-end]
```

It prints:

```{literalinclude} code/reading_values_three_boxes.out
:language: output
```

Notice the following:

- Each [group](../reference/glossary.md#reading-and-writing)'s `tensors` hold the values of one attribute, and row *i* belongs to object *i* of
  its `prim_list`.
- `result.dictionary` turns the group's numeric IDs into text: `token_to_string` gives the attribute
  name, and `get_path_strings` gives the path of each row.
- The rows come back as BoxC, BoxB, BoxA, not in the order of the scene file. The program stores
  the values by path, so it can combine the two groups safely.
- `row.tolist()` copies each row. The end of the `with` block closes the read, and its arrays are
  no longer valid after that, so copy what you keep.

`read_sync` waits for the result before it returns. A loop that reads every step can keep a query
and use `physx.read` instead, as [Keeping a Query](selecting_objects.md#keeping-a-query) shows.

## Common Rigid Body Attributes

| Attribute | Values | Unit |
| --- | --- | --- |
| `position` | x, y, z of the body in world space | Scene length unit, such as m |
| `orientation` | Rotation as a quaternion, in x, y, z, w order | None |
| `linearVelocity` | x, y, z velocity of the center of mass | Length unit per second |
| `angularVelocity` | x, y, z rotation speed | Radians per second |
| `mass` | The body's mass | Scene mass unit, such as kg |

Angular velocity is in radians per second, even though USD files write it in degrees per second.
A body that has not turned has the orientation `(0, 0, 0, 1)`.

## Finding Attribute Names

Each object type has its own attributes. The `ovphysx.attributes` module lists them, with their
units and frames, without a scene. The following program,
`reading_values_attribute_names.py`, prints the names that a `RIGID_BODY`
read accepts:

```{literalinclude} code/reading_values_attribute_names.py
:language: python
:start-after: [example-start]
:end-before: [example-end]
```

It prints:

```{literalinclude} code/reading_values_attribute_names.out
:language: output
```

`representation="default"` limits the list to names that a read accepts. The
[attribute reference](../read_write/attributes/index.md) has the same information for every type,
and [Discovering Attribute Metadata](../read_write/discovery.md) shows more of the module.

A name that the type does not have is not an error. The read returns no group for it, so check
that each group you expect arrived.

## More Detail

- [Readable Data](../read_write/readable.md) lists every readable attribute of each object type.
- [Data Model and Semantics](../read_write/data_model.md) explains groups, frames, and units in full.
- [Data on the GPU](gpu_data.md) covers reads that stay in GPU memory.
- Next: [Writing Values](writing_values.md) changes values in the simulation.
