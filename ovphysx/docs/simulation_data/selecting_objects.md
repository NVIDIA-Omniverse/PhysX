<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Selecting Objects

Before your program reads or writes values, it picks the objects to act on. A
[query](../getting_started/concepts.md#reading-and-writing-state) picks them by type, and a path
pattern narrows them down to part of the scene.

![A scene tree holds World, Shelf with BoxA and BoxB, and Floor with Ground and Ball. Three patterns type themselves in turn, and the rigid bodies that each one selects light up: /World/Shelf/* selects the two boxes, /World/** selects the boxes and the ball but not the ground, and /World/Box* finds the boxes inside Shelf.](images/guide_selecting_objects_patterns.png)

## Object Types

A [query](../reference/glossary.md#reading-and-writing) selects objects of one type, a `SimObjectType` from `ovphysx.types`. The table lists the
most common types. The `ARTICULATION` types are for [articulations](../building_scenes/articulations.md), such as robots.

| Type | Selects |
| --- | --- |
| `RIGID_BODY` | Rigid bodies that are not part of an articulation, such as loose boxes and balls. |
| `ARTICULATION` | Whole articulations, one row each. |
| `ARTICULATION_LINK` | The links of articulations. |
| `ARTICULATION_JOINT` | The joints of articulations, with one value for each axis that can move. |
| `CONTACT_SENSOR` | Bodies that report the contact forces they receive. |
| `DEFORMABLE_VOLUME` | Soft bodies. They require a GPU. |
| `PARTICLE_SET` | Sets of particles. They require a GPU. |

Each type has its own attributes. [Object types](../read_write/index.md#object-types) lists every
type.

## Path Patterns

A path pattern selects objects by their place in the scene tree, using [prim](../reference/glossary.md#scenes-and-stages) paths such as
`/World/Shelf/BoxA`. Pass it as the `pattern` argument. ovphysx compares it with each path one
level at a time, so a part between two slashes matches one name:

| Pattern Part | Matches |
| --- | --- |
| `*` | Any characters within one name. `Box*` matches `BoxA` and `BoxB`. |
| `?` | Exactly one character. |
| `**` | Any number of levels, when it is a whole part, as in `/World/**`. |
| A name without wildcards | That exact name. |

The last name in a pattern is also searched for deeper in the tree, so `/World/Box*` finds
`/World/Shelf/BoxA`. [Path Patterns](../read_write/path_patterns.md) has the exact rules.

A pattern must start with `/`. A pattern that selects nothing is not an error: the query is empty.

## Trying Patterns

The following program, `selecting_objects_patterns.py`, loads the scene from the figure. For each of four patterns, it reads the
rigid bodies that the pattern selects and prints the path of each row:

<details class="getting-started-code">
<summary>The imports and the scene, in USD</summary>

```{literalinclude} code/selecting_objects_patterns.py
:language: python
:start-after: [example-start]
:end-before: [scene-end]
```

</details>

```{literalinclude} code/selecting_objects_patterns.py
:language: python
:start-after: [scene-end]
:end-before: [example-end]
```

It prints:

```{literalinclude} code/selecting_objects_patterns.out
:language: output
```

Notice the following:

- `/World/**` also matches the ground, but the query keeps only rigid bodies, and the ground is
  not one.
- The rows come back as `BoxB`, then `BoxA`, not in the order of the scene file, so tell objects
  apart by their paths, as [Reading Values](reading_values.md) shows.

## Keeping a Query

`read_sync` and `write_sync` create a new query, and match the pattern again, on every call. A
program that reads or writes the same objects at every step can create the query once, with
`query = physx.query(SimObjectType.RIGID_BODY, pattern="/World/Shelf/*").wait()`. It then passes
the query to `physx.read` and `physx.write` at every step, and calls `query.close()` at the end.
Reads and writes through a kept query also reuse their setup work, so each one costs less. A query
with a pattern keeps the objects it matched when you created it.
[The session model](../read_write/index.md#the-session-model) shows a complete loop.

## More Detail

- [Path Patterns](../read_write/path_patterns.md) gives the full syntax, such as `[0-9]` and
  `BoxA|BoxB`.
- [Selecting Objects With Queries](../read_write/index.md#selecting-objects--queries) covers
  queries in full, including explicit lists of prims and contact queries.
- Next: [Reading Values](reading_values.md) reads attributes and tells rows apart by their paths.
