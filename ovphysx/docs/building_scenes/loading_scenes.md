<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Loading Scenes

This page shows how your program gets a USD file, a URL, or USD text into ovphysx, and how to
check what ovphysx found in it.

![The USD file crates.usda flows into an ovstage stage that already knows the PhysX schemas. The stage is sealed at ordinal 1, ovphysx attaches to it, and a read lists the three rigid bodies that ovphysx found. The band at the bottom shows the call that reads their positions.](images/guide_loading_scenes_flow.png)

## Four Steps, in Order

With ovphysx installed as in [Install](../getting_started/install.md), every program loads a scene
in the same four steps:

1. **Register** the PhysX schemas with `register_usd_schemas`, once, before the first load.
2. **Load** the scene into an `ovstage.Stage` with `open_usd_with_desc`, or
   `open_usd_from_string_with_desc` for text, using
   `descs=physics_population_descs(rendering=True)` with the helper from `ovphysx.population`.
3. **Seal** [ordinal](../reference/glossary.md#scenes-and-stages) 1 with `stage.advance_write_floor(ordinal=1).wait()`.
4. **Attach** with `physx.attach_ovstage(stage, read_ordinal=1)` and wait for its [task](../reference/glossary.md#running-the-simulation).

Registering late or not at all leads to the errors in
[Common Mistakes](../common_tasks/debugging.md#common-mistakes).
[Concepts](../getting_started/concepts.md#stages-and-ordinals) explains stages, ordinals, and
sealing.

> **Why are the PhysX schemas a separate package?**
>
> The PhysX schemas hold no code. They are definitions that tell USD which PhysX settings a scene
> can hold. ovphysx is only one of the tools that read those settings: a USD editor or your own
> `usd-core` script reads them too. As a package of their own, the schemas give every tool in
> your program one shared copy, and tools that never simulate can use them without ovphysx.
> [Install](../getting_started/install.md) shows how to add them, for Python and for C.

## Load a File

The following program, `loading_scenes_file.py`, loads a scene file with three crates on the ground and prints the rigid
bodies that ovphysx found. Save the scene as `loading_scenes_crates.usda` in the same folder as
the program:

<details class="getting-started-code">
<summary>loading_scenes_crates.usda</summary>

```{literalinclude} code/loading_scenes_crates.usda
:language: usda
:lines: 1-52
```

</details>

```{literalinclude} code/loading_scenes_file.py
:language: python
:start-after: [example-start]
:end-before: [example-end]
```

It prints:

```{literalinclude} code/loading_scenes_file.out
:language: output
```

Notice the following:

- `open_usd_with_desc` takes the path as a string. A URL goes in the same place.
- The ground is a static collider, not a rigid body, so it is not in the list.
  [Rigid Bodies](rigid_bodies.md) explains the difference.

## Choose What to Load

Pass `descs=physics_population_descs(rendering=True)` to `open_usd_with_desc`.
The helper, imported from `ovphysx.population`, combines the physics population
description with the rendering description for meshes, lights, materials and cameras.
Use `physics_population_descs()` for physics alone, including native USD
scene-graph instances. Add rendering when a renderer shares the stage or the
scene uses point-instancer prototype geometry.

## Where Scenes Come From

`open_usd_with_desc` reads local files and URLs:

| Source | Example |
| --- | --- |
| Local file | `/home/me/scenes/crates.usda` |
| Omniverse Nucleus | `omniverse://server/path/scene.usd` |
| Amazon S3 | `https://my-bucket.s3.us-east-1.amazonaws.com/path/scene.usd` |
| Azure Blob Storage | `https://account.blob.core.windows.net/container/scene.usd` |

Write S3 locations as `https://` URLs, not `s3://` URIs. Public files need no setup. For a
private S3 bucket or Azure container, your program gives the Omniverse Client Library its
credentials before it loads the scene.
[Remote USD Loading](../developer_guide.md#remote-usd-loading) names the calls.

## Scenes Without a USD File

A scene can also be built without any USD file: the helpers in `ovphysx.population`, such as
`create_physics_scenes`, `create_static_plane_colliders`, and `create_rigid_body_cubes`, write
objects straight into the [stage](../reference/glossary.md#scenes-and-stages). [Changing the Scene](../common_tasks/changing_the_scene.md)
uses one to add a box while the simulation runs.

## More Detail

- [Physics population](../ovstage_integration.md#populating-physics-the-physics-population-description) explains how to
  populate physics with or without rendering.
- [Registering with ovstage](../physics_schemas.md#registering-with-ovstage) covers the schemas
  and late registration.
- [Changing the Scene](../common_tasks/changing_the_scene.md) edits a scene after it is loaded.
- Next: [The Physics Scene](physics_scene.md) sets gravity and other scene-wide settings.
