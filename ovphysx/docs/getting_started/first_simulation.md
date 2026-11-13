<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Your First Simulation

This page drops a box onto the ground and prints its height while it falls. The program comes in
Python and in C. Follow [Install](install.md) first.

## The Scene

The scene is written in USD, the scene format that ovstage loads for ovphysx. It has a ground plane and a
box with 1 m sides, whose center starts 2 m above the ground. These parts make it a physics
scene:

| Scene Part | What It Does |
| --- | --- |
| `PhysicsScene` | Sets gravity and other simulation settings. Here, the default gravity pulls the box down the z axis at 9.81 m/s². |
| `PhysicsCollisionAPI` | Makes the ground and the box solid, so they collide. |
| `PhysicsRigidBodyAPI` | Makes the box respond to gravity and other forces. The ground does not have this API schema, so it stays still. |

The following figure shows what the program computes:

![Side view of the scene. A green box falls from a height of 2 m and lands on the ground, where its center rests at 0.5 m. Next to it, the program's seven output lines appear one by one as simulated time passes.](images/getting_started_falling_box.png)

## The Program

> **Why ovstage?**
>
> ovphysx does not open USD files itself. ovstage loads the scene once and keeps it in memory as
> a stage. ovphysx, a renderer such as ovrtx, and your own code can all work from that one copy
> instead of each loading the file. Nothing moves between them on its own: your program decides
> when each library reads the stage and what is written to it.

To read the box's height, the program uses one of these calls:

| Python Call | What It Does | C Calls Underneath |
| --- | --- | --- |
| `physx.query(type, pattern=...).wait()` | Picks the objects once and returns a query you can reuse | `ovphysx_query` |
| `physx.read(query, names).wait()` | Opens a read on a query you kept | `ovphysx_read` |
| `physx.read_sync(type, names, pattern=...)` | Shortcut: makes a query, reads every group, waits, and releases the query when the `with` block ends | both of the above, on every call |

The program loads the scene, simulates 1 s in steps of 1/60 s, and prints the box's
height every 10 steps. It comes in Python and in C:

```{eval-rst}
.. tab-set::
   :sync-group: language
   :class: getting-started-tabs

   .. tab-item:: Python
      :sync: python

      Save the program as ``falling_box.py``:

      .. literalinclude:: code/falling_box.py
         :language: python
         :start-after: [example-start]
         :end-before: [example-end]
         :caption: falling_box.py

      Run it from the folder where you saved it:

      .. code-block:: bash

         python falling_box.py

   .. tab-item:: C
      :sync: c

      Use the SDK from :doc:`Install <install>`. Save this build file as
      ``CMakeLists.txt``:

      .. literalinclude:: code/CMakeLists.txt
         :language: cmake
         :start-after: [example-start]
         :end-before: [example-end]
         :caption: CMakeLists.txt

      Save the program, which checks every call, as ``falling_box.c`` in the same folder:

      .. literalinclude:: code/falling_box.c
         :language: c
         :start-after: [example-start]
         :end-before: [example-end]
         :caption: falling_box.c

      Build and run it on Linux from that folder, with ``/path/to/ovphysx`` replaced by the folder
      where you extracted the SDK:

      .. code-block:: bash

         cmake -B build -S . -DCMAKE_PREFIX_PATH=/path/to/ovphysx
         cmake --build build
         PHYSX_USD_SCHEMAS_ROOT=~/physx-schemas ./build/falling_box

      On Windows with Visual Studio, build with ``cmake --build build --config Release``, set
      ``PHYSX_USD_SCHEMAS_ROOT``, and run ``build\Release\run_falling_box.cmd``.

      In C, your program also starts and stops the library with ``ovphysx_initialize`` and
      ``ovphysx_shutdown``. The ``done`` helper waits for background work and prints any error.
      A read returns a ``DLTensor``, an array described using the DLPack standard.
```

<div class="docs-portable-copy">

**Python.** Save the program as `falling_box.py`:

```{literalinclude} code/falling_box.py
:language: python
:start-after: [example-start]
:end-before: [example-end]
```

Run it from the folder where you saved it:

```bash
python falling_box.py
```

**C.** Use the SDK from [Install](install.md#install-the-c-sdk). Save this build file as
`CMakeLists.txt`:

```{literalinclude} code/CMakeLists.txt
:language: cmake
:start-after: [example-start]
:end-before: [example-end]
```

Save the program, which checks every call, as `falling_box.c` in the same folder:

```{literalinclude} code/falling_box.c
:language: c
:start-after: [example-start]
:end-before: [example-end]
```

Build and run it on Linux from that folder, with `/path/to/ovphysx` replaced by the folder where
you extracted the SDK:

```bash
cmake -B build -S . -DCMAKE_PREFIX_PATH=/path/to/ovphysx
cmake --build build
PHYSX_USD_SCHEMAS_ROOT=~/physx-schemas ./build/falling_box
```

On Windows with Visual Studio, build with `cmake --build build --config Release`, set
`PHYSX_USD_SCHEMAS_ROOT`, and run `build\Release\run_falling_box.cmd`.

In C, your program also starts and stops the library with `ovphysx_initialize` and
`ovphysx_shutdown`. The `done` helper waits for background work and prints any error.
A read returns a `DLTensor`, an array described using the DLPack standard.

</div>

After the startup messages, both programs print:

```{literalinclude} code/falling_box.out
:language: output
```

The program has three main parts:

1. **Prepare the scene.** Register the PhysX schemas, then load the scene into
   ovstage, the library that holds it in memory. Seal the loaded scene with
   `advance_write_floor`, then attach the stage to ovphysx.
2. **Move time forward.** Each `step_sync(1.0 / 60.0)` simulates 1/60 s and waits for the
   step to finish.
3. **Read the height.** `read_sync` returns the box's x, y, and z coordinates. `box_height`
   copies z into a plain number before leaving the `with` block.

The box comes to rest at 0.500 m because its center is half a side above the ground.

## Next Steps

[Concepts](concepts.md) explains the ideas behind these programs.
