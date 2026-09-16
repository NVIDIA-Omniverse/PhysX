<!-- SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved. -->
<!-- SPDX-License-Identifier: Apache-2.0 -->

# Hello World -- Populate ovstage and Step

This tutorial shows the smallest end-to-end ovphysx workflow: create an instance,
populate an ovstage Stage from USD, attach it, step simulation, and clean up
resources. You can use this flow as the starting point for larger integrations.

## Prerequisites

- A USD stage to populate. This tutorial uses stages that ship with every
  package: `links_chain_sample.usda` for Python and
  `simple_physics_scene.usda` for C. They are under `ovphysx/samples/data/` in
  the wheel, `<sdk-root>/samples/data/` in the C/C++ SDK, and `tests/data/` in a
  repository checkout. No hand-authoring is needed.
- **GPU simulation (optional):** prebuilt packages require a CUDA-capable
  NVIDIA driver compatible with CUDA 12.8, but no CUDA Toolkit installation.
  Refer to the [Quickstart prerequisites](quickstart.md#prerequisites) for the
  versioned corresponding-driver table and source-build requirements.

## Code Language

### Python

Install the package first:

```bash
pip install ovphysx
```

This complete sample creates a `PhysX` instance, populates an ovstage `Stage`
from `links_chain_sample.usda`, runs one synchronous step, and releases the
stage and the instance in lifetime-safe order:

```{literalinclude} ../../tests/python_samples/hello_world.py
:language: python
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

The `attach_scene` helper registers with ovstage the codeless PhysX USD schemas
that ovphysx ships (`ovstage.population.register_usd_schemas()` with
`ovphysx.codeless_schema_root()`) before it creates and populates the stage;
ovphysx never registers them itself, and the registration must precede the first
population call in the process.

### C

Download the ovphysx SDK and matching native ovstage archive as described in
the [SDK Quickstart](quickstart.md), and extract them as separate package roots.

**CMakeLists.txt**

Every C sample uses `find_package(ovphysx)` and links against `ovphysx::ovphysx`. Here is the `CMakeLists.txt` for `hello_world_c`:

```{literalinclude} ../../tests/c_samples/hello_world_c/CMakeLists.txt
:language: cmake
```

Build by pointing `CMAKE_PREFIX_PATH` at both package roots (refer to
[SDK Quickstart](quickstart.md) for details).

**Source**

The C sample performs the same sequence against `simple_physics_scene.usda`:

```{literalinclude} ../../tests/c_samples/hello_world_c/main.c
:language: c
:start-after: [tutorial-start]
:end-before: [tutorial-end]
```

The `ovphysx_sample_attach_usd_with_ovstage()` helper from
`samples/c_samples/common/ovstage_sample.h` registers the codeless PhysX USD
schemas with ovstage first, passing the root from
`ovphysx_get_codeless_schema_root()` to
`ovstage_population_register_usd_schemas()`, and only then creates and
populates the stage.

## Result

After this tutorial, you can step the simulation from both Python and C and release all resources cleanly.
